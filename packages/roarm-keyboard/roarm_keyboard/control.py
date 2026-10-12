"""RoArm-M3 的按键状态、单关节目标更新和单路通信。

Controller 处理按下、松开及模式切换；Sender 在一个线程中读取反馈和发送指令。
关节模式以度为单位更新 T121 目标，坐标模式保留固件的 T123 连续运动。
内部夹爪角度沿用数据集的 angular_direct 定义，发送时转换为原厂角度。
"""

import json
import math
import time
from collections import deque
from concurrent.futures import Future
from queue import Queue
from threading import Condition, RLock, Thread
from urllib.parse import urlencode
from urllib.request import ProxyHandler, build_opener

import serial


PAIRS = [("q", "a"), ("w", "s"), ("e", "d"), ("r", "f"), ("t", "g"), ("y", "h")]
# enumerate 的起始值设为 1，直接对应原厂关节编号和增加/减少方向。
KEYS = {key: (axis, direction) for axis, pair in enumerate(PAIRS, 1)
        for direction, key in enumerate(pair, 1)}
# 顺序对应底座、肩、肘、腕倾、腕转、夹爪；夹爪使用 angular_direct 角度。
JOINT_LIMITS = [(-180, 180), (-90, 90), (0, 180), (-90, 90), (-90, 90), (0, 100)]
JOINT_PERIOD = 0.1


class Controller:
    """把按键状态转换为关节运动请求或坐标连续运动指令。"""

    def __init__(self, send, mode=0, speed=3, joint_motion=None):
        """接收发送函数及关节运动回调；关节速度单位为度/秒。"""
        self.send = send
        self.joint_motion = joint_motion
        self.mode = 0
        self.speed = self.validate_speed(speed)
        self.held = set()
        self.muted = set()
        self.active = {}
        self.lock = RLock()
        if mode not in (0, 1):
            raise ValueError("运动模式请使用 0 或 1")
        self.mode = mode

    @staticmethod
    def validate_speed(value):
        """将输入转换为有限浮点数，返回 1～20 范围内的速度。"""
        value = float(value)
        if not math.isfinite(value) or not 1 <= value <= 20:
            raise ValueError("速度系数请使用 1～20")
        return value

    def command(self, axis, direction):
        """构造坐标运动或清理旧连续运动状态的 T123 指令。"""
        return {"T": 123, "m": self.mode, "axis": axis, "cmd": direction, "spd": self.speed}

    def update_axis(self, axis):
        """合并同轴正反键；关节模式交给通信线程持续更新目标。"""
        positive, negative = PAIRS[axis - 1]
        keys = self.held - self.muted
        direction = 0 if (positive in keys) == (negative in keys) else (1 if positive in keys else 2)
        if self.active.get(axis, 0) != direction:
            if self.mode == 0:
                if self.joint_motion is None:
                    raise ValueError("关节模式请配置关节运动回调")
                self.joint_motion(axis, direction, self.speed)
            else:
                self.send(self.command(axis, direction))
            if direction:
                self.active[axis] = direction
            else:
                self.active.pop(axis, None)

    def press(self, key):
        """记录首次按下，重复按下沿用已有运动请求。"""
        with self.lock:
            key = key.lower()
            if key in KEYS and key not in self.held:
                self.held.add(key)
                self.update_axis(KEYS[key][0])

    def release(self, key):
        """移除松开的键，重新计算对应轴的方向或持位请求。"""
        with self.lock:
            key = key.lower()
            if key in KEYS:
                self.held.discard(key)
                self.muted.discard(key)
                self.update_axis(KEYS[key][0])

    def stop(self):
        """结束目标更新，并逐轴清理固件已有的连续运动方向。"""
        with self.lock:
            self.muted.update(self.held)
            self.active.clear()
            for axis in range(1, 7):
                if self.joint_motion is not None:
                    self.joint_motion(axis, 0, self.speed)
                self.send(self.command(axis, 0))

    def home(self):
        """结束当前运动后发送初始位置指令，按住的键等待松开后再使用。"""
        with self.lock:
            self.stop()
            self.send({"T": 100})

    def set_mode(self, mode):
        """切换前结束旧模式；0 表示关节角度，1 表示末端坐标。"""
        if mode not in (0, 1):
            raise ValueError("运动模式请使用 0 或 1")
        with self.lock:
            if mode != self.mode:
                self.stop()
                self.mode = mode

    def set_speed(self, value):
        """速度变化时结束当前运动，让后续按键使用新速度。"""
        speed = self.validate_speed(value)
        with self.lock:
            if speed != self.speed:
                self.stop()
                self.speed = speed


class Link:
    """封装 USB、HTTP 和模拟通信，统一输出六轴反馈角度。"""
    def __init__(self, port=None, baudrate=115200, host=None, dry_run=False):
        self.serial = None
        self.host = host
        self.dry_run = dry_run
        self.feedback = None
        self.feedback_version = 0
        self.receive_buffer = bytearray()
        self.opener = build_opener(ProxyHandler({})) if host else None
        if port and not host and not dry_run:
            self.serial = serial.Serial(baudrate=baudrate, timeout=0.05,
                                        write_timeout=0.3, exclusive=True)
            self.serial.port = port
            self.serial.rts = False
            self.serial.open()

    def send(self, command):
        """发送完整 JSON；HTTP 的 T105 响应也进入反馈解析。"""
        data = json.dumps(command, separators=(",", ":"))
        if self.dry_run:
            print(data, flush=True)
            state = list(self.feedback["state"]) if self.feedback else [0, 0, 90, 0, 0, 0]
            if command["T"] == 121:
                axis = command["joint"]
                state[axis - 1] = 180 - command["angle"] if axis == 6 else command["angle"]
            elif command["T"] == 100:
                state = [0, 0, 90, 0, 0, 0]
            if command["T"] in (105, 121, 100):
                self.feedback = {"state": state, "received_at": time.perf_counter(), "raw": {}}
                self.feedback_version += 1
        elif self.host:
            url = f"http://{self.host}/js?{urlencode({'json': data})}"
            with self.opener.open(url, timeout=15 if command["T"] == 100 else 0.4) as response:
                body = response.read(4096)
            if command["T"] == 105:
                self.parse_feedback(body)
        else:
            if self.serial is None:
                raise ValueError("请指定串口或机械臂地址")
            payload = (data + "\n").encode()
            if command["T"] == 105:
                self.serial.reset_input_buffer()
                self.receive_buffer.clear()
            written = self.serial.write(payload)
            if written != len(payload):
                raise OSError("串口写入长度检查")
            self.serial.flush()

    def parse_feedback(self, line):
        """解析 T1051 弧度反馈，转换为度，并保留接收时间。"""
        try:
            start, end = line.find(b"{"), line.rfind(b"}")
            if start < 0 or end < start:
                return
            value = json.loads(line[start:end + 1])
            if value.get("T") != 1051:
                return
            state = [math.degrees(float(value[key])) for key in ("b", "s", "e", "t", "r", "g")]
            state[5] = 180 - state[5]
            if not all(math.isfinite(angle) for angle in state):
                return
            self.feedback = {"state": state, "received_at": time.perf_counter(), "raw": value}
            self.feedback_version += 1
        except (ValueError, KeyError, TypeError):
            return

    def drain(self):
        """接收串口中的完整行，保留尚未完成的一行等待下一次读取。"""
        if self.serial is not None and self.serial.in_waiting:
            self.receive_buffer.extend(self.serial.read(min(self.serial.in_waiting, 8192)))
            while b"\n" in self.receive_buffer:
                line, _, remainder = self.receive_buffer.partition(b"\n")
                self.receive_buffer = bytearray(remainder)
                self.parse_feedback(line)
            if len(self.receive_buffer) > 8192:
                self.receive_buffer.clear()

    def close(self):
        if self.serial is not None:
            self.serial.close()


class Sender:
    """统一发送、反馈和目标更新，GUI 与采集线程通过请求共用连接。

    每个关节独立保存目标。初次按下读取实际角度作为起点，之后沿用已发送目标；
    松手读取新反馈并发送该关节的持位目标。反馈等待期间的新按键以最新请求为准。
    """

    def __init__(self, link):
        self.link = link
        self.condition = Condition()
        self.pending = {}
        self.closing = False
        self.error = None
        self.errors = Queue()
        self.queries = deque()
        self.awaiting = None
        self.feedback_mark = 0
        self.query_started = 0
        self.motion_snapshot = {}
        self.joints = {}
        self.command_log = None
        self.sending = False
        self.thread = Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, command):
        """排队发送完整指令；T121 按 joint 区分，T123 按 mode/axis 区分。"""
        with self.condition:
            if self.closing or (self.error is not None and command.get("cmd") != 0):
                return
            if command["T"] == 105:
                # 普通发送入口的反馈请求也排入查询队列，保持一问一答的顺序。
                self.queries.append(Future())
                self.condition.notify()
                return
            if command["T"] == 123 and command.get("cmd") == 0:
                self.pending.pop((100, None, None), None)
            if command["T"] == 100:
                # 回位接管目标，旧反馈回复继续服务采样，旧关节请求结束。
                self.joints.clear()
                self.pending = {key: value for key, value in self.pending.items() if value["T"] != 121}
            key = (121, None, command.get("joint")) if command["T"] == 121 else \
                  (command["T"], command.get("m"), command.get("axis"))
            self.pending[key] = dict(command)
            self.condition.notify()

    def move_joint(self, axis, direction, speed):
        """请求一个关节增加、减少或持位；速度输入为度/秒，立即返回。"""
        if axis not in range(1, 7) or direction not in (0, 1, 2):
            raise ValueError("关节序号请使用 1～6，方向请使用 0、1、2")
        speed = Controller.validate_speed(speed)
        with self.condition:
            if self.closing or self.error is not None:
                return
            if direction == 0 and axis not in self.joints:
                return
            # 替换整个请求对象；回复时用对象身份核对，结束后的旧请求自然失效。
            self.joints[axis] = {"direction": direction, "speed": speed, "target": None,
                                 "next_at": 0, "last_at": 0}
            self.pending.pop((121, None, axis), None)
            self.log_event("motion", mode=0, joint=axis, direction=direction, speed=speed)
            self.condition.notify()

    def start_command_log(self):
        """开始记录本段操作和实际发送的 JSON，时间使用单调时钟。"""
        with self.condition:
            self.command_log = []

    def finish_command_log(self):
        """结束本段记录；已请求停止时，短等持位与停止指令发送完成。"""
        with self.condition:
            if not any(state["direction"] for state in self.joints.values()):
                self.condition.wait_for(
                    lambda: self.error is not None or (not self.pending and not self.joints and not self.sending),
                    timeout=1.5,
                )
            result = self.command_log or []
            self.command_log = None
            return result

    def log_event(self, kind, **fields):
        """在持有 condition 的调用处追加事件，保留输入字段。"""
        if self.command_log is not None:
            self.command_log.append({"time": time.perf_counter(), "kind": kind, **fields})

    @staticmethod
    def joint_command(axis, state):
        """将内部目标转换为 T121；夹爪发送原厂角度，速度取正整数。"""
        angle = 180 - state["target"] if axis == 6 else state["target"]
        return {"T": 121, "joint": axis, "angle": angle,
                "spd": max(1, math.ceil(state["speed"])), "acc": 10}

    def next_joint_command(self, now, holds_only=False):
        """生成到期目标，每次增量按 0.1 秒上限计算，保存后再发送。"""
        for axis, state in list(self.joints.items()):
            if state["target"] is None:
                continue
            if state["direction"] == 0:
                self.joints.pop(axis)
                return self.joint_command(axis, state)
            if holds_only or now < state["next_at"]:
                continue
            feedback = self.link.feedback
            if feedback is None or now - feedback["received_at"] > 0.6:
                continue
            elapsed = min(JOINT_PERIOD, max(0, now - state["last_at"]))
            sign = 1 if state["direction"] == 1 else -1
            if axis == 6:
                # Y 增加原厂夹爪角度；angular_direct 对应减少。
                sign = -sign
            low, high = JOINT_LIMITS[axis - 1]
            state["target"] = min(high, max(low, state["target"] + sign * state["speed"] * elapsed))
            state["last_at"] = now
            state["next_at"] = now + JOINT_PERIOD
            return self.joint_command(axis, state)
        return None

    def needs_feedback(self, now):
        """运动前、持位前及运动期间定期获取新反馈。"""
        if not self.joints:
            return False
        if any(state["target"] is None for state in self.joints.values()):
            return True
        feedback = self.link.feedback
        return feedback is None or now - feedback["received_at"] >= 0.2

    def resolve_feedback(self):
        """完成一条反馈请求，只给仍然有效的关节请求设置起点。"""
        if self.awaiting is None:
            return
        if self.link.feedback_version > self.feedback_mark:
            feedback = dict(self.link.feedback)
            now = time.perf_counter()
            for axis, state in self.motion_snapshot.items():
                if self.joints.get(axis) is not state or state["target"] is not None:
                    continue
                angle = float(feedback["state"][axis - 1])
                low, high = JOINT_LIMITS[axis - 1]
                if state["direction"] and not low - 1 <= angle <= high + 1:
                    raise ValueError(f"第 {axis} 关节当前角度 {angle:.2f}°，请先检查姿态范围")
                state["target"] = min(high, max(low, angle)) if state["direction"] else angle
                state["last_at"] = now - JOINT_PERIOD
                state["next_at"] = now
            if not self.awaiting.done():
                self.awaiting.set_result(feedback)
            self.awaiting = None
            self.motion_snapshot = {}
        elif time.perf_counter() - self.query_started > 0.6:
            raise TimeoutError("请检查机械臂角度反馈后继续运动")

    def read_feedback(self, timeout=0.6):
        """请求新反馈并等待结果；由通信线程处理，采样线程可同步调用。"""
        if not (getattr(self.link, "serial", None) is not None or getattr(self.link, "host", None)
                or getattr(self.link, "dry_run", False)):
            raise ValueError("角度读取请配置 USB 或 HTTP 连接")
        future = Future()
        with self.condition:
            if self.closing:
                raise RuntimeError("串口连接已结束")
            if self.error is not None:
                raise self.error
            self.queries.append(future)
            self.condition.notify()
        try:
            return future.result(timeout=timeout)
        except TimeoutError:
            with self.condition:
                future.cancel()
            raise TimeoutError("请检查机械臂角度反馈") from None

    def joints_angle_get(self):
        return self.read_feedback()["state"]

    def finish_queries(self, error, keep_inflight=False):
        """结束等待者；keep_inflight 保留正在传输的回复用于区分新旧请求。"""
        futures = list(self.queries)
        self.queries.clear()
        if self.awaiting is not None:
            futures.append(self.awaiting)
            if not keep_inflight:
                self.awaiting = None
                self.motion_snapshot = {}
        for future in futures:
            if not future.done():
                future.set_exception(error)

    def run(self):
        """执行反馈、持位、普通指令和定时运动，每次只进行一条通信。"""
        try:
            while True:
                try:
                    with self.condition:
                        self.resolve_feedback()
                        if self.closing and not self.pending and not self.joints:
                            break
                        now = time.perf_counter()
                        key = next((key for key, value in self.pending.items() if value.get("cmd") == 0), None)
                        command = self.pending.pop(key) if key is not None else self.next_joint_command(now, holds_only=True)
                        # 待读取持位角度时，坐标运动等指令等持位完成后继续。
                        holding = any(state["direction"] == 0 for state in self.joints.values())
                        if command is None and self.awaiting is None and self.needs_feedback(now):
                            command = {"T": 105}
                        if command is None and self.pending and not holding:
                            command = self.pending.pop(next(iter(self.pending)))
                        if command is None:
                            command = self.next_joint_command(now)
                        if command is None and self.awaiting is None:
                            while self.queries and self.queries[0].cancelled():
                                self.queries.popleft()
                            if self.queries:
                                command = {"T": 105}
                        if command is not None and command["T"] == 105:
                            while self.queries and self.queries[0].cancelled():
                                self.queries.popleft()
                            self.awaiting = self.queries.popleft() if self.queries else Future()
                            self.feedback_mark = self.link.feedback_version
                            self.query_started = now
                            self.motion_snapshot = dict(self.joints)
                        if command is None:
                            self.condition.wait(timeout=0.01 if self.awaiting is not None or self.joints else 0.05)
                        self.sending = command is not None
                    if command is not None:
                        self.link.send(command)
                        with self.condition:
                            self.log_event("command", command=dict(command))
                            self.sending = False
                            self.condition.notify_all()
                    self.link.drain()
                    with self.condition:
                        self.resolve_feedback()
                except Exception as error:
                    with self.condition:
                        if self.error is None:
                            self.error = error
                            self.errors.put(error)
                            self.log_event("error", message=str(error))
                        self.pending = {key: cmd for key, cmd in self.pending.items() if cmd.get("cmd") == 0}
                        self.joints.clear()
                        self.sending = False
                        self.finish_queries(error)
                        self.condition.notify_all()
        finally:
            with self.condition:
                self.finish_queries(RuntimeError("串口连接已结束"))
            self.link.close()

    def close(self):
        """先完成活动轴持位和连续运动停止，再释放连接。"""
        with self.condition:
            self.pending = {key: cmd for key, cmd in self.pending.items() if cmd.get("cmd") == 0}
            self.closing = True
            # 先消化原在途回复，再读取新的持位角度，保持请求与回复的顺序。
            self.finish_queries(RuntimeError("串口连接已结束"), keep_inflight=True)
            for axis, state in list(self.joints.items()):
                self.joints[axis] = {**state, "direction": 0, "target": None}
            self.condition.notify()
        self.thread.join(timeout=21)
        if self.thread.is_alive():
            raise TimeoutError("请检查连接，并使用机械臂电源开关停止运动")
