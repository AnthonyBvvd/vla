"""采集 5 Hz 图像、关节角度与操作记录，保存原生 LeRobot 数据集。"""

from contextlib import ExitStack
import json
from pathlib import Path
from queue import Queue
from threading import Event, Thread
import time


def open_camera(settings):
    """读取 front 相机配置，返回已连接的 LeRobot 相机。"""
    import draccus
    from lerobot.cameras import CameraConfig, make_cameras_from_configs
    from lerobot.cameras.opencv import OpenCVCameraConfig

    config = draccus.decode(CameraConfig, settings["cameras"]["front"])
    camera = make_cameras_from_configs({"front": config})["front"]
    camera.connect()
    return camera


class Recorder:
    """在独立线程采样，通过同一通信对象读取角度与操作记录。"""

    def __init__(self, sender, settings, root, task, seconds=20, countdown=5,
                 repo_id="local/roarm_keyboard"):
        self.sender = sender
        self.settings = settings
        self.root = Path(root).resolve()
        self.task = task
        self.seconds = seconds
        self.countdown = countdown
        self.repo_id = repo_id
        self.events = Queue()
        self.stop_event = Event()
        self.thread = None
        self.state = "idle"
        self.stop_motion = lambda: None
        self.last_episode = None

    @property
    def active(self):
        """返回采样和保存线程是否仍在运行。"""
        return self.thread is not None and self.thread.is_alive()

    def emit(self, text, kind="status"):
        """将进度送入界面消息队列，并同步输出到终端。"""
        self.events.put({"kind": kind, "text": text})
        print(text, flush=True)

    def start(self):
        """准备串口采集并启动一次录制，已有录制时返回 False。"""
        if self.active:
            return False
        if getattr(self.sender.link, "serial", None) is None:
            raise ValueError("图像与角度采集请通过 USB 串口启动窗口")
        self.stop_event.clear()
        self.state = "preparing"
        self.thread = Thread(target=self.run)
        self.thread.start()
        return True

    def stop(self, wait=False):
        """请求结束采样；wait 为 True 时等待视频与数据保存。"""
        self.stop_event.set()
        if wait and self.thread is not None:
            self.thread.join(timeout=30)
            if self.thread.is_alive():
                raise TimeoutError("请等数据保存完成后结束窗口")

    def run(self):
        """运行采样，向界面报告异常，并恢复空闲状态。"""
        try:
            self.collect()
        except Exception as error:
            self.emit(f"采集信息：{error}", "error")
        finally:
            self.state = "idle"

    def collect(self):
        """按 5 Hz 配对相邻姿态，连同图像和相对时间操作记录保存。"""
        import cv2
        import numpy as np
        from lerobot.configs.video import RGBEncoderConfig
        from lerobot.datasets import LeRobotDataset
        from .dataset import FPS, SIZE, JOINTS, add_pair, features, read_info, save_pending

        self.emit("正在连接 Action 6 并检查 USB 角度反馈")
        with ExitStack() as resources:
            camera = open_camera(self.settings)
            resources.callback(camera.disconnect)
            self.sender.read_feedback()
            self.state = "countdown"
            for remaining in range(self.countdown, 0, -1):
                self.emit(f"录制倒计时：{remaining} 秒，请保持起始姿态")
                if self.stop_event.wait(1):
                    self.emit("已结束本次录制准备")
                    return
            if self.stop_event.is_set():
                return
            self.root.parent.mkdir(parents=True, exist_ok=True)
            encoder = RGBEncoderConfig(vcodec="h264", preset="fast", crf=20)
            if self.root.exists():
                info = read_info(self.root)
                if info["fps"] != FPS or info["gripper_type"] != "angular_direct":
                    raise ValueError("请核对现有数据的频率和夹爪定义")
                dataset = LeRobotDataset.resume(info["repo_id"], root=self.root,
                                               rgb_encoder=encoder, encoder_threads=2,
                                               image_writer_threads=2)
            else:
                dataset = LeRobotDataset.create(self.repo_id, root=self.root, fps=FPS,
                                                features=features(), robot_type="roarm_m3_follower",
                                                rgb_encoder=encoder, encoder_threads=2,
                                                image_writer_threads=2)
                info = {"repo_id": self.repo_id, "fps": FPS, "gripper_type": "angular_direct",
                        "joint_names": JOINTS, "angle_unit": "degree", "image_size": list(SIZE),
                        "action_source": "next_observed_joint_state", "action_offset_seconds": 1 / FPS,
                        "input_camera": self.settings["cameras"]["front"], "control_input": "keyboard"}
            resources.callback(dataset.finalize)
            resources.callback(save_pending, dataset)
            capture_dir = self.root / "capture"
            capture_dir.mkdir(exist_ok=True)
            (capture_dir / "info.json").write_text(json.dumps(info, ensure_ascii=False, indent=2) + "\n")
            episode = dataset.num_episodes
            samples = []
            capture = {"episode": episode, "task": self.task, "samples": samples,
                       "requested_seconds": self.seconds, "control_input": "keyboard",
                       "control_protocol": {"joint": "T121", "cartesian": "T123"},
                       "control_events": []}
            trace_path = capture_dir / f"episode_{episode:04d}.json"
            def save_trace():
                """将本段采样时刻与操作记录写入对应的 JSON 文件。"""
                if len(samples) >= 2:
                    trace_path.write_text(json.dumps(capture, ensure_ascii=False, indent=2) + "\n")
            resources.callback(save_trace)
            # 指令和角度共用起点，便于逐项对应按键方向、目标角度与反馈。
            start = time.perf_counter()
            self.sender.start_command_log()
            self.state = "recording"
            self.emit(f"录制开始：第 {episode} 段，{self.seconds:g} 秒，5 Hz")
            previous = None
            try:
                for index in range(round(self.seconds * FPS) + 1):
                    target = start + index / FPS
                    if self.stop_event.wait(max(0, target - time.perf_counter())):
                        break
                    before = time.perf_counter()
                    if before - target > 0.1:
                        raise RuntimeError("采样节奏检查：请缩短演示后重试")
                    feedback = self.sender.read_feedback()
                    after = time.perf_counter()
                    state = np.asarray(feedback["state"], dtype=np.float32)
                    if state.shape != (6,) or not np.isfinite(state).all():
                        raise ValueError("请检查六项关节角度反馈")
                    image = cv2.resize(camera.async_read(timeout_ms=1000), SIZE, interpolation=cv2.INTER_AREA)
                    samples.append({"sample": index, "time_s": (before + after) / 2 - start,
                                    "read_seconds": after - before, "state": state.tolist(),
                                    "feedback_time_s": feedback["received_at"] - start})
                    current = {"state": state, "image": image}
                    if previous is not None:
                        add_pair(dataset, previous, current, self.task)
                    previous = current
                    if index % FPS == 0:
                        self.emit(f"录制中：{index / FPS:.0f}/{self.seconds:g} 秒 · {index} 帧")
            finally:
                self.state = "saving"
                try:
                    self.stop_motion()
                finally:
                    # 保留事件内容，将通信对象的计时转换为本段相对秒数。
                    events = self.sender.finish_command_log()
                    capture["control_events"] = [
                        {**{key: value for key, value in event.items() if key != "time"},
                         "time_s": event["time"] - start}
                        for event in events
                    ]
                self.emit("采样已结束，六轴已请求停止，正在保存视频")
        if len(samples) >= 2:
            self.last_episode = episode
            self.emit(f"已保存第 {episode} 段：{len(samples) - 1} 帧 · {self.root}", "saved")
        else:
            self.emit("本段采样数量为零，请重新开始录制")
