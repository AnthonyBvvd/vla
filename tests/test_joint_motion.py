"""使用模拟通信检查 T121 持续更新、松手持位与角度采集。"""

from collections import deque
from contextlib import contextmanager
from threading import Condition, Thread, get_ident
import time
import unittest

from roarm_keyboard.control import Controller, JOINT_LIMITS, Sender


class MotionLink:
    """模拟串口反馈和单关节目标，记录通信线程及全部发送内容。"""

    serial = object()
    host = None
    dry_run = False

    def __init__(self, auto_reply=True):
        """接收自动回复设置；初始六轴角度采用数据集的 direct 定义。"""
        self.condition = Condition()
        self.state = [5.0, 20.0, 150.0, -3.0, 4.0, 35.0]
        self.feedback = None
        self.feedback_version = 0
        self.sent = []
        self.replies = deque()
        self.auto_reply = auto_reply
        self.released_replies = 0
        self.thread_calls = []
        self.closed = 0

    def send(self, command):
        """保存完整指令；反馈请求保存请求时刻的姿态，单轴指令更新该轴。"""
        with self.condition:
            self.thread_calls.append(("send", get_ident()))
            value = dict(command)
            self.sent.append(value)
            if value["T"] == 105:
                self.replies.append(list(self.state))
            elif value["T"] == 121:
                joint = value["joint"]
                self.state[joint - 1] = 180 - value["angle"] if joint == 6 else value["angle"]
            elif value["T"] == 100:
                self.state = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
            self.condition.notify_all()

    def drain(self):
        """按指定时机发布一项反馈，用于检查等待回复期间的按键变化。"""
        with self.condition:
            self.thread_calls.append(("drain", get_ident()))
            if self.replies and (self.auto_reply or self.released_replies):
                state = self.replies.popleft()
                if self.released_replies:
                    self.released_replies -= 1
                self.feedback = {"state": state, "received_at": time.perf_counter(), "raw": {}}
                self.feedback_version += 1
                self.condition.notify_all()

    def close(self):
        """记录连接关闭次数，便于核对通信线程结束。"""
        with self.condition:
            self.closed += 1
            self.condition.notify_all()

    def wait_for(self, predicate, timeout=2):
        """等待指令或反馈满足条件，并返回等待结果。"""
        with self.condition:
            return self.condition.wait_for(predicate, timeout=timeout)

    def set_angle(self, joint, angle):
        """设置模拟实际角度，用于区分最后目标与松手时的反馈。"""
        with self.condition:
            self.state[joint - 1] = angle

    def release_reply(self):
        """仅放行下一条待回复反馈。"""
        with self.condition:
            self.released_replies += 1

    def enable_replies(self):
        """恢复自动回复，让测试结束时可完成持位和连接关闭。"""
        with self.condition:
            self.auto_reply = True

    def joint_commands(self, joint=None):
        """返回单关节指令副本，可按 joint 筛选。"""
        with self.condition:
            return [dict(value) for value in self.sent
                    if value["T"] == 121 and (joint is None or value["joint"] == joint)]


@contextmanager
def motion_setup(auto_reply=True, speed=3):
    """创建控制器及通信线程，并在测试结束时完成停止和资源释放。"""
    link = MotionLink(auto_reply)
    sender = Sender(link)
    controller = Controller(sender.send, speed=speed, joint_motion=sender.move_joint)
    try:
        yield link, sender, controller
    finally:
        link.enable_replies()
        controller.stop()
        sender.close()


class JointMotionTest(unittest.TestCase):
    """检查按键方向、逐轴更新及等待回复期间的状态变化。"""

    def test_q_uses_complete_t121_and_release_reads_actual_angle(self):
        """Q 只更新底座；松手持位使用新读取的实际角度。"""
        with motion_setup() as (link, sender, controller):
            controller.press("q")
            self.assertTrue(link.wait_for(lambda: len(link.joint_commands()) >= 2))
            commands = link.joint_commands()
            self.assertEqual({value["joint"] for value in commands}, {1})
            for value in commands:
                self.assertEqual(set(value), {"T", "joint", "angle", "spd", "acc"})
                self.assertEqual((value["T"], value["spd"], value["acc"]), (121, 3, 10))
            self.assertGreater(commands[0]["angle"], 5)
            self.assertGreater(commands[1]["angle"], commands[0]["angle"])
            with sender.condition:
                link.set_angle(1, 7.25)
                controller.release("q")
            self.assertTrue(link.wait_for(lambda: bool(link.joint_commands()) and
                                         link.joint_commands()[-1]["angle"] == 7.25))
            count = len(link.joint_commands())
            self.assertFalse(link.wait_for(lambda: len(link.joint_commands()) > count, timeout=0.15))
            self.assertEqual(controller.active, {})
            self.assertEqual(link.state[1:], [20.0, 150.0, -3.0, 4.0, 35.0])
            self.assertIsNone(sender.error)

    def test_multiple_axes_and_pending_t121_keep_separate_targets(self):
        """多个关节同时更新；直接排队的 T121 也分别保留目标。"""
        with motion_setup() as (link, sender, controller):
            with sender.condition:
                sender.send({"T": 121, "joint": 3, "angle": 149, "spd": 3, "acc": 10})
                sender.send({"T": 121, "joint": 4, "angle": -2, "spd": 3, "acc": 10})
                self.assertIn((121, None, 3), sender.pending)
                self.assertIn((121, None, 4), sender.pending)
            self.assertTrue(link.wait_for(lambda: len(link.joint_commands()) == 2))
            controller.press("q")
            controller.press("w")
            self.assertTrue(link.wait_for(lambda: len(link.joint_commands(1)) >= 2 and
                                         len(link.joint_commands(2)) >= 2))
            self.assertGreater(link.joint_commands(1)[-1]["angle"], 5)
            self.assertGreater(link.joint_commands(2)[-1]["angle"], 20)
            self.assertEqual(controller.active, {1: 1, 2: 1})
            self.assertEqual(link.state[2:], [149, -2, 4.0, 35.0])
            self.assertIsNone(sender.error)

    def test_y_and_h_match_original_direction_and_direct_feedback(self):
        """Y 增加原厂夹爪角度，H 减少；内部 direct 角度方向相反。"""
        with motion_setup() as (link, sender, controller):
            controller.press("y")
            self.assertTrue(link.wait_for(lambda: len(link.joint_commands(6)) >= 2))
            close_angle = link.joint_commands(6)[-1]["angle"]
            self.assertGreater(close_angle, 145)
            self.assertLess(link.state[5], 35)
            controller.release("y")
            self.assertTrue(link.wait_for(lambda: 6 not in sender.joints))
            controller.press("h")
            self.assertTrue(link.wait_for(lambda: bool(link.joint_commands(6)) and
                                         link.joint_commands(6)[-1]["angle"] < close_angle))
            self.assertEqual({value["joint"] for value in link.joint_commands()}, {6})
            self.assertEqual(link.state[:5], [5.0, 20.0, 150.0, -3.0, 4.0])
            self.assertIsNone(sender.error)

    def test_opposite_keys_hold_then_remaining_key_resumes(self):
        """同轴正反键同时按住时持位，松开其中一键后沿余下方向运动。"""
        with motion_setup() as (link, sender, controller):
            controller.press("q")
            self.assertTrue(link.wait_for(lambda: bool(link.joint_commands(1))))
            controller.press("a")
            self.assertTrue(link.wait_for(lambda: 1 not in sender.joints))
            self.assertEqual(controller.active, {})
            count = len(link.joint_commands(1))
            held_angle = link.joint_commands(1)[-1]["angle"]
            self.assertFalse(link.wait_for(lambda: len(link.joint_commands(1)) > count, timeout=0.15))
            controller.release("a")
            self.assertTrue(link.wait_for(lambda: len(link.joint_commands(1)) > count))
            self.assertGreater(link.joint_commands(1)[-1]["angle"], held_angle)
            self.assertEqual(controller.active, {1: 1})
            self.assertIsNone(sender.error)

    def test_release_during_initial_feedback_uses_new_hold_reply(self):
        """初次回复等待期间松手，旧回复结束，新的持位读取使用当前角度。"""
        with motion_setup(auto_reply=False) as (link, sender, controller):
            controller.press("q")
            self.assertTrue(link.wait_for(lambda: bool(link.replies)))
            controller.release("q")
            link.set_angle(1, 8.5)
            link.release_reply()
            self.assertTrue(link.wait_for(lambda: sum(value["T"] == 105 for value in link.sent) >= 2))
            self.assertEqual(link.joint_commands(), [])
            link.release_reply()
            self.assertTrue(link.wait_for(lambda: bool(link.joint_commands())))
            self.assertEqual(link.joint_commands(), [
                {"T": 121, "joint": 1, "angle": 8.5, "spd": 3, "acc": 10},
            ])
            self.assertEqual(controller.active, {})
            self.assertIsNone(sender.error)

    def test_home_during_initial_feedback_discards_old_motion_reply(self):
        """初次回复等待期间回位，回复完成后保持回位目标。"""
        with motion_setup(auto_reply=False) as (link, sender, controller):
            controller.press("q")
            self.assertTrue(link.wait_for(lambda: bool(link.replies)))
            controller.home()
            self.assertTrue(link.wait_for(lambda: any(value["T"] == 100 for value in link.sent)))
            link.release_reply()
            self.assertTrue(link.wait_for(lambda: link.feedback_version == 1))
            self.assertFalse(link.wait_for(lambda: bool(link.joint_commands()), timeout=0.15))
            self.assertEqual(link.state, [0.0, 0.0, 90.0, 0.0, 0.0, 0.0])
            self.assertEqual(controller.active, {})
            self.assertIsNone(sender.error)

    def test_close_during_initial_feedback_reads_current_hold_angle(self):
        """关闭时完成原回复，再读取当前角度作为最终持位目标。"""
        with motion_setup(auto_reply=False) as (link, sender, controller):
            controller.press("q")
            self.assertTrue(link.wait_for(lambda: bool(link.replies)))
            link.set_angle(1, 8.5)
            closing = Thread(target=sender.close)
            closing.start()
            try:
                self.assertTrue(link.wait_for(lambda: sender.closing))
                # 给线程处理关闭请求的时间，再分别放行原反馈与新持位反馈。
                link.wait_for(lambda: sum(value["T"] == 105 for value in link.sent) >= 2, timeout=0.05)
                link.release_reply()
                self.assertTrue(link.wait_for(lambda: sum(value["T"] == 105 for value in link.sent) >= 2))
                link.release_reply()
                closing.join(timeout=3)
                self.assertFalse(closing.is_alive())
                self.assertEqual(link.joint_commands(), [
                    {"T": 121, "joint": 1, "angle": 8.5, "spd": 3, "acc": 10},
                ])
                self.assertIsNone(sender.error)
            finally:
                link.enable_replies()
                closing.join(timeout=3)

    def test_five_hz_feedback_shares_one_thread_with_joint_motion(self):
        """采样线程按 5 Hz 请求角度，读写继续由唯一通信线程执行。"""
        with motion_setup() as (link, sender, controller):
            results = []
            errors = []

            def collect():
                """按固定采样时间请求六轴角度，并保存结果或异常。"""
                start = time.perf_counter()
                try:
                    for index in range(5):
                        time.sleep(max(0, start + index * 0.2 - time.perf_counter()))
                        results.append(sender.read_feedback())
                except Exception as error:
                    errors.append(error)

            controller.press("q")
            sampling = Thread(target=collect)
            sampling.start()
            sampling.join(timeout=3)
            self.assertFalse(sampling.is_alive())
            self.assertEqual(errors, [])
            self.assertEqual(len(results), 5)
            self.assertGreater(results[-1]["state"][0], results[0]["state"][0])
            for result in results:
                self.assertEqual(result["state"][1:], [20.0, 150.0, -3.0, 4.0, 35.0])
            self.assertEqual({thread_id for _, thread_id in link.thread_calls}, {sender.thread.ident})
            self.assertEqual({name for name, _ in link.thread_calls}, {"send", "drain"})
            self.assertIsNone(sender.error)

    def test_each_joint_target_stays_within_its_angle_range(self):
        """六轴分别接近上下限时，连续更新保持在各自角度范围内。"""
        for bound in ("low", "high"):
            with self.subTest(bound=bound), motion_setup(speed=20) as (link, sender, controller):
                with sender.condition:
                    for axis, (low, high) in enumerate(JOINT_LIMITS, 1):
                        link.set_angle(axis, low + 0.1 if bound == "low" else high - 0.1)
                        direction = 2 if bound == "low" else 1
                        if axis == 6:
                            direction = 3 - direction
                        sender.move_joint(axis, direction, 20)
                self.assertTrue(link.wait_for(lambda: all(len(link.joint_commands(axis)) >= 2
                                                         for axis in range(1, 7))))
                for axis, (low, high) in enumerate(JOINT_LIMITS, 1):
                    for command in link.joint_commands(axis):
                        target = 180 - command["angle"] if axis == 6 else command["angle"]
                        self.assertGreaterEqual(target, low)
                        self.assertLessEqual(target, high)
                    self.assertAlmostEqual(link.state[axis - 1], low if bound == "low" else high)
                self.assertIsNone(sender.error)

    def test_feedback_timeout_ends_motion_and_waiting_sample(self):
        """反馈等待结束时报告异常，并同时结束运动请求和采样等待。"""
        with motion_setup(auto_reply=False) as (link, sender, controller):
            controller.press("q")
            self.assertTrue(link.wait_for(lambda: bool(link.replies)))
            with self.assertRaises(TimeoutError):
                sender.read_feedback(timeout=1)
            self.assertIsInstance(sender.error, TimeoutError)
            self.assertEqual(sender.joints, {})
            self.assertEqual(link.joint_commands(), [])
            controller.stop()
            self.assertTrue(link.wait_for(lambda: {value["axis"] for value in link.sent
                                                 if value["T"] == 123 and value["cmd"] == 0}
                                         == set(range(1, 7))))


if __name__ == "__main__":
    unittest.main()
