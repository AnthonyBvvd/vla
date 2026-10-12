"""使用模拟相机与通信对象检查录制、续录和操作记录。"""

import argparse
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
from threading import Event
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from roarm_keyboard.dataset import check
from roarm_keyboard.control import Controller, Sender as CommandSender
from roarm_keyboard.recording import Recorder
from test_joint_motion import MotionLink


class Camera:
    """提供颜色随帧变化的图像，并记录资源关闭情况。"""
    def __init__(self):
        self.closed = False
        self.count = 0

    def async_read(self, timeout_ms):
        self.count += 1
        return np.full((720, 1280, 3), 40 + self.count * 10, dtype=np.uint8)

    def disconnect(self):
        self.closed = True


class Sender:
    """提供六轴角度，并模拟与录制共用计时的方向和指令事件。"""

    link = SimpleNamespace(serial=object())

    def __init__(self):
        self.count = 0
        self.command_log = None

    def start_command_log(self):
        """开始本段操作记录，每次续录使用新的事件列表。"""
        self.command_log = []

    def finish_command_log(self):
        """返回本段记录并结束本段事件收集。"""
        events = self.command_log
        self.command_log = None
        return events

    def read_feedback(self):
        """返回稳定的六轴反馈，并为录制期间的读取生成操作事件。"""
        self.count += 1
        received_at = time.perf_counter()
        if self.command_log is not None:
            self.command_log.extend([
                {"time": received_at, "kind": "motion", "mode": 0,
                 "joint": 1, "direction": 1, "speed": 3},
                {"time": received_at, "kind": "command",
                 "command": {"T": 121, "joint": 1, "angle": self.count * 0.5,
                             "spd": 3, "acc": 10}},
            ])
        return {"state": [self.count * 0.5, 20, 150, 0, 0, 10], "received_at": received_at}


class RecordingTest(unittest.TestCase):
    """验证两段录制及准备期间结束操作后的数据可读性。"""

    def test_record_append_and_cancel_keep_native_dataset_readable(self):
        """检查原生数据帧与操作时间，并确认续录分别保存事件。"""
        cache = Path(__file__).resolve().parents[1] / ".cache/record-dev"
        cache.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=cache) as temporary:
            root = Path(temporary) / "dataset"
            recorder = Recorder(Sender(), {"cameras": {"front": {}}}, root,
                                "键盘录制检查", seconds=0.6, countdown=0)
            stops = []
            recorder.stop_motion = lambda: stops.append(True)
            cameras = []

            def camera(settings):
                device = Camera()
                cameras.append(device)
                return device

            with patch("roarm_keyboard.recording.open_camera", side_effect=camera), redirect_stdout(io.StringIO()):
                for episode in range(2):
                    self.assertTrue(recorder.start())
                    recorder.thread.join(timeout=30)
                    self.assertFalse(recorder.active)
                    self.assertEqual(recorder.last_episode, episode)
                    capture = json.loads((root / f"capture/episode_{episode:04d}.json").read_text())
                    self.assertEqual(capture["control_protocol"], {"joint": "T121", "cartesian": "T123"})
                    control_events = capture["control_events"]
                    self.assertEqual(len(control_events), 8)
                    self.assertEqual({event["kind"] for event in control_events}, {"motion", "command"})
                    self.assertTrue(all("time" not in event for event in control_events))
                    self.assertTrue(all(0 <= event["time_s"] <= 0.7 for event in control_events))
                    for index, sample in enumerate(capture["samples"]):
                        motion, command = control_events[index * 2:index * 2 + 2]
                        self.assertEqual((motion["mode"], motion["joint"], motion["direction"], motion["speed"]),
                                         (0, 1, 1, 3))
                        self.assertEqual(command["command"]["T"], 121)
                        self.assertEqual(command["command"]["joint"], 1)
                        self.assertEqual(command["command"]["angle"], sample["state"][0])
                        self.assertAlmostEqual(command["time_s"], sample["feedback_time_s"], places=6)
                    self.assertIsNone(recorder.sender.command_log)
                check(argparse.Namespace(root=root, act_forward=False))
                recorder.countdown = 3
                recorder.start()
                recorder.stop(wait=True)
            events = list(recorder.events.queue)
            self.assertEqual([event for event in events if event["kind"] == "error"], [])
            self.assertEqual(len(stops), 2)
            self.assertTrue(all(device.closed for device in cameras))
            report = json.loads((root / "capture/dataset_check.json").read_text())
            self.assertEqual((report["episodes"], report["frames"], report["fps"]), (2, 6, 5))
            self.assertEqual(report["act_chunk_shape"], [10, 6])
            self.assertAlmostEqual(report["sample_interval_seconds"]["mean"], 0.2, delta=0.01)

    def test_real_sender_motion_and_recording_save_stop_commands(self):
        """用真实通信线程录入底座运动，检查原生数据和最终停止记录。"""
        cache = Path(__file__).resolve().parents[1] / ".cache/record-dev"
        cache.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=cache) as temporary:
            root = Path(temporary) / "dataset"
            link = MotionLink()
            sender = CommandSender(link)
            controller = Controller(sender.send, joint_motion=sender.move_joint)
            recorder = Recorder(sender, {"cameras": {"front": {}}}, root,
                                "单独移动底座", seconds=0.8, countdown=0)
            recorder.stop_motion = controller.stop
            camera = Camera()
            started = Event()
            original_emit = recorder.emit

            def emit(text, kind="status"):
                """转发录制进度，在录制开始时通知测试按下方向键。"""
                original_emit(text, kind)
                if text.startswith("录制开始"):
                    started.set()

            try:
                with patch("roarm_keyboard.recording.open_camera", return_value=camera), \
                     patch.object(recorder, "emit", side_effect=emit), redirect_stdout(io.StringIO()):
                    self.assertTrue(recorder.start())
                    self.assertTrue(started.wait(timeout=30))
                    controller.press("q")
                    recorder.thread.join(timeout=30)
                    self.assertFalse(recorder.active)
                    self.assertEqual(recorder.last_episode, 0)
                    check(argparse.Namespace(root=root, act_forward=False))
                capture = json.loads((root / "capture/episode_0000.json").read_text())
                events = capture["control_events"]
                commands = [event["command"] for event in events if event["kind"] == "command"]
                targets = [command for command in commands if command["T"] == 121]
                self.assertGreaterEqual(len(targets), 4)
                self.assertEqual({command["joint"] for command in targets}, {1})
                # 停止方向请求之后，日志仍包含实际发送的六轴停止与底座持位。
                stopped = next(index for index, event in enumerate(events)
                               if event["kind"] == "motion" and event["direction"] == 0)
                final_commands = [event["command"] for event in events[stopped + 1:]
                                  if event["kind"] == "command"]
                stopped_axes = {command["axis"] for command in final_commands
                                if command["T"] == 123 and command["cmd"] == 0}
                self.assertEqual(stopped_axes, set(range(1, 7)))
                self.assertTrue(any(command["T"] == 121 and command["joint"] == 1
                                    for command in final_commands))
                self.assertAlmostEqual(targets[-1]["angle"], link.state[0])
                samples = np.asarray([sample["state"] for sample in capture["samples"]])
                self.assertGreater(np.ptp(samples[:, 0]), 0.6)
                np.testing.assert_array_equal(samples[:, 1:],
                                              np.tile([20, 150, -3, 4, 35], (len(samples), 1)))
                report = json.loads((root / "capture/dataset_check.json").read_text())
                self.assertEqual((report["episodes"], report["frames"], report["fps"]), (1, 4, 5))
                self.assertGreater(report["joint_range_degrees"][0], 0.3)
                self.assertEqual(report["joint_range_degrees"][1:], [0, 0, 0, 0, 0])
                self.assertTrue(camera.closed)
                self.assertIsNone(sender.command_log)
                self.assertIsNone(sender.error)
                self.assertEqual([event for event in recorder.events.queue if event["kind"] == "error"], [])
            finally:
                recorder.stop(wait=True)
                controller.stop()
                sender.close()


if __name__ == "__main__":
    unittest.main()
