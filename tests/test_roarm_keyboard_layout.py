"""使用模拟通信检查窗口事件、录制快捷键和缩放布局。"""

from queue import Queue
from types import SimpleNamespace
import time
import tkinter as tk
from tkinter import font
import unittest
from unittest.mock import patch

from roarm_keyboard.app import KEY_RELEASE_DELAY_MS, Window


class Sender:
    error = None

    def __init__(self):
        self.errors = Queue()
        self.commands = []
        self.closed = False
        self.moves = []

    def send(self, command):
        self.commands.append(dict(command))

    def close(self):
        self.closed = True

    def move_joint(self, axis, direction, speed):
        """保存界面的运动请求，供窗口测试核对。"""
        self.moves.append((axis, direction, speed))


class Recorder:
    seconds = 20
    countdown = 5
    state = "idle"
    active = False

    def __init__(self):
        self.events = Queue()
        self.started = 0

    def start(self):
        self.started += 1
        self.active = True
        self.state = "countdown"

    def stop(self, wait=False):
        self.active = False
        self.state = "idle"


class LayoutTest(unittest.TestCase):
    def test_repeat_events_keep_sending_joint_targets(self):
        """窗口与真实通信线程结合模拟设备，长按一秒持续更新单轴目标。"""
        from roarm_keyboard.control import Sender as MotionSender
        from test_joint_motion import MotionLink

        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"图形显示会话：{error}")
        link = MotionLink()
        sender = MotionSender(link)
        window = Window(root, sender, speed=3)
        try:
            root.update()
            root.focus_force()
            root.update()
            sender.start_command_log()
            window.on_press(SimpleNamespace(keysym="q"))
            start = time.monotonic()
            next_repeat = start + 0.2
            while time.monotonic() - start < 1.1:
                now = time.monotonic()
                if now >= next_repeat:
                    window.on_release(SimpleNamespace(keysym="q"))
                    root.after(2, lambda: window.on_press(SimpleNamespace(keysym="q")))
                    next_repeat = now + 0.03
                root.update()
                time.sleep(0.001)
            commands = link.joint_commands()
            self.assertGreaterEqual(len(commands), 8)
            self.assertEqual({value["joint"] for value in commands}, {1})
            self.assertGreater(commands[-1]["angle"] - commands[0]["angle"], 2)
            self.assertEqual(window.controller.active, {1: 1})
            window.on_release(SimpleNamespace(keysym="q"))
            deadline = time.monotonic() + 0.12
            while time.monotonic() < deadline:
                root.update()
                time.sleep(0.001)
            events = sender.finish_command_log()
            motions = [event["direction"] for event in events if event["kind"] == "motion"]
            self.assertEqual(motions, [1, 0])
            self.assertEqual(window.controller.active, {})
            self.assertEqual(link.state[1:], [20, 150, -3, 4, 35])
            self.assertIsNone(sender.error)
        finally:
            window.close()

    def test_delayed_repeat_events_keep_joint_motion_until_real_release(self):
        """重复释放/按下间隔 2 毫秒时维持长按，真正松手后结束运动。"""
        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"图形显示会话：{error}")
        sender = Sender()
        window = Window(root, sender, speed=1)
        try:
            root.update()
            sender.moves.clear()
            window.on_press(SimpleNamespace(keysym="q"))
            for _ in range(6):
                window.on_release(SimpleNamespace(keysym="q"))
                # 先让事件循环进入空闲，再送入下一次按下，覆盖原实现的间隔。
                root.after(2, lambda: window.on_press(SimpleNamespace(keysym="q")))
                deadline = time.monotonic() + 0.012
                while time.monotonic() < deadline:
                    root.update()
                    time.sleep(0.001)
            self.assertEqual(sender.moves, [(1, 1, 1)])
            self.assertEqual(window.controller.active, {1: 1})
            window.on_release(SimpleNamespace(keysym="q"))
            deadline = time.monotonic() + (KEY_RELEASE_DELAY_MS + 15) / 1000
            while time.monotonic() < deadline:
                root.update()
                time.sleep(0.001)
            self.assertEqual(sender.moves, [(1, 1, 1), (1, 0, 1)])
            self.assertEqual(window.controller.active, {})
        finally:
            window.close()

    def test_record_shortcuts_and_motion_during_countdown(self):
        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"图形显示会话：{error}")
        sender, recorder = Sender(), Recorder()
        window = Window(root, sender, recorder=recorder)
        try:
            root.update()
            window.on_press(SimpleNamespace(keysym="F9"))
            window.on_press(SimpleNamespace(keysym="F9"))
            self.assertEqual(recorder.started, 1)
            sender.commands.clear()
            window.on_press(SimpleNamespace(keysym="q"))
            window.home()
            self.assertEqual(sender.commands, [])
            recorder.state = "recording"
            window.on_press(SimpleNamespace(keysym="q"))
            self.assertEqual(sender.moves[-1][:2], (1, 1))
            window.on_press(SimpleNamespace(keysym="F10"))
            self.assertFalse(recorder.active)
            self.assertTrue(all(command["cmd"] == 0 for command in sender.commands[-6:]))
        finally:
            window.close()

    def test_home_repeat_and_space_on_home_button(self):
        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"图形显示会话：{error}")
        sender = Sender()
        window = Window(root, sender, connection="模拟回位检查")
        try:
            root.update()
            root.focus_force()
            root.update()
            sender.commands.clear()
            root.event_generate("<KeyPress>", keysym="Home")
            root.update()
            for _ in range(3):
                root.event_generate("<KeyRelease>", keysym="Home")
                root.event_generate("<KeyPress>", keysym="Home")
                root.update()
            self.assertEqual(sum(command["T"] == 100 for command in sender.commands), 1)
            root.event_generate("<KeyRelease>", keysym="Home")
            root.update()
            time.sleep((KEY_RELEASE_DELAY_MS + 5) / 1000)
            root.update()
            root.event_generate("<KeyPress>", keysym="Home")
            root.update()
            self.assertEqual(sum(command["T"] == 100 for command in sender.commands), 2)
            root.event_generate("<KeyRelease>", keysym="Home")
            root.update()
            home_button = window.action_buttons[2]
            home_button.focus_set()
            root.update()
            sender.commands.clear()
            home_button.event_generate("<KeyPress>", keysym="space")
            root.update()
            home_button.event_generate("<KeyRelease>", keysym="space")
            root.update()
            self.assertEqual([command["T"] for command in sender.commands], [123] * 6)
            home_button.invoke()
            self.assertEqual(sender.commands[-1], {"T": 100})
        finally:
            window.close()

    def test_scaled_and_small_windows_keep_controls_accessible(self):
        for screen, scaling, font_size in (((800, 600), 1.0, 10),
                                            ((1280, 720), 2.0, 12),
                                            ((1366, 768), 3.0, 14)):
            with self.subTest(screen=screen, scaling=scaling, font_size=font_size):
                try:
                    root = tk.Tk()
                except tk.TclError as error:
                    self.skipTest(f"图形显示会话：{error}")
                root.tk.call("tk", "scaling", scaling)
                font.nametofont("TkDefaultFont", root=root).configure(size=font_size)
                sender = Sender()
                with patch.object(root, "winfo_screenwidth", return_value=screen[0]), \
                     patch.object(root, "winfo_screenheight", return_value=screen[1]):
                    window = Window(root, sender, connection="USB 串口：/dev/ttyUSB0 · 115200 波特率",
                                    recorder=Recorder())
                try:
                    root.update()
                    self.assertLessEqual(root.winfo_width(), round(screen[0] * 0.9))
                    self.assertLessEqual(root.winfo_height(), round(screen[1] * 0.85))
                    self.assertEqual(window.heading_font.cget("size"), font_size)
                    window.mode.set(1)
                    window.change_mode()
                    root.update()
                    self.check_bounds(window)
                    minimum = root.minsize()
                    root.geometry(f"{minimum[0]}x{minimum[1]}")
                    root.update()
                    self.check_bounds(window)
                    self.assertEqual(window.action_buttons[1].grid_info()["columnspan"], len(window.action_buttons))
                    window.set_status("显示检查：" + "保持文字可读并使用滚动条查看。" * 15)
                    root.update()
                    window.canvas.yview_moveto(1)
                    root.update()
                    label = window.wrapped_labels[-1]
                    self.assertLessEqual(label.winfo_rooty() + label.winfo_height(),
                                         window.canvas.winfo_rooty() + window.canvas.winfo_height() + 2)
                    window.on_press(SimpleNamespace(keysym="q"))
                    self.assertEqual(sender.commands[-1]["cmd"], 1)
                    window.release("q")
                    self.assertEqual(sender.commands[-1]["cmd"], 0)
                finally:
                    window.close()
                self.assertTrue(sender.closed)
                self.assertTrue(all(command["cmd"] == 0 for command in sender.commands[-6:]))

    def check_bounds(self, window):
        for label in window.table_labels + window.wrapped_labels:
            self.assertLessEqual(label.winfo_x() + label.winfo_width(), label.master.winfo_width() + 2)
            self.assertGreaterEqual(label.winfo_height(), label.winfo_reqheight())
        for widget in window.action_buttons + window.record_buttons + window.mode_buttons + [window.speed_box]:
            self.assertGreaterEqual(widget.winfo_width(), widget.winfo_reqwidth())


if __name__ == "__main__":
    unittest.main()
