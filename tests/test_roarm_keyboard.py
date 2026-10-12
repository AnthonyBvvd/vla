"""检查按键请求、发送顺序以及 PTY 和 HTTP 模拟通信。"""

import json
import math
import os
import pty
import select
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Condition, Event, Thread
from urllib.parse import parse_qs, urlsplit

from roarm_keyboard.control import Controller, Link, Sender


def command(axis, direction, mode=0, speed=3):
    return {"T": 123, "m": mode, "axis": axis, "cmd": direction, "spd": speed}


class MemoryLink:
    def __init__(self, hold_first=False):
        self.sent = []
        self.condition = Condition()
        self.entered = Event()
        self.resume = Event()
        self.hold_first = hold_first
        self.closed = 0

    def send(self, value):
        with self.condition:
            self.sent.append(dict(value))
            self.condition.notify_all()
        if self.hold_first and len(self.sent) == 1:
            self.entered.set()
            if not self.resume.wait(timeout=3):
                raise TimeoutError("模拟发送等待结束")

    def drain(self):
        pass

    def close(self):
        self.closed += 1

    def wait_sent(self, count):
        with self.condition:
            return self.condition.wait_for(lambda: len(self.sent) >= count, timeout=3)


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.motions = []
        self.controller = Controller(self.sent.append,
                                     joint_motion=lambda axis, direction, speed: self.motions.append((axis, direction, speed)))

    def test_repeat_and_release(self):
        for key in ("q", "q", "Q", "q"):
            self.controller.press(key)
        self.assertEqual(self.motions, [(1, 1, 3)])
        self.controller.release("Q")
        self.controller.release("q")
        self.assertEqual(self.motions, [(1, 1, 3), (1, 0, 3)])
        self.assertEqual(self.controller.active, {})

    def test_multiple_axes_and_opposite_keys(self):
        for key in ("q", "w", "y"):
            self.controller.press(key)
        self.controller.press("a")
        self.assertEqual(self.motions, [(1, 1, 3), (2, 1, 3), (6, 1, 3), (1, 0, 3)])
        self.assertEqual(self.controller.active, {2: 1, 6: 1})
        self.controller.release("a")
        self.controller.release("w")
        self.assertEqual(self.motions[-2:], [(1, 1, 3), (2, 0, 3)])
        self.assertEqual(self.controller.active, {1: 1, 6: 1})

    def test_stop_keeps_held_keys_muted(self):
        self.controller.press("q")
        self.controller.press("w")
        self.controller.stop()
        self.assertEqual(self.sent, [command(axis, 0) for axis in range(1, 7)])
        count = len(self.motions)
        for key in ("q", "w", "q", "W"):
            self.controller.press(key)
        self.assertEqual(len(self.motions), count)
        self.assertEqual(self.controller.active, {})
        self.controller.release("q")
        self.controller.press("q")
        self.assertEqual(self.motions[-1], (1, 1, 3))
        self.assertEqual(self.controller.active, {1: 1})

    def test_mode_change_stops_old_mode_first(self):
        self.controller.press("q")
        self.controller.set_mode(1)
        self.assertEqual(self.sent, [command(axis, 0, mode=0) for axis in range(1, 7)])
        self.controller.press("q")
        self.assertEqual(len(self.sent), 6)
        self.controller.release("q")
        self.controller.press("q")
        self.assertEqual(self.sent[-1], command(1, 1, mode=1))
        self.controller.set_mode(1)
        self.assertEqual(len(self.sent), 7)
        with self.assertRaises(ValueError):
            self.controller.set_mode(2)

    def test_home_stops_six_axes_and_mutes_held_keys(self):
        self.controller.press("q")
        self.controller.press("w")
        self.controller.home()
        self.assertEqual(self.sent, [command(axis, 0) for axis in range(1, 7)] + [{"T": 100}])
        self.assertEqual(self.controller.active, {})
        count = len(self.sent)
        self.controller.press("q")
        self.controller.press("w")
        self.assertEqual(len(self.sent), count)
        self.controller.release("q")
        self.controller.press("q")
        self.assertEqual(self.motions[-1], (1, 1, 3))

    def test_speed_range_and_change(self):
        for value in (1, 20, 3.5, "4"):
            self.assertEqual(Controller.validate_speed(value), float(value))
        for value in (0, 0.9, 20.1, float("nan"), float("inf"), -float("inf"), "x"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                Controller.validate_speed(value)
        self.controller.press("q")
        self.controller.set_speed(4)
        self.assertEqual(self.sent, [command(axis, 0, speed=3) for axis in range(1, 7)])
        self.controller.press("q")
        self.assertEqual(len(self.sent), 6)
        self.controller.release("q")
        self.controller.press("q")
        self.assertEqual(self.motions[-1], (1, 1, 4))


class SenderTest(unittest.TestCase):
    def test_home_follows_stops_and_can_be_removed_before_send(self):
        link = MemoryLink()
        sender = Sender(link)
        try:
            with sender.condition:
                controller = Controller(sender.send)
                controller.home()
            self.assertTrue(link.wait_sent(7))
            self.assertEqual(link.sent, [command(axis, 0) for axis in range(1, 7)] + [{"T": 100}])
            with sender.condition:
                sender.send({"T": 100})
                sender.send(command(1, 0))
                self.assertNotIn((100, None, None), sender.pending)
            self.assertTrue(link.wait_sent(8))
        finally:
            sender.close()
        self.assertEqual(sum(value["T"] == 100 for value in link.sent), 1)

    def test_latest_axis_command_and_stop_first(self):
        link = MemoryLink()
        sender = Sender(link)
        try:
            with sender.condition:
                sender.send(command(1, 1))
                latest = command(1, 2)
                sender.send(latest)
                latest["cmd"] = 1
                sender.send(command(2, 1))
                sender.send(command(3, 0))
            self.assertTrue(link.wait_sent(3))
        finally:
            sender.close()
        self.assertEqual(link.sent, [command(3, 0), command(1, 2), command(2, 1)])
        self.assertEqual(link.closed, 1)
        self.assertFalse(sender.thread.is_alive())
        self.assertIsNone(sender.error)

    def test_close_finishes_stops_and_releases_link(self):
        link = MemoryLink(hold_first=True)
        sender = Sender(link)
        closer = None
        try:
            sender.send(command(6, 1))
            self.assertTrue(link.entered.wait(timeout=3))
            sender.send(command(1, 1))
            sender.send(command(2, 0))
            sender.send(command(3, 0))
            closer = Thread(target=sender.close)
            closer.start()
            with sender.condition:
                self.assertTrue(sender.condition.wait_for(lambda: sender.closing, timeout=3))
            link.resume.set()
            closer.join(timeout=3)
            self.assertFalse(closer.is_alive())
        finally:
            link.resume.set()
            if closer is not None:
                closer.join(timeout=3)
            sender.close()
        self.assertEqual(link.sent, [command(6, 1), command(2, 0), command(3, 0)])
        self.assertEqual(link.closed, 1)
        self.assertEqual(sender.pending, {})
        self.assertFalse(sender.thread.is_alive())
        self.assertIsNone(sender.error)


class LinkTest(unittest.TestCase):
    def test_http_feedback_and_single_joint_motion(self):
        """通过本机 HTTP 服务读取角度并完成底座更新和松手持位。"""
        condition = Condition()
        received = []
        state = [12, 20, 100, 0, 0, 30]

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                """模拟原厂查询与单关节控制，返回 T1051 或普通响应。"""
                value = json.loads(parse_qs(urlsplit(self.path).query)["json"][0])
                with condition:
                    received.append(value)
                    if value["T"] == 121:
                        axis = value["joint"]
                        state[axis - 1] = 180 - value["angle"] if axis == 6 else value["angle"]
                    angles = [*state[:5], 180 - state[5]]
                    body = {"T": 1051, **dict(zip(("b", "s", "e", "t", "r", "g"),
                                                  map(math.radians, angles)))} if value["T"] == 105 else {}
                    condition.notify_all()
                self.send_response(200)
                self.end_headers()
                self.wfile.write(json.dumps(body).encode())

            def log_message(self, *args):
                pass

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = Thread(target=server.serve_forever, daemon=True)
        thread.start()
        sender = Sender(Link(host=f"127.0.0.1:{server.server_port}"))
        controller = Controller(sender.send, joint_motion=sender.move_joint)
        try:
            controller.press("q")
            with condition:
                self.assertTrue(condition.wait_for(lambda: sum(value["T"] == 121 for value in received) >= 2, timeout=2))
            controller.release("q")
            with condition:
                self.assertTrue(condition.wait_for(lambda: sum(value["T"] == 121 for value in received) >= 3, timeout=2))
            self.assertIsNone(sender.error)
            self.assertEqual(state[1:], [20, 100, 0, 0, 30])
            self.assertEqual({value["joint"] for value in received if value["T"] == 121}, {1})
        finally:
            controller.stop()
            sender.close()
            server.shutdown()
            server.server_close()
            thread.join(timeout=3)

    def test_feedback_query_keeps_control_active_and_reads_partial_reply(self):
        master, slave = pty.openpty()
        link = Link(port=os.ttyname(slave))
        sender = Sender(link)
        results = []
        errors = []

        def query():
            try:
                results.append(sender.read_feedback(timeout=2))
            except Exception as error:
                errors.append(error)

        def read_command():
            ready, _, _ = select.select([master], [], [], 2)
            self.assertTrue(ready)
            value = bytearray()
            while not value.endswith(b"\n"):
                value.extend(os.read(master, 1))
            return json.loads(value)

        query_thread = Thread(target=query)
        try:
            query_thread.start()
            self.assertEqual(read_command(), {"T": 105})
            sender.send(command(1, 0))
            self.assertEqual(read_command(), command(1, 0))
            angles = [12, 25, 140, -2, 3, 160]
            reply = {"T": 1051, **dict(zip(("b", "s", "e", "t", "r", "g"),
                                              map(math.radians, angles)))}
            data = json.dumps(reply).encode()
            os.write(master, b"status\n" + data[:15])
            time.sleep(0.03)
            os.write(master, data[15:] + b"\r\n")
            query_thread.join(timeout=2)
            self.assertFalse(query_thread.is_alive())
            self.assertEqual(errors, [])
            for actual, expected in zip(results[0]["state"], [12, 25, 140, -2, 3, 20]):
                self.assertAlmostEqual(actual, expected)
            self.assertGreater(results[0]["received_at"], 0)
            with self.assertRaises(TimeoutError):
                sender.read_feedback(timeout=0.03)
            sender.send(command(6, 0))
            self.assertEqual(read_command(), {"T": 105})
            self.assertEqual(read_command(), command(6, 0))
        finally:
            sender.close()
            query_thread.join(timeout=2)
            os.close(master)
            os.close(slave)

    def test_http_home_waits_for_completion_reply(self):
        received = []

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                value = json.loads(parse_qs(urlsplit(self.path).query)["json"][0])
                received.append(value)
                if value["T"] == 100:
                    time.sleep(0.6)
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b'{"ok":1}')

            def log_message(self, format, *args):
                pass

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = Thread(target=server.serve_forever, daemon=True)
        thread.start()
        link = Link(host=f"127.0.0.1:{server.server_port}")
        try:
            link.send({"T": 100})
            link.send(command(1, 0))
        finally:
            link.close()
            server.shutdown()
            server.server_close()
            thread.join(timeout=3)
        self.assertEqual(received, [{"T": 100}, command(1, 0)])

    def test_serial_json_lines_and_close(self):
        master, slave = pty.openpty()
        link = None
        try:
            link = Link(port=os.ttyname(slave))
            expected = [command(1, 1), command(1, 0)]
            for value in expected:
                link.send(value)
            data = b""
            deadline = time.monotonic() + 3
            while data.count(b"\n") < 2:
                remaining = deadline - time.monotonic()
                self.assertGreater(remaining, 0)
                ready, _, _ = select.select([master], [], [], remaining)
                self.assertTrue(ready)
                data += os.read(master, 4096)
            self.assertTrue(data.endswith(b"\n"))
            self.assertEqual([json.loads(line) for line in data.splitlines()], expected)
            link.close()
            self.assertFalse(link.serial.is_open)
        finally:
            if link is not None:
                link.close()
            os.close(master)
            os.close(slave)

    def test_http_js_json_parameter(self):
        received = []

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                received.append(self.path)
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b"{}")

            def log_message(self, *args):
                pass

        with ThreadingHTTPServer(("127.0.0.1", 0), Handler) as server:
            thread = Thread(target=server.serve_forever, daemon=True)
            thread.start()
            link = Link(host=f"127.0.0.1:{server.server_port}")
            try:
                expected = command(2, 0, mode=1, speed=4.5)
                link.send(expected)
                self.assertEqual(len(received), 1)
                parsed = urlsplit(received[0])
                self.assertEqual(parsed.path, "/js")
                query = parse_qs(parsed.query)
                self.assertEqual(list(query), ["json"])
                self.assertEqual(json.loads(query["json"][0]), expected)
            finally:
                link.close()
                server.shutdown()
                thread.join(timeout=3)


if __name__ == "__main__":
    unittest.main()
