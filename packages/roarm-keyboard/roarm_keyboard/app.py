"""RoArm-M3 键盘控制与采集窗口。

窗口负责按键、焦点和录制操作，通信线程负责反馈与定时目标更新。
关节模式显示角速度，坐标模式显示原厂速度系数。
释放事件短等 30 毫秒确认，处理 X11 长按产生的重复释放/按下事件。
"""

import argparse
import json
import math
import os
import signal
from pathlib import Path
from queue import Empty
import tkinter as tk
from tkinter import font, ttk

from .control import Controller, KEYS, Link, PAIRS, Sender


LABELS = {
    0: [("底座", "BL / 左", "BR / 右"), ("肩关节", "SD / 下", "SU / 上"),
        ("肘关节", "ED / 下", "EU / 上"), ("腕倾", "W+DG", "W-UG"),
        ("腕转", "R+DG", "R-UG"), ("夹爪", "G+DG / 夹紧", "G-UG / 张开")],
    1: [("X", "X+", "X-"), ("Y", "Y+", "Y-"), ("Z", "Z+", "Z-"),
        ("Pitch", "Pitch+", "Pitch-"), ("Roll", "Roll+", "Roll-"), ("夹爪", "夹紧", "张开")],
}

KEY_RELEASE_DELAY_MS = 30


class Window:
    """将界面事件传给控制器，并显示通信和录制进度。"""

    def __init__(self, root, sender, mode=0, speed=3, connection="USB 串口", recorder=None):
        """接收 Tk 窗口、通信对象和可选录制器，建立按键与反馈通道。"""
        self.root = root
        self.sender = sender
        self.controller = Controller(sender.send, mode, speed, joint_motion=sender.move_joint)
        self.recorder = recorder
        if recorder is not None:
            recorder.stop_motion = self.controller.stop
        self.record_held = set()
        self.release_jobs = {}
        self.home_held = False
        self.closing = False
        title = "RoArm-M3 键盘控制与采集" if recorder is not None else "RoArm-M3 键盘控制"
        root.title(title)
        root.withdraw()
        root.resizable(True, True)
        self.default_font = font.nametofont("TkDefaultFont", root=root)
        font_options = self.default_font.configure()
        self.title_font = font.Font(root=root, **{**font_options,
                                   "size": round(font_options["size"] * 1.25), "weight": "bold"})
        self.heading_font = font.Font(root=root, **{**font_options, "weight": "bold"})
        self.line_height = self.default_font.metrics("linespace")
        self.padding = max(6, round(self.line_height * 0.3))
        self.gap = max(4, round(self.line_height * 0.15))
        self.mode = tk.IntVar(value=mode)
        self.speed = tk.StringVar(value=str(speed))
        self.status = tk.StringVar(value="点击此窗口后使用 Q/A 等按键；长按运动，松开停止")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        viewport = ttk.Frame(root)
        viewport.grid(row=0, column=0, sticky="nsew")
        viewport.columnconfigure(0, weight=1)
        viewport.rowconfigure(0, weight=1)
        background = ttk.Style(root).lookup("TFrame", "background") or root.cget("background")
        self.canvas = tk.Canvas(viewport, highlightthickness=0, background=background)
        self.scrollbar = ttk.Scrollbar(viewport, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.scrollbar.grid(row=0, column=1, sticky="ns")
        self.panel = ttk.Frame(self.canvas, padding=self.padding)
        self.panel.columnconfigure(0, weight=1)
        self.panel_id = self.canvas.create_window((0, 0), window=self.panel, anchor="nw")
        self.wrapped_labels = []
        for row, text, options in (
            (0, title, {"font": self.title_font}),
            (1, connection, {}),
        ):
            label = ttk.Label(self.panel, text=text, **options)
            label.grid(row=row, column=0, sticky="ew", pady=(0, self.gap))
            self.wrapped_labels.append(label)
        choices = ttk.Frame(self.panel)
        choices.grid(row=2, column=0, sticky="ew", pady=self.gap)
        choices.columnconfigure(0, weight=1)
        self.mode_row = ttk.Frame(choices)
        self.mode_row.grid(row=0, column=0, sticky="ew")
        self.mode_buttons = []
        for label, value in (("关节角度", 0), ("末端坐标", 1)):
            button = ttk.Radiobutton(self.mode_row, text=label, value=value, variable=self.mode,
                                     command=self.change_mode)
            button.grid(row=0, column=value, sticky="w", padx=(0, self.gap), pady=self.gap)
            self.mode_buttons.append(button)
        self.speed_row = ttk.Frame(choices)
        self.speed_row.grid(row=1, column=0, sticky="ew", pady=self.gap)
        self.speed_label = ttk.Label(self.speed_row, text="速度系数")
        self.speed_label.grid(row=0, column=0, sticky="w", padx=(0, self.gap))
        speed_box = ttk.Spinbox(self.speed_row, from_=1, to=20, width=5, textvariable=self.speed,
                                command=self.change_speed, state="readonly")
        speed_box.grid(row=0, column=1, sticky="w")
        self.speed_box = speed_box
        speed_box.bind("<Return>", self.change_speed)
        speed_box.bind("<FocusOut>", self.change_speed)
        self.rows = []
        table = ttk.Frame(self.panel)
        table.grid(row=3, column=0, sticky="ew", pady=self.gap)
        self.table_labels = []
        for column in range(3):
            table.columnconfigure(column, weight=1, uniform="controls")
        for column, text in enumerate(("控制轴", "增加 / 正向", "减少 / 反向")):
            label = ttk.Label(table, text=text, font=self.heading_font)
            label.grid(row=0, column=column, sticky="nw", padx=self.gap, pady=self.gap)
            self.table_labels.append(label)
        for index in range(6):
            row = []
            for column in range(3):
                label = ttk.Label(table)
                label.grid(row=index + 1, column=column, padx=self.gap, pady=self.gap, sticky="nw")
                row.append(label)
                self.table_labels.append(label)
            self.rows.append(row)
        self.buttons = ttk.Frame(self.panel)
        self.buttons.grid(row=4, column=0, sticky="ew", pady=self.gap)
        self.action_buttons = [ttk.Button(self.buttons, text="停止全部", command=self.stop),
                               ttk.Button(self.buttons, text="开启保持力", command=self.torque_on),
                               ttk.Button(self.buttons, text="回到初始位置", command=self.home)]
        for column, button in enumerate(self.action_buttons):
            self.buttons.columnconfigure(column, weight=1)
            button.grid(row=0, column=column, sticky="ew", padx=self.gap, pady=self.gap)
            button.bind("<KeyPress-space>", self.on_press)
            button.bind("<KeyRelease-space>", lambda event: "break")
        next_row = 5
        self.record_buttons = []
        if recorder is not None:
            record_panel = ttk.LabelFrame(self.panel, text="Action 6 · USB 角度 · 5 Hz", padding=self.gap)
            record_panel.grid(row=next_row, column=0, sticky="ew", pady=self.gap)
            record_panel.columnconfigure(0, weight=1)
            record_actions = ttk.Frame(record_panel)
            record_actions.grid(row=0, column=0, sticky="ew")
            self.record_buttons = [ttk.Button(record_actions, text="F9 开始录制", command=self.start_recording),
                                   ttk.Button(record_actions, text="F10 结束保存", command=self.finish_recording)]
            for column, button in enumerate(self.record_buttons):
                record_actions.columnconfigure(column, weight=1)
                button.grid(row=0, column=column, sticky="ew", padx=self.gap, pady=self.gap)
                button.bind("<KeyPress-space>", self.on_press)
                button.bind("<KeyRelease-space>", lambda event: "break")
            self.record_status = tk.StringVar(value=f"每段 {recorder.seconds:g} 秒；按 F9 后倒计时 {recorder.countdown} 秒")
            record_label = ttk.Label(record_panel, textvariable=self.record_status)
            record_label.grid(row=1, column=0, sticky="ew", pady=self.gap)
            self.record_label = record_label
            self.wrapped_labels.append(record_label)
            next_row += 1
        help_label = ttk.Label(self.panel, text="Home 回到初始位置 · Space 停止\nTab 切换 · Esc 结束\n"
                                                "同轴相反键同时按住时停止")
        help_label.grid(row=next_row, column=0, sticky="ew", pady=self.gap)
        status_label = ttk.Label(self.panel, textvariable=self.status)
        status_label.grid(row=next_row + 1, column=0, sticky="ew", pady=self.gap)
        self.wrapped_labels.extend((help_label, status_label))
        self.canvas.bind("<Configure>", self.resize_layout)
        self.panel.bind("<Configure>", lambda event: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        root.bind("<MouseWheel>", self.scroll)
        root.bind("<Button-4>", lambda event: self.canvas.yview_scroll(-3, "units"))
        root.bind("<Button-5>", lambda event: self.canvas.yview_scroll(3, "units"))
        root.bind("<KeyPress>", self.on_press)
        root.bind("<KeyRelease>", self.on_release)
        root.bind("<FocusOut>", lambda event: root.after_idle(self.check_focus))
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.update_labels()
        longest_cell = max(self.default_font.measure(label.cget("text")) for label in self.table_labels)
        max_width = round(root.winfo_screenwidth() * 0.9)
        max_height = round(root.winfo_screenheight() * 0.85)
        width = min(max(longest_cell * 3 + self.padding * 2 + self.gap * 6 + 24, 480), max_width)
        height = min(self.line_height * 18, max_height)
        root.minsize(min(max(320, self.line_height * 7), max_width),
                     min(max(240, self.line_height * 5), max_height))
        root.geometry(f"{width}x{height}")
        root.deiconify()
        root.update_idletasks()
        height = min(self.panel.winfo_reqheight(), max_height)
        root.geometry(f"{width}x{height}")
        self.controller.stop()
        self.poll_job = root.after(50, self.poll_errors)

    def resize_layout(self, event):
        self.canvas.itemconfigure(self.panel_id, width=event.width)
        available = max(1, event.width - self.padding * 2)
        for label in self.wrapped_labels:
            inset = self.gap * 2 + 8 if label is getattr(self, "record_label", None) else 0
            label.configure(wraplength=max(1, available - inset))
        cell_width = max(1, available // 3 - self.gap * 2)
        for label in self.table_labels:
            label.configure(wraplength=cell_width)
        modes_stacked = available < sum(button.winfo_reqwidth() for button in self.mode_buttons) + self.gap * 2
        for index, button in enumerate(self.mode_buttons):
            button.grid_configure(row=index if modes_stacked else 0, column=0 if modes_stacked else index)
        self.speed_label.configure(wraplength=available)
        speed_stacked = available < self.speed_label.winfo_reqwidth() + self.speed_box.winfo_reqwidth() + self.gap
        self.speed_box.grid_configure(row=1 if speed_stacked else 0, column=0 if speed_stacked else 1)
        buttons_stacked = available < sum(button.winfo_reqwidth() for button in self.action_buttons) + self.gap * 2 * len(self.action_buttons)
        for index, button in enumerate(self.action_buttons):
            button.grid_configure(row=index if buttons_stacked else 0, column=0 if buttons_stacked else index,
                                  columnspan=len(self.action_buttons) if buttons_stacked else 1)
        record_stacked = available < sum(button.winfo_reqwidth() for button in self.record_buttons) + self.gap * 6
        for index, button in enumerate(self.record_buttons):
            button.grid_configure(row=index if record_stacked else 0, column=0 if record_stacked else index,
                                  columnspan=2 if record_stacked else 1)

    def scroll(self, event):
        direction = -1 if event.delta > 0 else 1
        self.canvas.yview_scroll(direction * 3, "units")
        return "break"

    def set_status(self, text):
        if self.sender.error is not None:
            text = f"连接信息：{self.sender.error}。请检查连接，使用机械臂电源开关结束运动。"
        self.status.set(text)

    def motion_status(self):
        keys = " / ".join(key.upper() for key in sorted(self.controller.held - self.controller.muted))
        self.set_status(f"运动中：{keys}" if self.controller.active else "已请求当前方向停止")

    def update_labels(self):
        """同步模式对应的关节名称、方向说明和速度单位。"""
        self.speed_label.configure(text="角速度 (°/s)" if self.controller.mode == 0 else "速度系数")
        for row, pair, labels in zip(self.rows, PAIRS, LABELS[self.controller.mode]):
            row[0].configure(text=labels[0])
            row[1].configure(text=f"{pair[0].upper()}   {labels[1]}")
            row[2].configure(text=f"{pair[1].upper()}   {labels[2]}")

    def change_mode(self):
        """先结束旧模式运动，再更新表格；继续运动时需重新按键。"""
        self.controller.set_mode(self.mode.get())
        self.update_labels()
        self.set_status("模式已切换；松开方向键，再按下继续运动")

    def change_speed(self, event=None):
        """验证输入速度并结束当前运动，使新速度从下一次按键开始使用。"""
        try:
            self.controller.set_speed(self.speed.get())
            speed_text = f"角速度 {self.controller.speed:g}°/s" if self.controller.mode == 0 else \
                         f"速度系数 {self.controller.speed:g}"
            self.set_status(f"{speed_text}；松开方向键，再按下继续运动")
        except ValueError as error:
            self.stop()
            self.set_status(str(error))
        return "break"

    def on_press(self, event):
        """处理按下事件；再次按下同键时取消待确认的释放任务。"""
        if self.closing:
            return "break"
        key = event.keysym.lower()
        if key == "escape":
            self.close()
            return "break"
        if key == "space":
            self.stop()
            return "break"
        if self.sender.error is not None:
            return "break"
        if key in self.release_jobs:
            self.root.after_cancel(self.release_jobs.pop(key))
        if key == "tab":
            self.mode.set(1 - self.controller.mode)
            self.change_mode()
        elif key == "home":
            if not self.home_held:
                self.home_held = True
                self.home()
        elif key in ("f9", "f10") and self.recorder is not None:
            if key not in self.record_held:
                self.record_held.add(key)
                self.start_recording() if key == "f9" else self.finish_recording()
        elif key in KEYS:
            try:
                with self.controller.lock:
                    if self.recorder is not None and self.recorder.state in ("preparing", "countdown", "saving"):
                        self.set_status("请等录制准备或保存完成，再按运动键")
                        return "break"
                    self.controller.set_speed(self.speed.get())
                    self.controller.press(key)
                self.motion_status()
            except ValueError as error:
                self.stop()
                self.set_status(str(error))
        else:
            return
        return "break"

    def on_release(self, event):
        """短等后确认松手；返回 break 结束该键事件的后续处理。"""
        key = event.keysym.lower()
        tracked = key in KEYS or key in ("home", "f9", "f10")
        if tracked and not self.closing:
            if key in self.release_jobs:
                self.root.after_cancel(self.release_jobs.pop(key))
            # X11 自动重复中的两个事件可能相隔几毫秒；on_press 会取消这次任务。
            self.release_jobs[key] = self.root.after(KEY_RELEASE_DELAY_MS, lambda: self.release(key))
        return "break" if tracked else None

    def release(self, key):
        self.release_jobs.pop(key, None)
        if key == "home":
            self.home_held = False
            return
        if key in ("f9", "f10"):
            self.record_held.discard(key)
            return
        self.controller.release(key)
        self.motion_status()

    def stop(self):
        self.controller.stop()
        self.set_status("已请求六轴停止；松开方向键后可继续操作")

    def check_focus(self):
        if not self.closing and self.root.focus_get() is None:
            self.stop()
            self.set_status("窗口失去焦点，已请求六轴停止")

    def torque_on(self):
        self.stop()
        self.sender.send({"T": 210, "cmd": 1})
        self.set_status("已请求保持力开启")

    def home(self):
        if self.recorder is not None and self.recorder.active:
            self.set_status("请先按 F10 保存本段，再回到初始位置")
            return
        self.controller.home()
        self.set_status("已请求回到初始位置；请等姿态到位后继续操作")

    def start_recording(self):
        if self.recorder is None or self.recorder.active:
            return
        self.stop()
        try:
            self.recorder.start()
            self.record_status.set("正在准备相机与关节反馈")
        except ValueError as error:
            self.record_status.set(str(error))

    def finish_recording(self):
        self.stop()
        if self.recorder is not None and self.recorder.active:
            self.recorder.stop()
            self.record_status.set("正在结束采样并保存")

    def poll_errors(self):
        try:
            error = self.sender.errors.get_nowait()
        except Empty:
            pass
        else:
            self.stop()
            self.set_status(str(error))
        if self.recorder is not None:
            while True:
                try:
                    event = self.recorder.events.get_nowait()
                except Empty:
                    break
                self.record_status.set(event["text"])
                if event["kind"] == "error":
                    self.stop()
        if not self.closing:
            self.poll_job = self.root.after(50, self.poll_errors)

    def close(self):
        if self.closing:
            return
        self.closing = True
        for job in self.release_jobs.values():
            self.root.after_cancel(job)
        self.release_jobs.clear()
        self.root.after_cancel(self.poll_job)
        try:
            self.controller.stop()
            if self.recorder is not None:
                self.recorder.stop(wait=True)
            self.sender.close()
        finally:
            self.root.destroy()


def main():
    """读取配置与启动参数，建立通信线程并运行窗口事件循环。"""
    project = Path(os.environ.get("VLA_ROOT", Path.cwd()))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=project / "configs/roarm_m3.json")
    parser.add_argument("--port", help="USB 串口设备路径")
    parser.add_argument("--baudrate", type=int, default=None)
    parser.add_argument("--host", help="机械臂局域网地址，如 192.168.1.134")
    parser.add_argument("--speed", type=float, default=3, help="关节模式为度/秒；坐标模式为速度系数")
    parser.add_argument("--mode", choices=("joint", "cartesian"), default="joint")
    parser.add_argument("--dry-run", action="store_true", help="在窗口中验证按键并打印指令")
    parser.add_argument("--record-root", type=Path, default=project / "datasets/roarm_keyboard")
    parser.add_argument("--record-seconds", type=float, default=20)
    parser.add_argument("--record-countdown", type=int, default=5)
    parser.add_argument("--task", default="用键盘缓慢移动底座并开合夹爪")
    args = parser.parse_args()
    try:
        Controller.validate_speed(args.speed)
        if not math.isfinite(args.record_seconds) or args.record_seconds < 0.4 or args.record_countdown < 0:
            parser.error("录制时长请至少设为 0.4 秒，倒计时请设为零或正整数")
        settings = json.loads(args.config.read_text()) if args.config.exists() else {}
        port = args.port or settings.get("port")
        baudrate = args.baudrate or settings.get("baudrate", 115200)
        if not args.host and not port and not args.dry_run:
            parser.error("请使用 --port 指定串口")
        root = tk.Tk()
        link = Link(port=port, baudrate=baudrate, host=args.host, dry_run=args.dry_run)
        sender = Sender(link)
        connection = "模拟通信：打印按键指令" if args.dry_run else (f"局域网：{args.host}" if args.host else
                      f"USB 串口：{Path(port).resolve()} · {baudrate} 波特率")
        recorder = None
        if not args.host and not args.dry_run and settings.get("cameras", {}).get("front"):
            from .recording import Recorder
            recorder = Recorder(sender, settings, args.record_root, args.task,
                                args.record_seconds, args.record_countdown)
        window = Window(root, sender, 0 if args.mode == "joint" else 1, args.speed, connection, recorder)
        signal.signal(signal.SIGTERM, lambda signum, frame: root.after(0, window.close))
        try:
            root.mainloop()
        finally:
            window.close()
    except (ValueError, OSError, tk.TclError) as error:
        parser.exit(1, f"连接或界面信息：{error}\n")
