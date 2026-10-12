"""微雪网页遥控的 USB 轨迹采集、LeRobot 数据检查与回放。"""

import argparse
from contextlib import ExitStack
import json
import math
from pathlib import Path
import time

import cv2
import draccus
import numpy as np
from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.configs.video import RGBEncoderConfig
from lerobot.datasets import LeRobotDataset
from roarm_sdk.roarm import roarm
from roarm_keyboard.dataset import FPS, IMAGE, JOINTS, SIZE, add_pair, check, features, read_info, read_state, save_pending


ROOT = Path(__file__).resolve().parents[1]
LOW = np.array([-180, -90, 0, -90, -90, -10], dtype=np.float32)
HIGH = np.array([180, 90, 180, 90, 90, 100], dtype=np.float32)


def open_arm(settings):
    arm = roarm(roarm_type="roarm_m3", port=settings["port"],
                baudrate=settings["baudrate"], gripper_type="angular_direct")
    try:
        arm._serial_port.exclusive = True
    except OSError:
        arm.disconnect()
        raise ValueError("请结束已有 USB 控制窗口，再启动采集或回放") from None
    return arm


def record(args):
    settings = json.loads(Path(args.config).read_text(encoding="utf-8"))
    root = args.root.resolve()
    root.parent.mkdir(parents=True, exist_ok=True)
    encoder = RGBEncoderConfig(vcodec="h264", preset="fast", crf=20)
    if args.append:
        info = read_info(root)
        if info["gripper_type"] != "angular_direct" or info["fps"] != FPS:
            raise ValueError("请核对现有数据的关节定义和采样频率")
        dataset = LeRobotDataset.resume(info["repo_id"], root=root,
                                       rgb_encoder=encoder, encoder_threads=2,
                                       image_writer_threads=2)
    else:
        if root.exists():
            raise FileExistsError("请选用新的数据目录，添加演示可使用 --append")
        dataset = LeRobotDataset.create(args.repo_id, fps=FPS, features=features(), root=root,
                                        robot_type="roarm_m3_follower", use_videos=True,
                                        rgb_encoder=encoder, encoder_threads=2,
                                        image_writer_threads=2)
        info = {"repo_id": args.repo_id, "fps": FPS, "gripper_type": "angular_direct",
                "joint_names": JOINTS, "angle_unit": "degree", "image_size": list(SIZE),
                "action_source": "next_observed_joint_state", "action_offset_seconds": 1 / FPS,
                "input_camera": settings["cameras"]["front"]}
    capture_dir = root / "capture"
    capture_dir.mkdir(exist_ok=True)
    (capture_dir / "info.json").write_text(json.dumps(info, ensure_ascii=False, indent=2) + "\n")
    episode = dataset.num_episodes
    samples = []
    capture = {"episode": episode, "task": args.task, "samples": samples,
               "requested_seconds": args.seconds, "interrupted": False}
    trace_path = capture_dir / f"episode_{episode:04d}.json"
    with ExitStack() as resources:
        resources.callback(dataset.finalize)
        resources.callback(save_pending, dataset)
        resources.callback(lambda: trace_path.write_text(json.dumps(capture, ensure_ascii=False, indent=2) + "\n"))
        arm = open_arm(settings)
        resources.callback(arm.disconnect)
        camera_config = draccus.decode(CameraConfig, settings["cameras"]["front"])
        camera = make_cameras_from_configs({"front": camera_config})["front"]
        camera.connect()
        resources.callback(camera.disconnect)
        for remaining in range(args.countdown, 0, -1):
            print(f"录制倒计时：{remaining} 秒", flush=True)
            time.sleep(1)
        print("录制开始：请用微雪网页缓慢操作", flush=True)
        start = time.perf_counter()
        previous = None
        try:
            for index in range(round(args.seconds * FPS) + 1):
                target = start + index / FPS
                time.sleep(max(0, target - time.perf_counter()))
                before = time.perf_counter()
                if before - target > 0.1:
                    raise RuntimeError("采样节奏检查：请缩短演示后重试")
                state = read_state(arm)
                after = time.perf_counter()
                frame = camera.async_read(timeout_ms=1000)
                image = cv2.resize(frame, SIZE, interpolation=cv2.INTER_AREA)
                current = {"state": state, "image": image}
                samples.append({"sample": index, "time_s": (before + after) / 2 - start,
                                "read_seconds": after - before, "state": state.tolist()})
                if previous is not None:
                    add_pair(dataset, previous, current, args.task)
                previous = current
                if index % FPS == 0:
                    print(f"已采集 {index / FPS:.0f} 秒，{index} 组图像/状态/动作", flush=True)
        except KeyboardInterrupt:
            capture["interrupted"] = True
            print("结束当前演示并保存已采集的完整配对", flush=True)
        print("采样完成：请松开网页运动按钮，正在保存视频", flush=True)
    if len(samples) < 2:
        raise ValueError("请录制至少两次采样")
    print(json.dumps({"root": str(root), "episode": episode,
                      "frames": len(samples) - 1, "fps": FPS, "trace": str(trace_path)},
                     ensure_ascii=False, indent=2), flush=True)


def send_target(arm, target, speed):
    wire = np.asarray(target, dtype=float).copy()
    wire[5] = 180 - wire[5]
    command = dict(zip(("b", "s", "e", "t", "r", "h"), wire.tolist()))
    command.update({"T": 122, "spd": speed, "acc": 10})
    grip = {"T": 121, "joint": 6, "angle": float(wire[5]), "spd": speed, "acc": 10}
    arm._serial_port.write((json.dumps(command) + "\n" + json.dumps(grip) + "\n").encode())
    arm._serial_port.flush()


def replay(args):
    root = args.root.resolve()
    info = read_info(root)
    dataset = LeRobotDataset(info["repo_id"], root=root, episodes=[args.episode], video_backend="torchcodec")
    states = np.asarray(dataset.hf_dataset["observation.state"], dtype=np.float32)
    actions = np.asarray(dataset.hf_dataset["action"], dtype=np.float32)
    all_positions = np.concatenate([states[:1], actions])
    if not np.isfinite(all_positions).all():
        raise ValueError("请核对回放角度数据")
    if (all_positions < LOW - 0.5).any() or (all_positions > HIGH + 0.5).any():
        raise ValueError(f"请核对回放范围：{all_positions.min(axis=0)}, {all_positions.max(axis=0)}")
    step = np.max(np.abs(np.diff(all_positions, axis=0)))
    if step > args.max_step:
        raise ValueError(f"每步角度变化 {step:.2f}°，请使用更缓慢的演示")
    print(json.dumps({"episode": args.episode, "frames": len(actions), "seconds": len(actions) / (FPS * args.rate),
                      "source_seconds": len(actions) / FPS, "playback_rate": args.rate,
                      "first_state": states[0].tolist(), "last_target": actions[-1].tolist(),
                      "max_step_degrees": float(step), "execute": args.execute}, ensure_ascii=False, indent=2), flush=True)
    if args.execute:
        settings = json.loads(Path(args.config).read_text())
        arm = open_arm(settings)
        readings = []
        try:
            current = read_state(arm)
            delta = float(np.max(np.abs(states[0] - current)))
            if delta > args.max_start:
                raise ValueError(f"起点姿态差值为 {delta:.2f}°，请先将机械臂靠近录制起点")
            arm.torque_set(cmd=1)
            for remaining in range(args.countdown, 0, -1):
                print(f"回放倒计时：{remaining} 秒", flush=True)
                time.sleep(1)
            current = read_state(arm)
            if float(np.max(np.abs(states[0] - current))) > args.max_start:
                raise ValueError("请将机械臂姿态重新对齐录制起点")
            send_target(arm, states[0], args.speed)
            time.sleep(1)
            start = time.perf_counter()
            sent_target = states[0]
            for index, target in enumerate(actions):
                time.sleep(max(0, start + index / (FPS * args.rate) - time.perf_counter()))
                actual = read_state(arm)
                tracking_error = np.abs(sent_target - actual)
                if float(np.max(tracking_error)) > args.max_step:
                    raise ValueError(f"第 {index} 帧跟随差值：{tracking_error.tolist()}，请重新对齐起点")
                send_target(arm, target, args.speed)
                readings.append({"frame": index, "time_s": time.perf_counter() - start,
                                 "state": actual.tolist(), "target": target.tolist(),
                                 "tracking_error_degrees": tracking_error.tolist()})
                sent_target = target
                if index % FPS == 0:
                    print(f"回放 {index + 1}/{len(actions)} 帧 · {index / (FPS * args.rate):.0f} 秒", flush=True)
            time.sleep(1)
            final_state = read_state(arm)
        finally:
            arm.disconnect()
        result = {"episode": args.episode, "frames_sent": len(readings), "playback_rate": args.rate,
                  "readings": readings,
                  "final_state": final_state.tolist(), "last_target": actions[-1].tolist(),
                  "final_error_degrees": np.abs(final_state - actions[-1]).tolist(),
                  "ending": "保持末帧姿态，串口已关闭"}
        report_path = root / f"capture/replay_{args.episode:04d}.json"
        report_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"report": str(report_path), "final_error_degrees": result["final_error_degrees"],
                          "ending": result["ending"]}, ensure_ascii=False, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "configs/roarm_m3.json"))
    commands = parser.add_subparsers(dest="command", required=True)
    rec = commands.add_parser("record", help="网页操控期间录制一段数据")
    rec.add_argument("--root", type=Path, required=True)
    rec.add_argument("--repo-id", default="local/roarm_web")
    rec.add_argument("--task", default="用网页缓慢移动底座并开合夹爪")
    rec.add_argument("--seconds", type=float, default=20)
    rec.add_argument("--countdown", type=int, default=5)
    rec.add_argument("--append", action="store_true")
    rec.set_defaults(func=record)
    verify = commands.add_parser("check", help="验证数据配对、图像和 ACT 读取")
    verify.add_argument("--root", type=Path, required=True)
    verify.add_argument("--act-forward", action="store_true")
    verify.set_defaults(func=check)
    play = commands.add_parser("replay", help="检查或执行机械臂轨迹回放")
    play.add_argument("--root", type=Path, required=True)
    play.add_argument("--episode", type=int, default=0)
    play.add_argument("--execute", action="store_true")
    play.add_argument("--max-step", type=float, default=5)
    play.add_argument("--max-start", type=float, default=3)
    play.add_argument("--speed", type=float, default=10)
    play.add_argument("--countdown", type=int, default=5)
    play.add_argument("--rate", type=float, default=1, help="回放倍率，0.25 为四分之一速度")
    play.set_defaults(func=replay)
    args = parser.parse_args()
    if hasattr(args, "seconds") and args.seconds < 1 / FPS:
        parser.error("录制时长请至少设为 0.2 秒")
    if hasattr(args, "countdown") and args.countdown < 0:
        parser.error("倒计时请使用零或正整数")
    for name in ("seconds", "speed", "max_step", "max_start", "rate"):
        if hasattr(args, name) and (not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0):
            parser.error(f"{name} 请使用有限正数")
    if hasattr(args, "rate") and args.rate > 1:
        parser.error("回放倍率请使用大于零、至多为 1 的数值")
    args.func(args)


if __name__ == "__main__":
    main()
