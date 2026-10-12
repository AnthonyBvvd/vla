"""读取项目相机画面，保存检查结果或打开实时预览。"""

import argparse
import json
import subprocess
import time
from pathlib import Path

import cv2
import draccus
from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.cameras.opencv import OpenCVCameraConfig


project_dir = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--config", default=str(project_dir / "configs/roarm_m3.json"))
parser.add_argument("--camera", default="front")
parser.add_argument("--seconds", type=float, default=5)
parser.add_argument("--preview", action="store_true")
args = parser.parse_args()

settings = json.loads(Path(args.config).read_text(encoding="utf-8"))
config = draccus.decode(CameraConfig, settings["cameras"][args.camera])

if args.preview:
    command = [
        str(project_dir / ".tools/ffmpeg/usr/bin/ffplay"),
        "-hide_banner", "-loglevel", "warning",
        "-f", "v4l2", "-input_format", "mjpeg",
        "-video_size", f"{config.width}x{config.height}",
        "-framerate", str(config.fps),
        "-window_title", "DJI Action 6 - front", "-x", "960", "-y", "540",
    ]
    command.append(str(config.index_or_path))
    process = subprocess.Popen(command)
    try:
        process.wait(timeout=args.seconds if args.seconds > 0 else None)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=5)
else:
    assert args.seconds > 0
    camera = make_cameras_from_configs({args.camera: config})[args.camera]
    try:
        camera.connect()
        frames = 0
        start = time.perf_counter()
        while time.perf_counter() - start < args.seconds:
            frame = camera.read()
            assert frame.shape == (config.height, config.width, 3)
            assert str(frame.dtype) == "uint8"
            frames += 1
        elapsed = time.perf_counter() - start
        output_dir = project_dir / ".cache/action6"
        output_dir.mkdir(parents=True, exist_ok=True)
        preview_path = output_dir / f"{args.camera}.jpg"
        assert cv2.imwrite(str(preview_path), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        result = {
            "camera": args.camera,
            "device": str(config.index_or_path),
            "width": config.width,
            "height": config.height,
            "configured_fps": config.fps,
            "fourcc": config.fourcc,
            "color_mode": config.color_mode.value,
            "frames_read": frames,
            "elapsed_seconds": round(elapsed, 3),
            "measured_fps": round(frames / elapsed, 2),
            "preview": str(preview_path),
        }
    finally:
        if camera.is_connected:
            camera.disconnect()
    result["disconnected"] = camera.is_connected is False
    report = json.dumps(result, ensure_ascii=False, indent=2)
    (output_dir / f"{args.camera}-check.json").write_text(report + "\n", encoding="utf-8")
    print(report)
