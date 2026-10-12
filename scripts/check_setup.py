"""查看项目软件版本、插件发现及视频读取结果。"""

import importlib.metadata
import json
from pathlib import Path

import torch
from lerobot.robots.config import RobotConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.utils.import_utils import register_third_party_plugins
from torchcodec.decoders import VideoDecoder


register_third_party_plugins()
config_class = RobotConfig.get_choice_class("roarm_m3_follower")
robot = make_robot_from_config(
    config_class(id="roarm_m3", port="/dev/ttyUSB0", baudrate=115200, max_relative_target=5.0)
)
assert len(robot.action_features) == 6
assert robot.config.baudrate == 115200
assert robot.observation_features == robot.action_features

result = {
    "versions": {
        name: importlib.metadata.version(name)
        for name in ("lerobot", "torch", "torchvision", "torchcodec", "roarm_sdk", "lerobot_robot_roarm_m3")
    },
    "torch_backend": "cpu" if torch.version.cuda is None else torch.version.cuda,
    "robot_type": robot.config.type,
    "joint_keys": list(robot.action_features),
    "video_frames": {},
}
project_dir = Path(__file__).resolve().parents[1]
for name in ("h264", "av1"):
    decoder = VideoDecoder(str(project_dir / ".tools/ffmpeg/checks" / f"{name}.mp4"))
    frame = decoder[0]
    assert tuple(frame.shape) == (3, 128, 160)
    result["video_frames"][name] = list(frame.shape)

print(json.dumps(result, ensure_ascii=False, indent=2))
