"""使用项目配置连接 RoArm-M3，读取 LeRobot 观测并结束连接。"""

import argparse
import json
import math
from pathlib import Path

import draccus
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots.config import RobotConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.utils.import_utils import register_third_party_plugins


project_dir = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--config", default=str(project_dir / "configs/roarm_m3.json"))
args = parser.parse_args()

register_third_party_plugins()
config = draccus.parse(RobotConfig, config_path=args.config, args=[])
robot = make_robot_from_config(config)
try:
    robot.connect()
    observation = robot.get_observation()
    angles = {key: observation[key] for key in robot.action_features}
    assert len(angles) == 6, f"关节反馈：{angles!r}"
    assert all(math.isfinite(float(value)) for value in angles.values()), f"角度反馈：{angles!r}"
    images = {key: list(observation[key].shape) for key in robot.cameras}
    result = {"robot_type": config.type, "port": config.port, "baudrate": config.baudrate, "observation": angles, "images": images}
finally:
    if robot.is_connected:
        robot.disconnect()

result["disconnected"] = robot.is_connected is False
assert result["disconnected"]
report = json.dumps(result, ensure_ascii=False, indent=2)
(project_dir / ".cache/roarm-connection.json").write_text(report + "\n", encoding="utf-8")
print(report)
