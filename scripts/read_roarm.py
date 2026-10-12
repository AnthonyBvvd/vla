"""通过串口读取 RoArm-M3 六项关节角度。"""

import argparse
import json
import math

from roarm_sdk.roarm import roarm


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--port", required=True, help="串口路径，如 /dev/ttyUSB0")
parser.add_argument("--baudrate", type=int, default=115200)
args = parser.parse_args()

arm = roarm(roarm_type="roarm_m3", port=args.port, baudrate=args.baudrate)
try:
    angles = arm.joints_angle_get()
    assert isinstance(angles, (list, tuple)) and len(angles) == 6, f"串口反馈：{angles!r}"
    assert all(math.isfinite(float(angle)) for angle in angles), f"角度反馈：{angles!r}"
    names = ("base", "shoulder", "elbow", "wrist_tilt", "wrist_roll", "gripper")
    print(json.dumps(dict(zip(names, angles)), ensure_ascii=False, indent=2))
finally:
    arm.disconnect()
