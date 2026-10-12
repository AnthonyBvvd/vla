# RoArm-M3 本机环境

软件、源代码和下载缓存均位于 `/home/mul/vla`。

键盘连续运动小包位于 `packages/roarm-keyboard`。激活项目环境后运行 `roarm-keyboard`，默认通过 USB 串口控制；`roarm-keyboard --host 192.168.1.134` 使用当前 QAQ 局域网地址。操作方式和检查命令见该包的 `README.md`。

关节模式已适配 `T121` 单关节目标：按住时以 0.1 秒间隔更新所选关节，松手读取新角度并持位；坐标模式保留 `T123`。关节速度单位为度/秒，默认 3°/s。录制 JSON 同时保存操作方向、实际指令和时间。源码可编辑安装，保存后重启窗口即可加载。

- uv 0.13.0：`.tools/bin`
- Python 3.12.3 环境：`.venv`
- LeRobot 0.6.1：`vendor/lerobot`，以可编辑方式安装
- RoArm-M3 驱动 0.1.0：`vendor/lerobot_robot_roarm_m3`，以可编辑方式安装
- PyTorch 2.10.0 CPU、TorchVision 0.25.0 CPU、TorchCodec 0.10.0
- RoArm SDK 0.1.1、FFmpeg 6.1.1

每次打开终端后运行：

```bash
cd /home/mul/vla
source /home/mul/vla/env.sh
```

软件检查：

```bash
uv run python scripts/check_setup.py
uv pip check
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run python -m pytest vendor/lerobot_robot_roarm_m3/tests -q
```

测试环境变量让 pytest 使用本项目测试插件。系统 ROS 同时提供的附加测试插件由其自身环境加载。

串口检查：固定底座，接入独立电源并开机，再通过数据线连接驱动板串口 Type-C 接口。使用实际串口路径执行：

```bash
uv run python -m serial.tools.list_ports -v
uv run python scripts/read_roarm.py --port /dev/ttyUSB0
```

角度读取脚本发送反馈查询，结束时关闭串口。关节角度以度表示。

本机已识别 CP2102N 串口 `/dev/ttyUSB0`，账户 `mul` 已加入 `dialout` 组。当前终端需要启用该组时运行 `newgrp dialout`，再执行上述环境启用命令。重新登录后的会话会自动带上该组。

`configs/roarm_m3.json` 已保存机械臂的固定设备路径、115200 波特率及六项关节配置。LeRobot 实机连接检查前，用支撑物托稳机械臂：

```bash
uv run python scripts/check_robot.py
```

检查会开启关节保持力，读取六项角度，再结束连接并释放保持力。检查结束后继续保持支撑。结果保存在 `.cache/roarm-connection.json`。

LeRobot 配置使用 `robot.type=roarm_m3_follower`，原厂固件填写 `robot.baudrate=115200`，并填写实际 `robot.port`。单次目标变化可设置 `robot.max_relative_target=5.0`。同一串口交给一个程序使用。

当前本地适配将驱动 `roarm_m3_follower.py` 的类型导入路径设为 `lerobot.lerobot_types`。软件检查覆盖插件发现、工厂创建、六项关节字段、视频读取及驱动原有的 8 项测试。2026-10-11 已完成 SDK 角度读取及 LeRobot 实机连接、观测读取和结束连接检查。

项目提供 follower；LeRobot 遥操作和录制需要配套遥操作输入。

## DJI Action 6 相机

相机开机后，通过 USB 数据线接到电脑，在相机屏幕选择“网络摄像头 / Webcam”。

`configs/roarm_m3.json` 已将 Action 6 配置为 `front` 相机：固定设备路径、1280×720、30 FPS、MJPEG 输入、RGB 图像输出。LeRobot 数据字段为 `observation.images.front`。

2026-10-11 已通过本机 UVC 和 LeRobot OpenCVCamera 验证，5.025 秒读取 151 帧，实测约 30.05 FPS。图像与结果保存在 `.cache/action6/front.jpg` 和 `.cache/action6/front-check.json`。

相机检查与持续实时预览：

```bash
uv run python scripts/check_camera.py
uv run python scripts/check_camera.py --preview --seconds 0
```

实时预览按 `q` 结束。同一时间使用相机的一个采集程序；结束预览后启动录制程序。相机输出为 30 FPS，首轮夹持数据按 5 Hz 采样。数据采集阶段可将图像等比例缩小到 640×360，并在训练与运行时使用一致的图像处理方式。

相机单独检查会打开视频接口。机械臂联合检查使用 `scripts/check_robot.py`，按前文先托稳机械臂。

参考：[DJI 网络摄像头说明](https://repair.dji.com/help/content?customId=zh-cn03400006962&lang=zh-CN&re=CN&spaceId=34)。

## 键盘控制与采集

USB 键盘窗口已整合 Action 6 图像与六轴实际角度采集。操作、代码及数据均位于本项目。

```bash
cd /home/mul/vla
source /home/mul/vla/env.sh
sg dialout -c 'bash /home/mul/vla/scripts/run_vla.sh roarm-keyboard --record-root /home/mul/vla/datasets/roarm_keyboard --record-seconds 20 --speed 1'
```

点击“开启保持力”，按 F9 准备录制；5 秒倒计时结束后使用原有运动键。默认录制 20 秒，按 F10 可提前保存。程序以 5 Hz 保存 LeRobot v3 数据：640×360 RGB 图像、六轴状态，以及下一次实际姿态作为动作。保存完成后再次按 F9 可追加新段；Esc 停止并保存后退出。操作详情见 `packages/roarm-keyboard/README.md`。

实机检查与回放使用以下命令。回放前结束键盘窗口、靠近录制起点并留出活动空间。

```bash
uv run python scripts/web_capture.py check --root /home/mul/vla/datasets/roarm_keyboard --act-forward
uv run python scripts/web_capture.py replay --root /home/mul/vla/datasets/roarm_keyboard
sg dialout -c 'bash /home/mul/vla/scripts/run_vla.sh python /home/mul/vla/scripts/web_capture.py replay --root /home/mul/vla/datasets/roarm_keyboard --execute --rate 0.25'
```

2026-10-11 已完成整合窗口实机录制与回放：`datasets/roarm_keyboard_demo_20261011` 包含一段 20 秒演示、100 帧、5 Hz、640×360 视频；六轴状态与下一次姿态配对，ACT 前向及反向计算通过。四分之一速度实机回放发送了全部 100 帧，末帧最大差值约 3.69°，结束时保留保持力并关闭串口。详细结果为 `capture/dataset_check.json` 和 `capture/replay_0000.json`。

该段包含底座、肩、肘和腕倾运动，夹爪保持原角度。首轮静态画面检查另存于 `datasets/roarm_keyboard_trial_20261011`。后续夹持演示可记录夹爪开合、夹取及放下动作。

本机键鼠共用罗技无线接收器，Action 6 当前连接速度为 USB 3.0 的 5000M。调整接收器位置后，已确认相机采集期间键鼠正常，并完成上述运动录制。保持该连接布局，后续换口可先做短时相机与输入检查。`scripts/run_vla.sh` 在串口用户组内加载项目环境，使本地 FFmpeg 和 TorchCodec 一起生效。

## 微雪原网页采集

手机连接机械臂热点并使用微雪原网页，电脑保留现有网络。机械臂 USB 与 Action 6 摄像头都连接电脑。网页负责运动，采集程序以 USB 查询六项关节角度，并读取 RGB 图像。

新程序为 `scripts/web_capture.py`，数据保存到项目的 `datasets` 目录。每次录制一段，已有数据可添加 `--append` 继续增加演示。

```bash
cd /home/mul/vla
source /home/mul/vla/env.sh
uv run python scripts/web_capture.py record \
  --root /home/mul/vla/datasets/roarm_web \
  --repo-id local/roarm_web --seconds 20 \
  --task "用网页缓慢移动底座并开合夹爪"
```

按倒计时和“录制开始”提示操作手机网页。采样完成后松开运动按钮，程序保存当前演示。终端按 Ctrl+C 会保存已采集的完整配对。

数据使用 LeRobotDataset v3、5 Hz、640×360 RGB 视频；状态与动作各为六项 float32 角度，顺序与当前 follower 相同。夹爪采用 SDK `angular_direct` 定义。每帧图像和状态来自第 t 次采样，动作来自第 t+1 次观测姿态；最后一次采样提供末行动作。`capture` 目录保存采样时间、状态、配置与动作生成方式。

数据检查与 ACT 读取检查：

```bash
uv run python scripts/web_capture.py check \
  --root /home/mul/vla/datasets/roarm_web --act-forward
```

检查包含原始记录和数据行的配对、时间、演示边界、RGB 解码、统计信息、10 帧动作片段及段尾补齐。`--act-forward` 运行小尺寸 ACT 配置的一次前向和反向计算，用于确认数据能进入训练流程。

机械臂回放先查看数据目标：

```bash
uv run python scripts/web_capture.py replay --root /home/mul/vla/datasets/roarm_web
```

底座固定、活动区域留空，手机网页松开运动按钮，机械臂靠近录制起点后执行：

```bash
uv run python scripts/web_capture.py replay \
  --root /home/mul/vla/datasets/roarm_web --execute
```

回放前检查角度范围、相邻目标变化和当前位置。默认每步变化上限 5°、起点最大差值 3°、速度 10°/s。回放结束后保留末帧姿态，USB 串口关闭，记录保存为 `capture/replay_0000.json`。

首轮实机检查可添加 `--rate 0.25`，按四分之一速度回放原有目标。回放比较实际角度与已发送目标，记录各帧跟随差值；启动时会检查串口的单程序使用情况。

数据图像和 5 Hz 节奏在训练与运行阶段保持一致。首轮 ACT 可采用 `chunk_size=10`、`n_action_steps=1`；训练和运行阶段也采用 `angular_direct` 夹爪定义。

重新安装依赖：

```bash
uv pip install --no-sources --torch-backend cpu -r requirements.txt
```

参考：[LeRobot 0.6.1](https://github.com/huggingface/lerobot/tree/v0.6.1)、[RoArm-M3 驱动](https://github.com/metrox-eth/lerobot_robot_roarm_m3)、[微雪入门教程](https://docs.waveshare.net/RoArm-M3/Introduction/)。
