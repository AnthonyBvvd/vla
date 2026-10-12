# 网络连接

1. 连接RoArm-M3网络；
2. 打开192.168.4.1；
3. 输入如下json指令：

```JSON
{"T":404,"ap_ssid":"RoArm-M3","ap_password":"12345678","sta_ssid":"QAQ","sta_password":"12345678"}
```

4. 切换电脑网络到QAQ；
5. 查看机械臂显示屏上的IP，浏览器输入该IP即可。

# RoArm-M3 键盘控制与采集

关节模式采用 `T=121` 单关节目标控制；坐标模式采用原网页的 `T=123` 连续运动。按住方向键运动，松开持位；同模式支持多个轴一起运动，同一轴相反键同时按住时持位。

关节模式先读取当前角度，再每 0.1 秒更新被按下关节的目标。松手后读取新角度，并向该关节发送持位目标。Q/A 只更新底座，其他五个关节保留已有目标；夹爪方向与原网页一致，采集仍采用 `angular_direct` 角度定义。

在本机启动 USB 串口控制：

```bash
cd /home/mul/vla
source /home/mul/vla/env.sh
sg dialout -c 'bash /home/mul/vla/scripts/run_vla.sh roarm-keyboard'
```

`sg dialout` 使用本机已配置的串口用户组启动窗口；`scripts/run_vla.sh` 在该会话内加载项目环境，让本地 FFmpeg 库一起生效。已带该用户组的会话也可直接执行 `roarm-keyboard`。窗口中点击“开启保持力”（Torque ON），点击窗口后使用方向键。关节模式默认速度为 3°/s，首轮检查可设为 1°/s。

窗口初始大小按系统字体和屏幕尺寸计算，可拖动边缘调整大小。窄窗口中模式与按钮会纵向排列，文字自动换行；内容超过窗口高度时，可用鼠标滚轮或右侧滚动条查看。

| 按键 | 关节模式 | 坐标模式 |
|---|---|---|
| Q / A | 底座左 / 右 | X+ / X− |
| W / S | 肩关节下 / 上 | Y+ / Y− |
| E / D | 肘关节下 / 上 | Z+ / Z− |
| R / F | 腕倾＋ / − | Pitch＋ / − |
| T / G | 腕转＋ / − | Roll＋ / − |
| Y / H | 夹紧 / 张开 | 夹紧 / 张开 |

`Tab` 切换关节/坐标模式，`Space` 停止六轴，`Esc` 停止并结束。窗口失去焦点也会请求六轴停止。切换模式或速度、使用空格停止后，松开方向键，再重新按下继续。程序结束时保留关节保持力。

长按期间，系统自动重复可能发送成对的释放/按下事件。程序短等 30 毫秒确认松手；期间同键再次按下会取消释放任务，保持目标连续更新。Space 和窗口失焦继续即时请求停止。

点击“回到初始位置”或按 `Home`，程序先停止六轴，再发送微雪初始化指令 `{"T":100}`。机械臂按固件设置的初始姿态回位；等姿态到位后继续操作。长按 `Home` 触发一次，松开后可再次触发。使用前留出完整的回位活动空间。

局域网控制：

```bash
roarm-keyboard --host 192.168.1.134
```

指定串口、速度及启动模式：

```bash
roarm-keyboard --port /dev/ttyUSB0 --baudrate 115200 --speed 3 --mode joint
```

使用模拟通信查看按键指令：

```bash
roarm-keyboard --dry-run
```

默认串口路径读取 `/home/mul/vla/configs/roarm_m3.json`；通过 `--config` 可指定其他配置文件。HTTP 使用 `GET /js?json=...`，串口使用每行一条 JSON。T121 字段为 `joint/angle/spd/acc`，其中 `angle` 是绝对角度，单位为度。停止操作也逐轴发送 T123 的 `axis=1..6, cmd=0`，清理固件已有连续运动方向。

关节目标范围为：底座 ±180°、肩 ±90°、肘 0～180°、腕倾 ±90°、腕转 ±90°，夹爪 direct 角度 0～100°。通信线程持续检查反馈是否新鲜，等待反馈时的松手、回位和窗口关闭都会结束旧运动请求。

源码采用可编辑安装。保存修改后，关闭旧窗口并重新启动即可加载新代码。

## Action 6 与 5 Hz 录制

相机接入 USB 并选择 Webcam 模式后，默认 USB 键盘窗口会显示录制区。键盘指令与角度查询由同一个发送线程管理；录制读取实际六轴角度，保持原有按住运动、松开停止的操作。

```bash
cd /home/mul/vla
source /home/mul/vla/env.sh
sg dialout -c 'bash /home/mul/vla/scripts/run_vla.sh roarm-keyboard --record-root /home/mul/vla/datasets/roarm_keyboard --record-seconds 60 --speed 20'
```

1. 点击“开启保持力”，用短按检查方向并摆好起始姿态。
2. 按 `F9` 或点击“开始录制”，等待相机准备与 5 秒倒计时。
3. 出现“录制开始”后，通过原有六组按键完成演示。每段默认 20 秒，也可按 `F10` 提前结束并保存。
4. 保存期间程序请求六轴停止。看到“已保存”后，再按 `F9` 可向同一目录追加下一段。
5. 按 `Esc` 停止运动、保存当前段并结束窗口。

`Space` 停止运动后仍继续采样。窗口失去焦点也会请求六轴停止。录制准备和保存期间，方向键等待完成后继续使用；回位功能可在当前段保存完成后使用。

使用共用无线接收器的键鼠时，先做相机独立采集检查，观察鼠标移动和字母输入。相机传输期间若键鼠响应受到影响，可将相机接到 USB 2.0，或通过 USB 延长线把键鼠接收器放近键鼠、远离 USB 3.0 插口与相机线缆。确认输入连续后再开始机械臂演示。参考：[罗技 USB 3.0 干扰说明](https://support.logi.com/hc/en-ch/articles/360023414273-Wireless-product-not-working-properly-when-also-using-a-USB-3-0-device)。

输出使用 LeRobot 0.6.1 的 v3 数据格式，包含 640×360 RGB 视频、六项关节状态和六项目标角度。所有角度单位为度，顺序是底座、肩、肘、腕倾、腕转、夹爪；夹爪与本机 follower 的 `angular_direct` 定义一致。每个数据行使用图像及状态 `t`，动作取下一次实际姿态 `t+1`；20 秒采集 101 次，保存 100 个完整数据行。

`capture/info.json` 记录相机和动作定义，`capture/episode_0000.json` 等记录实际采样时间和角度。新增 `control_protocol` 和 `control_events`：记录方向请求、实际发送的 JSON 及其相对秒数，便于检查按键与关节反馈的对应。数据检查和回放复用项目已有程序：

```bash
uv run python scripts/web_capture.py check --root /home/mul/vla/datasets/roarm_keyboard --act-forward
uv run python scripts/web_capture.py replay --root /home/mul/vla/datasets/roarm_keyboard
```

实机回放前关闭键盘窗口，机械臂靠近录制起点，留出活动空间，然后运行：

```bash
sg dialout -c 'bash /home/mul/vla/scripts/run_vla.sh python /home/mul/vla/scripts/web_capture.py replay --root /home/mul/vla/datasets/roarm_keyboard --execute --rate 0.25'
```

可用 `--task` 设置演示内容、`--record-seconds` 设置时长、`--record-countdown` 设置倒计时。局域网模式继续提供键盘控制，USB 模式提供整合录制。

回放 `--rate 0.25` 按四分之一速度发送原有目标，20 秒数据约用 80 秒回放；数据集本身保持 5 Hz。

本机测试：

```bash
uv run python -m unittest discover -s tests -v
```

覆盖按键重复、松手持位、反向键、多轴、夹爪方向、反馈等待期间的回位及关闭、5 Hz 采样、录制指令记录、发送顺序与清理，并使用模拟通信、PTY 和本机 HTTP 服务验证数据发送。

参考：[微雪 JSON 连续运动指令](https://docs.waveshare.net/RoArm-M3/JSON-Command-Control/#cmd_constant_ctrl---连续运动控制角度控制--逆运动学控制)。
