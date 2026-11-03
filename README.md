# 睿尔曼遥操代码

**v1.1.0：Quest 原生六自由度遥操 + 双 D405 RGB-D 采集 + A/B 分段录制。**

使用流程：安装 → 配置机器人和两台相机 → 手动设置方向 → 只读预览 → 遥操或双机位录制。平移和转腕使用同一套已保存方向；松 Grip 后重新握住只重建相对起点，不改变方向设置。默认转腕比例 0.5。

无需 ROS 或 Unity 编辑器。发布包附带适配 Quest APK；源码仓库安装时自动从 Release 获取并校验。当前版本保留相机原始资料与操作者标签，不包含私人标定、录制数据、模型或训练程序。

## 支持环境

| 项目 | 适配范围 |
| --- | --- |
| 电脑 | Ubuntu 24.04 x86_64，Python 3.12 |
| 机械臂 | RealMan RM65-BI；控制和规划固件 V1.7.4 |
| 工作坐标系 | Base，零偏移 |
| 工具 | 名称 tcp，法兰 Z 偏移 145 mm，无旋转 |
| 负载 | 0.5 kg，质心 `[0,0,0]`；碰撞等级 4 |
| 头显 | Meta Quest 2 / Quest 3，开发者模式与 USB 调试已授权 |
| 相机 | 两台 Intel RealSense D405，分别绑定 wrist 与 external 序列号 |
| 图像流 | 每台 640×480@30 FPS RGB-D；默认约 10 FPS 保存 |

其他机器人、固件、TCP、负载需要同步适配校验与离线运动学模型。程序不会自动使能、回零、修改工具、负载或碰撞设置。相机的 USB 带宽、供电及电脑磁盘写入能力须能支持两路图像流。

## 1. 下载和安装

推荐 [完整运行包](https://github.com/ccstudylearn/realman-teleop/releases/download/v1.1.0/realman_quest_teleop-v1.1.0-ubuntu24.04-x86_64.tar.gz)：附带 PC Service、5 个 Python wheels 和 Quest APK。也可下载 [源码包](https://github.com/ccstudylearn/realman-teleop/releases/download/v1.1.0/realman_quest_teleop-v1.1.0-source.zip)，或克隆仓库。

解压后，在 `robot.sh` 所在目录执行：

```bash
sudo apt update
sudo apt install -y python3.12 python3.12-venv adb dpkg \
  libgl1 libegl1 libxkbcommon0 libdbus-1-3 libfontconfig1 libxcb1 libx11-6 \
  libusb-1.0-0 libudev1
bash setup.sh
```

安装创建项目自己的 `.venv`，锁定 NumPy、SciPy、Robotic_Arm、OpenCV 与 pyrealsense2；PC Service 解压到 `.native/`，不安装成系统服务。安装只检查依赖和离线运动学，不启动机械臂或相机。

源码仓库首次安装需联网下载依赖；完整运行包优先使用附带 DEB 和 wheels。Ubuntu 系统包仍需 apt 软件源或预先安装。PC Service 使用[上游 v1.0.0 Ubuntu 24.04 发布包](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/tag/v1.0.0)，下载失败可手动放入 `vendor/` 后重新安装。APK/DEB 哈希在 [vendor/manifest.json](vendor/manifest.json)。

## 2. 配置机器人、相机和保存目录

先连接机器人网络、Quest USB 和两台 D405，戴上头显授权 USB 调试。此时先不开 Viewer 或其他占用相机流的程序。

```bash
adb devices -l
bash robot.sh cameras
```

`cameras` 只枚举设备名称与序列号，不开启采集或机器人控制。识别哪台是近景、哪台是外部相机后，将下面的序列号和 IP 替换为自己的值：

```bash
bash robot.sh configure --robot-ip 192.168.0.18 \
  --wrist-serial WRIST_SERIAL --external-serial EXTERNAL_SERIAL \
  --data-dir data
bash robot.sh install-quest
```

两台相机必须使用不同序列号，不按 USB 枚举顺序自动决定角色。多个 ADB 设备时追加 `--serial YOUR_QUEST_SERIAL`。`data` 相对路径基于项目目录，也可以填写其他可写绝对路径。

保存到外接或独立挂载的数据盘时，建议明确挂载点：

```bash
bash robot.sh configure --robot-ip 192.168.0.18 \
  --wrist-serial WRIST_SERIAL --external-serial EXTERNAL_SERIAL \
  --data-dir /mnt/data/realman_quest --mount-point /mnt/data
```

配置后，挂载点不存在或目录不在该挂载点内会拒绝录制，不会自动回退到系统盘。录制盘不足 1 GiB 或不可写也会拒绝启动；1 GiB 是启动下限，持续录制需要更多空间。

| 本机配置 | 用途 |
| --- | --- |
| `config/robot.json` | IP、端口和预期硬件配置 |
| `config/follow.json` | 手动方向、速度、夹爪和两路相机参数 |
| `config/devices.json` | 可选 Quest USB 序列号 |
| `config/storage.json` | 数据根目录和可选挂载点要求 |

这些文件和数据目录由 `.gitignore` 排除。再次 `configure` 会按模板重建设置并备份旧文件；日常调参可直接编辑自己的配置，或使用 `direction` 修改方向。

## 3. 手动设置方向与只读预览

```bash
bash robot.sh monitor --seconds 30
bash robot.sh direction
bash robot.sh direction --show
bash robot.sh check
bash robot.sh preview --seconds 60
```

程序自动启动或复用 PC Service，设置 USB 转发并打开适配 Quest 应用。头显内连接 `127.0.0.1`，开启 Controller 和 Send，保持佩戴及前台焦点。监视时确认帧数增加、`stream_fresh=true`、`right.valid=true`，移动手柄和按松按钮时数据变化。

`direction` 依次输入：

1. Base X、Y、Z 分别取哪个 Quest 轴，例如 `-x,z,y`。
2. 映射后绕 Base +Z 的偏航修正角，回车为 0。

也可以明确填写自己的设置：

```bash
bash robot.sh direction --axis-map=-x,z,y --yaw-deg 0
```

上述数值只是示例：零偏航时 `Base ΔX=-Quest ΔX`、`Base ΔY=Quest ΔZ`、`Base ΔZ=Quest ΔY`。轴必须各出现一次并组成旋转，镜像映射会被拒绝。非零偏航按右手规则绕 Base +Z 修正；同一矩阵同时用于平移与相对转腕。

方向只保存到本机，不连接或驱动机械臂。保存标志仅代表已经设置，不代表实际方向通过硬件验收。头显重新居中、应用重启或站位改变后，重新预览核对。运行中不热加载方向，修改前先退出遥操。

`check` 只读核对硬件和 FK；`preview` 读取反馈、计算目标但不发送运动、停止或夹爪写入命令。预览分别检查右移、前移、上抬和转腕，松 Grip 后 `target_xyz=None`。未设置方向时，真机 `run` 和 `record` 会拒绝启动。

## 4. 双相机检查与 A/B 录制

先单独检查双相机的启动、分段保存、PNG 可读性和一对一配对：

```bash
bash robot.sh camera-check --seconds 5 --episodes 2
```

此命令仅连接两台相机，不连接机械臂或 Quest；自动生成的段是落盘检查，不是成功任务示范。

完成方向预览、空载小范围运动和夹爪检查后，由操作者执行：

```bash
bash robot.sh record --task "把物体放进托盘"
```

**`record` 会直接开启真机遥操，无需 START。A/B 控制录制与结果标签，不是机械臂运动授权开关；Grip 才是跟随接合键。**

| 操作 | 行为 |
| --- | --- |
| 待机按右 A | 同一条示范开始记录两路相机和机器人资料 |
| 录制中按右 A | 标记成功，停止并保存该条 |
| 录制中按右 B | 标记失败，停止并保存该条 |
| 保存完成后松开 A/B，再按右 A | 录制下一条；无需重连相机 |
| 握右 Grip 平移或转腕 | 相对六自由度跟随；松 Grip 请求停止并核对反馈 |
| 保持 Grip，按/松 Trigger | 夹取/张开；启动或重新握 Grip 不自动松爪 |
| Ctrl+C | 停止会话并保存，未结束条目保留为中断 |

只遥操、不录制时使用 `bash robot.sh run --seconds 60`，无需接入相机。首次有效帧若 Grip 已握住，会建立当前零位移锚点，建议启动前先松开 Grip。夹爪需显示 `gripper=ready`；ACK 不等于真实夹牢。

详细数据目录、同步边界和故障处理见 [DUAL_CAMERA_zh.md](DUAL_CAMERA_zh.md)。

## 控制参数与停止保护

模板参数：平移比例 1.0、转腕比例 0.5、控制目标 50 Hz、TCP 上限 100 mm/s、关节上限 45°/s、关节加速度 120°/s²、启动 TCP 周围半径 400 mm。重新握 Grip 不扩大该范围。`--seconds` 可缩短运行时长。

保留输入过期、重复帧、追踪/焦点丢失、位姿跳变、关节限位、奇异性、自碰撞、刹车轨迹、反馈时效和配置变化检查。故障后松开再重新握 Grip，不保持握住自动恢复。任一路相机故障或有效帧心跳停滞约 1 秒，会进入现有停止流程。

模型自碰撞检查不包含桌面、障碍物、相机支架或被抓物体。软件限速和工作空间不能代替真实净空检查。普通 Linux/Python 不保证硬实时；`stop_unverified` 时使用实体急停并检查控制器。

## 代码与验证范围

- `teleop.py`、`robot.sh`：统一入口、设备配置、手动方向与启动。
- `realman_follow.py`、`robot_core.py`、`follow_transport.py`、`gripper_transport.py`：跟随、预检与停止保护。
- `xrobotoolkit_input.py`：原生输入及 A/B、Grip、Trigger 的逐帧状态。
- `episode_recording.py`、`episode_session.py`、`camera_layout.py`、`follow_recording.py`：双路采集、分段、配对与有界日志。
- `recording_storage.py`：可配置保存位置、挂载及空间检查。
- `check_dual_camera_recording.py`：仅相机落盘检查。
- `setup.sh`、`install_runtime.py`、`start_service.sh`：锁定依赖、下载校验与服务。
- `config/`：公开模板；`client/`：适配客户端补丁及构建依据。

离线回归和独立环境安装用于检查代码与数据完整性，不能证明目标电脑上的双路采集或真实抓放成功。发布版本仍需按上述步骤在使用者自己的硬件上验收。

## 许可与来源

保留 [MIT License](LICENSE)。Quest 客户端来源与重建步骤见 [client/README.md](client/README.md)。第三方归属及原始许可见 [THIRD_PARTY.md](THIRD_PARTY.md) 和 `licenses/`。
