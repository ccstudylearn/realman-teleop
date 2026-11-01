# 睿尔曼遥操代码

使用 Meta Quest 原生手柄控制睿尔曼 RM65-BI 机械臂，实现相对六自由度跟随和 Trigger 夹爪控制。

使用流程：**安装 → 配置 IP → 手动设置方向 → 只读预览 → 按保存的方向遥操**。方向设置保存在本机；平移和转腕共用同一套映射，松 Grip 再握住只重建相对起点，不改变方向设置。

本仓库只保留遥操运行所需代码、配置模板与安装脚本。首次安装需要下载 Python 依赖与 XRoboToolkit PC Service。源码仓库中的 `setup.sh` 还会从本仓库 Release 下载并校验适配 Quest APK。下载 Release 中的完整运行包可省去这些下载。

## 支持环境

| 项目 | 当前适配范围 |
| --- | --- |
| 电脑 | Ubuntu 24.04，x86_64，Python 3.12 |
| 机械臂 | RealMan **RM65-BI**，控制和规划固件 **V1.7.4** |
| 坐标系 | `Base`，零偏移 |
| 工具 | 名称 `tcp`，沿法兰 Z 轴偏移 **145 mm**，无旋转 |
| 工具负载 | **0.5 kg**，负载质心 `[0,0,0]` |
| 碰撞等级 | **4**；程序只检查，不自动修改 |
| 头显 | Meta Quest 2 / Quest 3，开发者模式，已授权 USB 调试 |
| 输入方式 | USB + XRoboToolkit RealMan 适配客户端，包名 `com.xrobotoolkit.client.quest.realman` |
| 夹爪 | 支持 RealMan `get_gripper_state` / `set_gripper_pick` / `set_gripper_release` 的夹爪 |

**“下载可用”指上述适配范围内，完成设备配置与检查后可运行。** 其他型号、固件、工具几何或负载需要同步适配配置校验和离线运动学模型，不能仅修改 IP 或删除校验。当前模型中的工具参数与上表一致。

机械臂须由操作者提前正确设置工具、负载、碰撞等级并使能六轴。程序不会自动使能、回零、改变工具或负载，也不会在退出时返回起点。

## 1. 下载与安装

推荐下载 [完整运行包](https://github.com/ccstudylearn/realman-teleop/releases/download/v1.0.0/realman_quest_teleop-ubuntu24.04-x86_64.tar.gz)，解压后进入 `realman_quest_teleop` 文件夹。也可下载 [源码包](https://github.com/ccstudylearn/realman-teleop/releases/download/v1.0.0/realman_quest_teleop-source.zip)。这两个发布包均附带适配 Quest APK。

下载本仓库 ZIP 并解压，或克隆仓库，然后在 `robot.sh` 所在目录打开终端。文件夹可以放在任意路径，包括中文路径。

安装系统依赖：

```bash
sudo apt update
sudo apt install -y python3.12 python3.12-venv adb dpkg \
  libgl1 libegl1 libxkbcommon0 libdbus-1-3 libfontconfig1 libxcb1 libx11-6
bash setup.sh
```

`setup.sh` 会创建项目自己的 `.venv`，安装锁定版本的 `Robotic_Arm`、NumPy、SciPy，并将 PC Service 解压到 `.native/`，无需将厂商服务安装到系统目录。安装脚本检查 APK 和 PC Service 的 SHA256，并运行离线运动学检查；不会连接机械臂或启动遥操。

PC Service 使用[上游 v1.0.0 发布包](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/tag/v1.0.0)。下载失败时，可从该页面手动下载 `XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb`，放入 `vendor/` 后重跑 `bash setup.sh`。哈希记录在 [vendor/manifest.json](vendor/manifest.json)。

完整运行包已包含该 DEB 和 `vendor/wheels/`；安装系统依赖后，同样运行 `bash setup.sh`，脚本会优先使用本地文件。完整运行包不包含 Ubuntu 系统包，因此全新系统的 `apt install` 仍需网络或已有离线软件源。

## 2. 连接设备并生成配置

1. 用网线连接电脑和机械臂，为电脑网卡设置与机器人相同网段的地址。
2. 用 USB 连接 Quest，佩戴头显，在弹窗中允许 USB 调试。
3. 执行以下命令，将示例 IP 替换为实际机械臂 IP：

```bash
adb devices -l
bash robot.sh configure --robot-ip 192.168.0.18
bash robot.sh install-quest
```

`adb devices` 中 Quest 应显示为 `device`，不能是 `unauthorized` 或 `offline`。有多个 ADB 设备时，配置命令追加 `--serial YOUR_QUEST_SERIAL`。序列号会保存到本机配置，无需改源码。

配置命令生成：

| 本机文件 | 用途 |
| --- | --- |
| `config/robot.json` | IP、端口、反馈时效及预期硬件配置 |
| `config/follow.json` | 手动设置的方向映射、位移/转腕比例、速度与范围 |
| `config/devices.json` | 可选的 Quest USB 序列号；未指定时检测唯一已授权设备 |

这些文件由 `.gitignore` 排除。再次运行 `configure` 会按模板重建配置，并保存旧文件备份；调整速度、比例时直接编辑本机 `config/follow.json`。

安装命令使用 `vendor/quest-realman.apk`：发布包已附带该文件，克隆源码仓库后由 `setup.sh` 自动获取。也可从 [Release](https://github.com/ccstudylearn/realman-teleop/releases/tag/v1.0.0) 手动下载 APK 放入 `vendor/`，文件哈希须与清单一致。适配客户端包含逐手柄追踪状态、应用焦点和佩戴状态字段；原版客户端缺少这些字段时不能授权运动。适配客户端与原版使用不同包名。

## 3. 先确认手柄，再做只读预览

先监视手柄，此命令不连接机械臂：

```bash
bash robot.sh monitor --seconds 30
```

程序自动启动或复用 PC Service、设置 USB 端口转发并打开适配应用。头显中使用 `127.0.0.1` 连接，开启 `Controller` 和 `Send`，保持佩戴且应用在前台。查看终端：`frames` 持续增加、`stream_fresh=true`、`right.valid=true`，移动手柄时 `position_m` 变化，按松 Grip/Trigger 时对应值变化。

确认手柄有效后，先手动设置方向，再运行硬件检查与预览：

```bash
bash robot.sh direction
bash robot.sh check
bash robot.sh preview --seconds 60
```

`check` 只读取机械臂状态、工具、负载、使能、固件和关节限位，并比较模型 FK 与实测 TCP；`preview` 读取机器人并计算目标关节，但不会发送运动、停止或夹爪写入命令。机器人配置不匹配、起始位置靠近限位或运动学不匹配时会拒绝继续。

预览时分别尝试右移、前移、上抬和小角度转腕，观察 `target_xyz` / `target_rpy`，确认方向符合你的站位与任务。松 Grip 后应显示 `target_xyz=None`；中断追踪或停止数据流后目标也应清除。

### 手动设置方向，然后遥操使用该方向

运行 `bash robot.sh direction` 后，终端依次要求输入：

1. 轴映射：Base X、Y、Z 分别对应哪个 Quest 轴，使用逗号分隔，例如 `-x,z,y`。
2. 偏航修正：轴映射之后绕机器人 Base +Z 旋转的角度，回车为 0。用于手柄参考方向与机器人坐标没有严格对齐的情况。

该命令只修改本机配置，不连接机械臂、头显或相机。保存时备份旧配置，保留已调整的速度、范围、转腕比例和夹爪参数。之后每次遥操都读取保存结果，直到使用者重新修改。

也可以用命令明确填写自己的设置，以下数值仅为示例：

```bash
bash robot.sh direction --axis-map=-x,z,y --yaw-deg 0
bash robot.sh direction --show
```

`--show` 显示当前映射、旋转矩阵和是否已手动设置。未完成方向设置时，`run` 会拒绝启动；可以使用只读 `monitor` 和 `preview` 检查数据。

`axis_map` 表示 **Base X、Y、Z 分别取 Quest 的哪个轴**，属于当前头显参考空间与机器人基坐标系的对应关系，不是通用的“向右/向前”定义。

例如 `-x,z,y` 表示：

```text
Base ΔX = -Quest ΔX
Base ΔY = +Quest ΔZ
Base ΔZ = +Quest ΔY
```

偏航修正为 0 时，上式直接成立；非零时，再将该向量绕 Base +Z 按右手规则旋转。例如 +90° 会将原本沿 Base +X 的目标变成 Base +Y。程序将同一旋转矩阵用于平移和相对转腕，避免两者方向不一致。

映射必须每个轴只用一次，并构成旋转矩阵；镜像映射会被拒绝。**示例不是所有工作站的通用方向**。先通过 `monitor` 观察自己手柄的轴变化，结合机械臂 Base 轴定义填写设置，然后用 `preview` 分别检查右移、前移、上抬和转腕。需要修改时，退出当前遥操，再运行 `direction`；运行中的进程不会热加载配置。

松 Grip 再握住不会改变方向矩阵。头显应用重启、重新居中或站位改变后，仍须重新预览核对，必要时重新设置方向。本精简版未携带原工作站的相机外参或历史方向标定，也不需要 D405。

预览中的方向确认仅证明计算结果；首次真实小范围动作仍需操作者直接观察机械臂的实际方向。

## 4. 启动真机遥操

手动方向已保存、预览正确、机械臂静止、整臂周围有足够净空，停止其他控制程序后，由操作者执行：

```bash
bash robot.sh run --seconds 60
```

**此命令直接开启真机模式，无需再输入 START。** 首次有效帧到来时若 Grip 已握住，会以当前位姿建立零位移锚点；建议启动时先松开 Grip，再小幅接合。

| 操作 | 行为 |
| --- | --- |
| 握住右 Grip 并移动手柄 | 相对平移跟随 |
| 握住右 Grip 并转腕 | 相对姿态跟随，默认转腕比例 0.5 |
| 松开右 Grip | 清除目标并请求停止，核对反馈 |
| 保持 Grip，按下 Trigger | 力控闭合夹爪 |
| 保持 Grip，松开 Trigger | 张开夹爪；独立于机械臂规划暂停 |
| 松 Grip 后重新握住 | 以当前机器人/手柄位姿重建相对锚点 |
| Ctrl+C | 停止遥操、核对停止反馈并保存报告 |

启动或重新握 Grip 不会自动松爪。夹爪需要健康反馈，终端显示 `gripper=ready` 后才可控制；不可用时仅保留机械臂跟随。收到夹爪 ACK 不代表物体已夹牢，需观察真实夹爪与物体。

首次验收先做空载短距离动作，再检查夹爪开闭，最后做短距离夹持抬升。每段先控制在 2–5 cm，松 Grip 停稳后再接合下一段。操作者应能够操作实体急停；报告 `stop_unverified` 时使用实体急停并检查控制器。

## 关键参数与保护

| 参数 | 模板默认值 |
| --- | --- |
| `axis_map` / `axis_yaw_deg` | 由使用者手动设置并保存，支持轴符号/排列与偏航修正 |
| `direction_configured` | 模板为 false，手动设置后为 true；只表示已保存，不代表硬件方向已验收 |
| `position_scale` | 1.0 |
| `rotation_scale` | 0.5；持续相对转腕，跨 180° 保持连续 |
| `control_hz` | 50 Hz，控制器规划的 `movej_follow` |
| `workspace_radius_mm` | 启动 TCP 周围 400 mm；重新握 Grip 不扩展范围 |
| `tcp_speed_mm_s` | 100 mm/s 上限 |
| `rotation_speed_deg_s` | 60°/s 上限 |
| `joint_speed_deg_s` / `joint_accel_deg_s2` | 45°/s / 120°/s² |
| `input_timeout_s` | 100 ms |
| `feedback_timeout_s` | 200 ms，位于 `robot.json` |
| `max_duration_s` | 3600 s，可用 `--seconds` 缩短 |

保留输入时效、重复帧/时钟回退、位姿跳变、丢追踪与失焦检查；故障后需要松开并重新握 Grip，不能保持握住自动恢复。规划前检查关节限位、自碰撞、奇异性和刹车轨迹；持续反馈异常、工具配置改变、控制器错误均触发退出与停止验证。运动指令不会在应答不明时重试。

工作空间球体不能代替环境碰撞检测；模型自碰撞检查不包含桌面、障碍物或被抓物体。50 Hz 是软件目标频率，普通 Linux/Python 不提供硬实时保证。

## 日志与排错

每次预览或遥操生成 `run/follow_时间/`：

- `events.jsonl`：输入、目标、真实反馈、关节发送、夹爪与停止事件。
- `report.json`：运行状态、命令计数、停止结果、代码哈希与 SDK 清理结果。
- `run/service.log`：自动启动的 PC Service 日志。

| 现象 | 处理 |
| --- | --- |
| ADB 未授权 | 佩戴头显确认 USB 调试，检查数据线与 `adb devices -l` |
| 无有效帧 | 确认适配客户端、佩戴/焦点、`127.0.0.1`、Controller/Send；重新插 USB 后重跑入口 |
| 机器人连接超时 | 检查网卡静态地址、子网、IP、8080 端口和其他控制程序 |
| 工具/负载/固件检查失败 | 对照支持环境核对真实参数；不要删除校验绕过 |
| SDK 缺失或动态库错误 | 重跑 `bash setup.sh`；检查 PC Service DEB 是否完整 |
| 另一个进程占用 | 退出旧 monitor/preview/run；不要删除锁来强行并行控制 |
| `gripper=unavailable` | 检查夹爪供电、连线、使能与错误状态 |
| `stop_unverified` | 使用实体急停并检查机械臂，不把命令发送成功当作停止成功 |

同一台机械臂只运行一个控制入口。项目目录内的文件锁不代替多个电脑、多个项目副本之间的控制权协调。

## 代码结构

```text
robot.sh / teleop.py        统一入口、设备选择、服务生命周期
setup.sh                   Python 环境与运行依赖安装
install_runtime.py         PC Service 下载、校验与解压
start_service.sh           启动项目内 PC Service
xrobotoolkit_input.py      原生 SDK 回调、追踪和时效检查
realman_follow.py          相对 6D 映射、IK、规划、跟随与停止流程
robot_core.py              机器人配置、反馈解析、互锁、预检与停止验证
follow_transport.py        持久、有序的机器人查询/跟随/停止连接
gripper_transport.py       独立有序的夹爪连接
follow_recording.py        有界异步事件日志与原子报告写入
config/*.example.json      可公开的配置模板
vendor/                    依赖校验清单；APK 由发布包附带或安装时下载
client/                    Quest 源码补丁、构建入口与版本依据
licenses/                  第三方许可
```

已移除测试程序、诊断数据、历史备份、旧网页/UDP/步进遥操入口、桌面 UI、D405 采集、数据集导出与 π₀.₅ 训练/推理模块。保留的日志写入是运行与停止诊断所需代码。

## 许可与来源

保留项目原有 [MIT License](LICENSE)。Quest 适配客户端基于 [XRoboToolkit Unity Client](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client-Quest)，源码提交、补丁及重新构建方法见 [client/README.md](client/README.md)。PC Service 使用 [XRoboToolkit PC Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)。第三方归属见 [THIRD_PARTY.md](THIRD_PARTY.md)，对应许可位于 `licenses/`。
