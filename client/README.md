# Quest 适配客户端

使用者直接通过 `bash robot.sh install-quest` 安装仓库随附的 APK，无需 Unity。此目录供修改客户端的开发者复现构建。

- 上游：<https://github.com/XR-Robotics/XRoboToolkit-Unity-Client-Quest>
- 固定提交：`07a28333341629008187e988b73b48bced316092`
- Unity：`2021.3.45f2`，Android Build Support / SDK / NDK / OpenJDK
- 适配版本：`1.0.1-realman.2`，包名 `com.xrobotoolkit.client.quest.realman`
- 协议：`realman-quest-v1`
- 许可：`../licenses/XRoboToolkit_Quest_MIT.txt`

`native_client_tracking.patch` 增加逐手柄追踪、焦点/佩戴状态及协议字段，并使用物理控制器的位姿；`native_client_compat.patch` 修复与上游 OVR 手部骨架编译选项的兼容问题。缺少有效追踪字段的客户端只能用于诊断，不能授权机械臂运动。

```bash
# 在仓库根目录执行，填写实际 Unity 编辑器路径：
REALMAN_UNITY_EDITOR=/path/to/2021.3.45f2/Editor/Unity bash client/build.sh
```

脚本克隆上游到 `client/source/`，固定提交、应用补丁并调用 Unity，输出到 `client/build/quest-realman.apk`。Unity 需已完成许可激活并安装 Android 工具链；构建日志为 `client/build/unity.log`。不同 Unity 安装环境的系统库问题需要按本机环境处理。

自行替换运行 APK 时，将新文件复制到 `vendor/quest-realman.apk`，同步更新 `vendor/manifest.json` 中的 SHA256，再运行安装命令。构建产物的哈希可能因本地工具链和签名不同而与随附 APK 不同。
