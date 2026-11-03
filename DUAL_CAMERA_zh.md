# 双 D405 数据格式与同步边界

角色固定为 `wrist`（近景）和 `external`（外部）。使用者通过两个不同的序列号手动绑定角色，不使用任何发布者的设备编号或历史外参。

两台设备各运行 640×480@30 FPS RGB-D 流，默认约 10 FPS 保存 RGB PNG 和 uint16 原始深度。颜色与深度的逐帧时间、两台设备各自的内参/深度尺度及流间外参保留；原始深度未对齐 RGB。本版不提供相机之间或手眼外参标定结果。

## 目录

默认数据根目录为项目下 `data/`，可通过 configure 的 `--data-dir` 更改：

```text
data/
  recordings/follow_日期时间/
    session.json
    events.jsonl
    report.json
    camera/worker.log
    camera_external/worker.log
    episodes/episode_000001/
      episode.json
      events.jsonl
      samples.jsonl
      camera/                  # wrist
        rgb/ depth_raw/
        calibration.json
        frames.jsonl
        report.json
      camera_external/         # external，同样的结构
  diagnostics/dual_camera_probe_日期时间/
    camera_probe_report.json
```

相机原始帧在独立进程中获取和保存，PNG 编码不在机械臂运动事件循环中执行。A/B 让两路相机切换同一个 episode 和边界；每路分别确认边界，不能将只结束一台的条目视为完整。

## 对齐

1. 每台相机内先检查 RGB/深度的时钟域一致和时间差（最大 20 ms）；孤立不匹配帧会记录为拒绝，不刷新有效帧心跳。
2. 两路相机按主机单调时钟的接收时间做一对一最近邻配对，默认最大差值 `camera_max_skew_s=0.06` 秒。
3. 每路图像再与机械臂查询中点比较，最大距离 0.1 秒，机械臂查询耗时最大 0.15 秒。

两台设备的独立时钟不直接相互比较。外部帧不复用来填补近景缺帧；缺失或超差样本保留为无效，不伪造同步图像。

`samples.jsonl` 的 `frames.wrist`、`frames.external` 保存逐路源帧，`camera_alignment_valid`、`camera_alignment_skew_s` 和 `camera_robot_alignment_skew_s` 记录对齐判定。

`episode.json` 保存逐路帧数、配对/未配对计数、最大差值、完整性及操作者成功/失败标签。完整录制、配对有效、任务成功分别记录，不互相替代。

**这是主机接收时间的软件对齐，不是曝光时刻硬件同步。**

## 故障和磁盘

任一路相机退出，或有效帧心跳停滞约 1 秒，都会触发当前停止流程。已经保存的原始资料和操作者标签仍保留，录制完整性单独标记；相机错误不会自动变成成功示范。

录制目录需可写、至少有 1 GiB 空闲；持续 RGB-D 保存应预留更多空间。可选的 `mount_point` 未实际挂载时会拒绝录制。不会静默改写到其他目录，也不会覆盖已有 episode。

相机与其他软件不能同时占用同一流。两路不稳定时先检查 USB 线、独立 USB 控制器的带宽、供电与磁盘写入速度，再做 `camera-check`；不能通过放宽同步阈值来宣称数据已同步。
