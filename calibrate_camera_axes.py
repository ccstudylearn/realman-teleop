"""Read-only one-gesture direction calibration; no motion/stop/gripper writes."""
import argparse
import asyncio
from dataclasses import replace
from datetime import datetime, timezone
import fcntl
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np

from direction_calibration import RightGesture, AxisGestures, camera_axis_matrix, camera_target_basis
from follow_transport import FollowRPC
from realman_follow import FollowConfig
from robot_core import ROOT, Config, preflight
from xrobotoolkit_input import NativeInput
from device_config import resolve_quest_serial

DEFAULT_MOUNT = ROOT/'config/camera_mount.json'


def quest_pid(serial):
    p = subprocess.run(['adb', '-s', serial, 'shell', 'pidof',
                        'com.xrobotoolkit.client.quest.realman'], capture_output=True, text=True)
    if p.returncode or not p.stdout.strip():
        raise RuntimeError('XRoboToolkit RealMan is not running in the headset')
    return int(p.stdout.split()[0])


async def calibrate(config_path, output, seconds, serial, mount_path, three_axes=False,
                    third_direction='forward', robot_config_path=None):
    config_path = config_path.resolve()
    config = FollowConfig.load(config_path)
    robot = Config.load(robot_config_path or ROOT/'config/robot.json')
    output.mkdir(parents=True, exist_ok=False)
    raw_mount = mount_path.read_bytes()
    mount = json.loads(raw_mount)
    if ('x right, y down, z forward' not in mount.get('camera_frame', '')
            or 'flange' not in mount.get('robot_frame', '')):
        raise ValueError('Unexpected camera/flange coordinate convention')
    rotation = np.array(mount['T_flange_color_optical'])[:3, :3].tolist()
    # Current supported tool is a pure +145mm flange-Z translation. Its
    # rotation is identity, so the flange-camera rotation also maps tool-camera.
    lock = (ROOT/f'.realman-{robot.robot_ip}-{robot.robot_port}.lock').open('a+')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    rpc, source = FollowRPC(robot), NativeInput()
    gesture = AxisGestures(third_direction) if three_axes else RightGesture()
    started = time.monotonic()
    pid = quest_pid(serial)
    try:
        prepared = await preflight(rpc)
        await rpc.close()
        source.start()
        print('只读方向记录：机械臂不会运动。', flush=True)
        print('先松开右 Grip，再握住 Grip，向你认定的右边平移约10厘米；到右端松开Grip，先不要把手移回。', flush=True)
        if three_axes:
            print('随后松Grip回位，握Grip竖直抬高10厘米并在高处松开；再松Grip回位，握Grip'+
                  gesture.labels[2]+'10厘米并在终点松开。', flush=True)
        last_status, last_heartbeat = None, -float('inf')
        while time.monotonic()-started < seconds and gesture.result is None:
            source.drain(gesture.observe, gesture.trip)
            if gesture.error:
                raise RuntimeError(gesture.error)
            status = ('正在记录向右动作' if gesture.samples else
                      '已就绪：握Grip并向右移动' if gesture.ready else '等待有效追踪并松开Grip')
            if three_axes:
                status = f'第{min(len(gesture.groups)+1, 3)}/3步：{gesture.stage}；'+(
                    '正在记录' if gesture.samples else '握Grip开始' if gesture.ready else '等待有效追踪并松开Grip')
            if status != last_status:
                print(status, flush=True); last_status = status
            if time.monotonic()-last_heartbeat > 5:
                print(f'收到 {source.frames} 帧，有效追踪 {source.tracking_frames} 帧；'+status, flush=True)
                live = dict(native_frames=source.frames, valid_tracking_frames=source.tracking_frames,
                            last_hand=source.last, last_error=source.last_error,
                            stage=getattr(gesture, 'stage', 'right'),
                            completed_gestures=getattr(gesture, 'groups', None))
                (output/'live_state.json').write_text(json.dumps(live, ensure_ascii=False, indent=2))
                if source.last:
                    print('追踪状态：'+str({k: source.last.get(k) for k in
                        ('valid', 'grip', 'device_valid', 'is_tracked', 'tracking_state',
                         'focus', 'user_present', 'controller_device', 'device_name')}), flush=True)
                last_heartbeat = time.monotonic()
            await asyncio.sleep(.005)
        if gesture.result is None:
            raise RuntimeError('Direction capture timed out; existing config was not changed')
        if quest_pid(serial) != pid:
            raise RuntimeError('Headset application restarted during direction capture')
        result = dict(schema='camera-screen-directions-v1', camera_serial=config.camera_serial,
                      camera_rotation_tool=rotation, vr_right_axis=gesture.result,
                      quest_app_pid=pid, quest_serial=serial,
                      mount_source=str(mount_path), mount_sha256=hashlib.sha256(raw_mount).hexdigest(),
                      mount_use='rotation-only directional reference; not absolute position certification',
                      created_utc=datetime.now(timezone.utc).isoformat())
        result['target_basis_base'] = camera_target_basis(
            prepared['anchor']['tcp_pose_m_rad'][3:], result).tolist()
        result['reference_tcp_pose_m_rad'] = prepared['anchor']['tcp_pose_m_rad']
        result['translation_reference'] = 'fixed-at-calibration-camera-right-table-forward-base-up'
        replace(config, direction_calibration=result, direction_configured=True).validate()
        matrix = camera_axis_matrix(prepared['anchor']['tcp_pose_m_rad'][3:], result)
        (output/'capture.json').write_text(json.dumps(dict(
            calibration=result, samples=gesture.samples, matrix_at_capture=matrix.tolist(),
            motion_commands_sent=0, stop_commands_sent=0, gripper_commands_sent=0,
            direction_gestures=getattr(gesture, 'groups', None),
            angular_errors_deg=getattr(gesture, 'angular_errors_deg', None),
            third_direction=third_direction if three_axes else None,
            physical_direction_verification_pending=True), ensure_ascii=False, indent=2))
        targets = [config_path]
        suffix = datetime.now().strftime('%Y%m%d_%H%M%S')
        for target in targets:
            data = json.loads(target.read_text())
            data['direction_calibration'] = result
            data['direction_configured'] = True
            FollowConfig(**data).validate()
            backup = target.with_name(target.name+'.before-direction-'+suffix)
            backup.write_bytes(target.read_bytes())
            temporary = target.with_suffix('.tmp')
            temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
            temporary.replace(target)
        print('方向已保存：右/左沿校准时D405画面右/左，前/后沿桌面，上/下为离开/接近桌面。', flush=True)
        print('重新握Grip只重置起点，方向不随手腕转动重算；速度设置未改变。', flush=True)
        print('移动停止方向仍须用一次小幅真机动作核对。记录：'+str(output), flush=True)
    finally:
        source.close()
        await rpc.close()
        lock.close()
        (output/'input_summary.json').write_text(json.dumps(dict(
            native_frames=source.frames, valid_tracking_frames=source.tracking_frames,
            last_error=source.last_error, capture_error=gesture.error,
            last_hand=source.last,
            direction_gestures=getattr(gesture, 'groups', None),
            incomplete_gesture=gesture.samples, completed=gesture.result is not None,
            third_direction=third_direction if three_axes else None,
            motion_commands_sent=0), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/follow.json')
    parser.add_argument('--robot-config', type=Path, default=ROOT/'config/robot.json')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--seconds', type=float, default=180.)
    parser.add_argument('--serial', help='默认使用设备配置或唯一已授权 Quest')
    parser.add_argument('--mount', type=Path, default=DEFAULT_MOUNT)
    parser.add_argument('--three-axes', action='store_true', help='连续记录右、上、前三方向并核对串轴')
    parser.add_argument('--third-direction', choices=('forward', 'backward'), default='forward')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 600:
        parser.error('seconds must be within [1, 600]')
    output = args.output or ROOT/'run'/('direction_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    try:
        serial = resolve_quest_serial(args.serial)
        asyncio.run(calibrate(args.config, output, args.seconds, serial, args.mount,
                              args.three_axes, args.third_direction, args.robot_config))
    except (OSError, RuntimeError, ValueError) as error:
        print(str(error), flush=True)
        raise SystemExit(1)
