"""Single entry point for configuring, checking and running Quest/RealMan teleop."""
import argparse
import asyncio
import ctypes
from datetime import datetime
import fcntl
import ipaddress
import json
import math
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
PACKAGE = 'com.xrobotoolkit.client.quest.realman'


def write_local(path, data):
    from follow_recording import write_json
    if path.exists():
        backup = path.with_name(path.name+'.before-'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        backup.write_bytes(path.read_bytes())
    write_json(path, data)


def configure(args):
    from realman_follow import FollowConfig
    from robot_core import Config
    ipaddress.ip_address(args.robot_ip)
    robot = json.loads((ROOT/'config/robot.example.json').read_text())
    robot.update(robot_ip=args.robot_ip, robot_port=args.robot_port)
    Config(**robot).validate()
    follow = json.loads((ROOT/'config/follow.example.json').read_text())
    if bool(args.wrist_serial) != bool(args.external_serial):
        raise ValueError('双机位需要同时指定 --wrist-serial 和 --external-serial')
    if args.wrist_serial:
        follow.update(camera_serial=args.wrist_serial, camera_external_serial=args.external_serial)
    if not args.data_dir.strip() or (args.mount_point is not None and not args.mount_point.strip()):
        raise ValueError('保存目录和挂载点不能是空字符串')
    if args.axis_map is not None:
        follow.update(axis_map=args.axis_map, direction_configured=True)
    FollowConfig(**follow).validate()
    write_local(ROOT/'config/robot.json', robot)
    write_local(ROOT/'config/follow.json', follow)
    write_local(ROOT/'config/devices.json', {'quest_serial': args.serial})
    write_local(ROOT/'config/storage.json', {'data_dir': args.data_dir, 'mount_point': args.mount_point})
    print('已生成本机机器人、方向/相机、设备和保存目录配置；未连接机械臂。')
    print('下一步：bash robot.sh direction 手动设置方向，然后用 preview 逐方向核对。')
    print('再次 configure 会重置跟随参数，并备份旧配置。')


def cameras(_args):
    """Enumerate devices without starting capture or robot control."""
    import pyrealsense2 as rs
    devices = [{'name': d.get_info(rs.camera_info.name), 'serial': d.get_info(rs.camera_info.serial_number)}
               for d in rs.context().query_devices()]
    print(json.dumps({'cameras': devices, 'robot_motion_enabled': False}, ensure_ascii=False, indent=2))
    if not devices:
        raise RuntimeError('未找到 RealSense 相机，请检查 USB 连接和设备访问权限')


def load_config():
    from realman_follow import FollowConfig
    from robot_core import Config
    if not (ROOT/'config/robot.json').is_file() or not (ROOT/'config/follow.json').is_file():
        raise RuntimeError('请先运行 bash robot.sh configure --robot-ip 机械臂IP')
    return Config.load(ROOT/'config/robot.json'), FollowConfig.load(ROOT/'config/follow.json')


def direction(args):
    """Edit only local direction settings; never start ADB, SDK or robot RPC."""
    from dataclasses import asdict, replace
    from realman_follow import axis_matrix
    _, current = load_config()
    if args.show:
        mapping, yaw = current.axis_map, current.axis_yaw_deg
        configured = current.direction_configured
    else:
        mapping, yaw = args.axis_map, args.yaw_deg
        if mapping is None and yaw is None:
            print('手动设置方向：不连接机械臂，不发送运动或夹爪命令。')
            print('依次填写 Base X、Y、Z 对应的 Quest 轴，用逗号分隔。')
            print('可用 x/y/z 或 -x/-y/-z；例如 -x,z,y。方向须根据自己的站位核对。')
            mapping = input('轴映射（必填）：').strip()
            yaw_text = input('绕 Base +Z 的偏航修正角度，单位度（回车为 0）：').strip()
            yaw = float(yaw_text or '0')
        else:
            mapping = mapping if mapping is not None else current.axis_map
            yaw = yaw if yaw is not None else current.axis_yaw_deg
        updated = replace(current, axis_map=mapping, axis_yaw_deg=yaw,
                          direction_configured=True).validate()
        write_local(ROOT/'config/follow.json', asdict(updated))
        configured = True
    matrix = axis_matrix(mapping, yaw)
    print(json.dumps(dict(direction_configured=configured, axis_map=mapping,
                          axis_yaw_deg=yaw, quest_to_base_rotation=matrix.tolist(),
                          robot_motion_enabled=False), ensure_ascii=False, indent=2))
    if not args.show:
        print('方向已保存，其他速度和夹爪参数保留。下一步：bash robot.sh preview --seconds 60')


def check(_args):
    from device_config import resolve_quest_serial
    from follow_transport import FollowRPC
    from realman_follow import RealManModel
    from robot_core import preflight
    from xrobotoolkit_input import SDK
    robot, _ = load_config()
    ctypes.CDLL(str(SDK))
    print('SDK 已加载；Quest USB：'+resolve_quest_serial(), flush=True)
    async def inspect():
        rpc = FollowRPC(robot, execute=False)
        try:
            result = await preflight(rpc)
            model = RealManModel(*result['bounds'])
            import numpy as np
            from scipy.spatial.transform import Rotation
            measured = result['anchor']['tcp_pose_m_rad']
            pose = model.fk(result['anchor']['joint_deg'])
            position_error = float(np.linalg.norm(pose[:3]-measured[:3])*1000)
            angle_error = math.degrees((Rotation.from_euler('xyz',pose[3:]) *
                                       Rotation.from_euler('xyz',measured[3:]).inv()).magnitude())
            if position_error > 5 or angle_error > 2:
                raise RuntimeError(f'模型与真实 TCP 不匹配：{position_error:.2f} mm / {angle_error:.2f} deg')
            model.check(result['anchor']['joint_deg'])
            return dict(robot=result['anchor'], software=result['software'],
                        fk_error_mm=position_error, fk_angle_error_deg=angle_error,
                        robot_motion_commands_sent=rpc.motion_attempts,
                        stop_commands_sent=rpc.stop_attempts)
        finally:
            await rpc.close()
    print(json.dumps(asyncio.run(inspect()), ensure_ascii=False, indent=2))


def listening():
    try:
        with socket.create_connection(('127.0.0.1',60061), timeout=.2):
            return True
    except OSError:
        return False


def check_idle(robot=None):
    paths = [ROOT/'.xrobotoolkit-input.lock']
    if robot:
        paths.append(ROOT/f'.realman-{robot.robot_ip}-{robot.robot_port}.lock')
    for path in paths:
        with path.open('a+') as owner:
            try:
                fcntl.flock(owner, fcntl.LOCK_SH | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise RuntimeError('已有监视或遥操进程运行，请先退出。') from error


def wait_child(arguments):
    child = subprocess.Popen(arguments, cwd=ROOT)
    signals = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    previous = {sig: signal.getsignal(sig) for sig in signals}
    def forward(sig, _frame):
        if child.poll() is None:
            try:
                child.send_signal(sig)
            except ProcessLookupError:
                pass
    try:
        for sig in signals:
            signal.signal(sig, forward)
        return child.wait()
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def install_quest(args):
    from device_config import resolve_quest_serial
    from install_runtime import verify
    manifest = json.loads((ROOT/'vendor/manifest.json').read_text())
    apk = ROOT/'vendor/quest-realman.apk'
    verify(apk, manifest['quest_apk']['sha256'])
    serial = resolve_quest_serial(args.serial)
    subprocess.run(['adb','-s',serial,'install','-r',str(apk)],check=True)
    print('适配客户端已安装；原版 XRoboToolkit 应用保留。')


def launch(args):
    from device_config import resolve_quest_serial
    robot = None
    if args.command != 'monitor':
        robot, config = load_config()
        if args.command in ('run', 'record') and not config.direction_configured:
            raise RuntimeError('尚未手动设置方向：先运行 bash robot.sh direction，再用 preview 核对。')
        if args.seconds is not None and args.seconds > config.max_duration_s:
            raise RuntimeError('seconds 超过 config/follow.json 的 max_duration_s。')
        if args.command == 'record':
            if not args.task.strip():
                raise RuntimeError('record 需要 --task 描述示范任务')
            if config.camera_serial == 'UNCONFIGURED_WRIST' or not config.camera_external_serial:
                raise RuntimeError('请先 configure 时通过 --wrist-serial 和 --external-serial 指定两个不同的相机序列号')
    check_idle(robot)
    serial = resolve_quest_serial(args.serial)
    installed = subprocess.run(['adb','-s',serial,'shell','pm','path',PACKAGE],
                               capture_output=True,text=True,check=False)
    if installed.returncode or 'package:' not in installed.stdout:
        raise RuntimeError('请先运行 bash robot.sh install-quest 安装带追踪有效性字段的客户端。')
    service = log = None
    try:
        if not listening():
            (ROOT/'run').mkdir(exist_ok=True)
            log = (ROOT/'run/service.log').open('a')
            service = subprocess.Popen(['bash',str(ROOT/'start_service.sh')],stdout=log,stderr=subprocess.STDOUT)
            for _ in range(50):
                if service.poll() is not None:
                    raise RuntimeError('PC Service 启动失败，查看 run/service.log。')
                if listening():
                    break
                time.sleep(.1)
            else:
                raise RuntimeError('PC Service 未就绪，查看 run/service.log。')
        subprocess.run(['adb','-s',serial,'reverse','tcp:63901','tcp:63901'],check=True)
        subprocess.run(['adb','-s',serial,'shell','am','start','-n',
                        PACKAGE+'/com.unity3d.player.UnityPlayerActivity'],check=True)
        print('佩戴头显并保持应用在前台。客户端连接地址 127.0.0.1；Controller + Send 开启。',flush=True)
        if args.command == 'monitor':
            entry = 'xrobotoolkit_input.py'
            flags = ['--seconds',str(args.seconds)]
        else:
            entry = 'realman_follow.py'
            flags = ['--direct'] if args.command in ('run', 'record') else []
            if args.command == 'record':
                flags += ['--record', '--task', args.task]
            if args.command in ('run', 'record'):
                print('真机模式：握右 Grip 可立即运动，松 Grip 停止。',flush=True)
            if args.seconds is not None:
                flags += ['--seconds',str(args.seconds)]
        if getattr(args, 'output', None) is not None:
            flags += ['--output', str(args.output)]
        return wait_child([sys.executable,'-u',str(ROOT/entry),*flags])
    finally:
        if service is not None:
            service.terminate()
            try:
                service.wait(timeout=4)
            except subprocess.TimeoutExpired:
                service.kill()
                service.wait()
        if log is not None:
            log.close()


def positive_seconds(value):
    number = float(value)
    if not math.isfinite(number) or not 0 < number <= 7200:
        raise argparse.ArgumentTypeError('seconds 必须是 (0,7200] 内的有限数值。')
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command',required=True)
    conf = sub.add_parser('configure',help='生成本机配置，不连接机器人')
    conf.add_argument('--robot-ip',required=True)
    conf.add_argument('--robot-port',type=int,default=8080)
    conf.add_argument('--axis-map',help='可选：配置时同时明确设置方向，如 --axis-map=-x,z,y')
    conf.add_argument('--serial',help='可选：指定 USB Quest 序列号')
    conf.add_argument('--wrist-serial', help='近景 D405 序列号，录制时必需')
    conf.add_argument('--external-serial', help='外部 D405 序列号，必须与近景不同')
    conf.add_argument('--data-dir', default='data', help='保存根目录；相对路径基于项目目录')
    conf.add_argument('--mount-point', help='可选：要求数据目录位于此真实挂载点，未挂载则拒绝录制')
    sub.add_parser('cameras', help='枚举 RealSense 设备及序列号，不启动采集或机械臂')
    probe = sub.add_parser('camera-check', help='仅检查双相机分段落盘，不连接机械臂或 Quest')
    probe.add_argument('--seconds', type=positive_seconds, default=5.)
    probe.add_argument('--episodes', type=int, default=2)
    probe.add_argument('--output', type=Path)
    directions = sub.add_parser('direction', help='手动设置方向，不连接机器人；无参数时交互输入')
    directions.add_argument('--axis-map', help='Base XYZ 对应的 Quest 轴，如 --axis-map=-x,z,y')
    directions.add_argument('--yaw-deg', type=float, help='轴映射后绕 Base +Z 旋转的角度，范围 [-180,180]')
    directions.add_argument('--show', action='store_true', help='只显示当前设置')
    sub.add_parser('check',help='只读检查硬件配置与模型，不发运动/停止/夹爪命令')
    install = sub.add_parser('install-quest',help='通过 USB 安装随附的适配 APK')
    install.add_argument('--serial')
    for command,help_text in [('monitor','仅检查手柄，不连接机器人'),
                              ('preview','读取机器人并计算目标，不发运动或夹爪命令'),
                              ('run','真机六自由度跟随与夹爪控制'),
                              ('record','真机遥操 + 双 D405 RGB-D + A/B 分段采集')]:
        mode = sub.add_parser(command,help=help_text)
        mode.add_argument('--serial')
        mode.add_argument('--seconds',type=positive_seconds,
                          default=30. if command=='monitor' else 60. if command=='preview' else None)
        if command == 'record':
            mode.add_argument('--task', required=True)
            mode.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.command == 'cameras':
        return cameras(args)
    if args.command == 'camera-check':
        if not 2 <= args.seconds <= 60 or not 1 <= args.episodes <= 10:
            parser.error('camera-check 要求每段 2–60 秒，1–10 条检查段')
        _, config = load_config()
        if config.camera_serial == 'UNCONFIGURED_WRIST' or not config.camera_external_serial:
            parser.error('请先配置两个不同的 D405 序列号')
        from recording_storage import data_root, check_recording_output
        from check_dual_camera_recording import check as camera_check
        output = args.output or data_root()/'diagnostics'/datetime.now().strftime('dual_camera_probe_%Y%m%dT%H%M%S_%f')
        check_recording_output(output)
        asyncio.run(camera_check(config, output, args.seconds, args.episodes))
        return 0
    if args.command == 'direction':
        if args.show and (args.axis_map is not None or args.yaw_deg is not None):
            parser.error('--show 不能与方向修改参数同时使用')
        return direction(args)
    if args.command == 'configure':
        return configure(args)
    if args.command == 'check':
        return check(args)
    if args.command == 'install-quest':
        return install_quest(args)
    return launch(args)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (Exception, KeyboardInterrupt) as error:
        print(str(error),file=sys.stderr)
        raise SystemExit(1)
