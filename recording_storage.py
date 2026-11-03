"""Portable recording location, free-space and optional mount checks."""
from datetime import datetime
import json
import os
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parent


def storage_config():
    path = ROOT/'config/storage.json'
    if not path.is_file():
        path = ROOT/'config/storage.example.json'
    config = json.loads(path.read_text())
    if set(config) - {'data_dir', 'mount_point'}:
        raise ValueError('Unknown storage configuration fields')
    if not isinstance(config.get('data_dir'), str) or not config['data_dir'].strip():
        raise ValueError('data_dir must be a nonempty path')
    mount = config.get('mount_point')
    if mount is not None and (not isinstance(mount, str) or not mount.strip()):
        raise ValueError('mount_point must be null or a nonempty path')
    return config


def resolve_path(value):
    path = Path(value).expanduser()
    return (path if path.is_absolute() else ROOT/path).resolve()


def data_root():
    return resolve_path(storage_config()['data_dir'])


def check_recording_output(output):
    config = storage_config()
    path = Path(output).expanduser().resolve()
    if config.get('mount_point'):
        mount = resolve_path(config['mount_point'])
        if not os.path.ismount(mount):
            raise RuntimeError(f'配置的数据盘未挂载，拒绝录制：{mount}')
        if not path.is_relative_to(mount):
            raise RuntimeError('录制目录必须位于配置的 mount_point 内')
    existing = path
    while not existing.exists():
        existing = existing.parent
    if not existing.is_dir() or not os.access(existing, os.W_OK):
        raise RuntimeError(f'录制目录不可写：{existing}')
    if shutil.disk_usage(existing).free < 1024**3:
        raise RuntimeError('录制盘可用空间不足 1 GiB，拒绝录制')


def default_recording_output():
    output = data_root()/'recordings'/datetime.now().strftime('follow_%Y%m%dT%H%M%S_%f')
    check_recording_output(output)
    return output
