"""Install a pinned, hash-checked upstream PC Service without root privileges."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parent


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def verify(path, expected):
    if sha256(path) != expected:
        raise RuntimeError(f'SHA256 校验失败：{path}；请重新下载正确文件。')


def download_verified(info, destination):
    partial = destination.with_suffix('.part')
    try:
        request = urllib.request.Request(info['url'], headers={'User-Agent': 'RealMan-Quest-Teleop/1.0'})
        with urllib.request.urlopen(request, timeout=60) as response, partial.open('wb') as stream:
            shutil.copyfileobj(response, stream)
        verify(partial, info['sha256'])
        partial.replace(destination)
    finally:
        partial.unlink(missing_ok=True)


def main():
    manifest = json.loads((ROOT/'vendor/manifest.json').read_text())
    apk = ROOT/'vendor/quest-realman.apk'
    if not apk.is_file():
        print('从本仓库 Release 下载适配 Quest APK…', flush=True)
        download_verified(manifest['quest_apk'], apk)
    verify(apk, manifest['quest_apk']['sha256'])
    info = manifest['pc_service']
    native = ROOT/'.native'
    native.mkdir(exist_ok=True)
    runtime = native/'pc-service'
    marker = runtime/'installed.sha256'
    sdk = runtime/'opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so'
    if marker.is_file() and marker.read_text().strip() == info['sha256'] and sdk.is_file():
        print('PC Service 已安装，复用锁定版本。')
        return
    bundled = ROOT/'vendor'/info['filename']
    cached = native/info['filename']
    archive = bundled if bundled.is_file() else cached
    if not archive.is_file():
        print('下载 PC Service v1.0.0（约 110 MiB，可在 README 中找到手动下载地址）…', flush=True)
        download_verified(info, cached)
        archive = cached
    verify(archive, info['sha256'])
    if runtime.exists():
        raise RuntimeError('已有不同或不完整的 .native/pc-service；请将该目录改名备份后重试。')
    with tempfile.TemporaryDirectory(prefix='extract-', dir=native) as temporary:
        stage = Path(temporary)/'pc-service'
        subprocess.run(['dpkg-deb', '--extract', str(archive), str(stage)], check=True)
        if not (stage/'opt/apps/roboticsservice/RoboticsServiceProcess').is_file():
            raise RuntimeError('PC Service 安装包目录结构不符合预期。')
        (stage/'installed.sha256').write_text(info['sha256']+'\n')
        stage.rename(runtime)
    print('PC Service 与原生 SDK 已解压安装；未启动任何设备。')


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        raise SystemExit(str(error))
