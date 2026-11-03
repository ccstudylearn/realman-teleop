#!/usr/bin/env bash
set -euo pipefail
teleop_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$teleop_root"
unset PYTHONPATH LD_LIBRARY_PATH
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
teleop_python="${REALMAN_PYTHON:-python3.12}"
command -v "$teleop_python" >/dev/null || { echo '需要 Python 3.12，见 README 的系统依赖。'; exit 1; }
"$teleop_python" - <<'PY'
from pathlib import Path
import platform, sys
os_release = dict(line.split('=',1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
if (platform.machine() != 'x86_64' or sys.version_info[:2] != (3,12)
        or os_release.get('ID','').strip('"') != 'ubuntu'
        or os_release.get('VERSION_ID','').strip('"') != '24.04'):
    raise SystemExit('此运行包支持 Ubuntu 24.04 x86_64 / Python 3.12。')
PY
for teleop_command in adb dpkg-deb; do
  command -v "$teleop_command" >/dev/null || { echo "缺少 $teleop_command，见 README 的系统依赖。"; exit 1; }
done
"$teleop_python" -m venv .venv
if [[ -d vendor/wheels ]]; then
  .venv/bin/python -m pip install --no-index --find-links vendor/wheels -r requirements.txt
else
  .venv/bin/python -m pip install -r requirements.txt
fi
.venv/bin/python install_runtime.py
.venv/bin/python - <<'PY'
import ctypes
from xrobotoolkit_input import SDK
from realman_follow import RealManModel
import numpy, scipy, cv2, pyrealsense2
ctypes.CDLL(str(SDK))
model = RealManModel([-178,-130,-135,-178,-128,-360], [178,130,135,178,128,360])
model.fk([0,-30,90,0,60,0])
print('安装完成：SDK 加载与离线运动学检查通过；未连接或驱动机械臂。')
PY
echo '下一步：bash robot.sh configure --robot-ip 你的机械臂IP，然后 bash robot.sh direction'
