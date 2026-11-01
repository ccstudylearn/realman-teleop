#!/usr/bin/env bash
set -euo pipefail
teleop_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -x "$teleop_root/.venv/bin/python" ]]; then
  echo '请先在项目目录运行 bash setup.sh' >&2
  exit 1
fi
exec env -u PYTHONPATH -u LD_LIBRARY_PATH PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  "$teleop_root/.venv/bin/python" -u "$teleop_root/teleop.py" "$@"
