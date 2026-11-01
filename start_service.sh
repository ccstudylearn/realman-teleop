#!/usr/bin/env bash
set -euo pipefail
teleop_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
service_dir="$teleop_root/.native/pc-service/opt/apps/roboticsservice"
test -x "$service_dir/RoboticsServiceProcess" || { echo '请先运行 bash setup.sh'; exit 1; }
cd -- "$service_dir"
exec env -u PYTHONPATH LD_LIBRARY_PATH="$service_dir:$service_dir/lib:$service_dir/SDK/x64" \
  QT_PLUGIN_PATH="$service_dir/plugins" QT_QPA_PLATFORM=offscreen ./RoboticsServiceProcess
