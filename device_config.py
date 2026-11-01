"""Portable, explicit selection of an authorized ADB headset."""
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent


def resolve_quest_serial(requested=None):
    devices = ROOT/"config/devices.json"
    saved = json.loads(devices.read_text()).get("quest_serial") if devices.exists() else None
    serial = requested or os.environ.get("QUEST_SERIAL") or saved
    result = subprocess.run(["adb", "devices"], capture_output=True, text=True, check=True)
    states = {}
    for line in result.stdout.splitlines()[1:]:
        fields = line.split()
        if len(fields) >= 2:
            states[fields[0]] = fields[1]
    if serial:
        if states.get(serial) != "device":
            raise RuntimeError(f"Quest {serial} 状态为 {states.get(serial, '未连接')}；请检查 USB 并允许 USB 调试")
        return serial
    authorized = [s for s, state in states.items() if state == "device"]
    if len(authorized) != 1:
        raise RuntimeError("需要且只允许一个已授权 ADB 设备，或通过 --serial 指定 Quest；请运行 adb devices -l")
    return authorized[0]


if __name__ == "__main__":
    import sys
    print(resolve_quest_serial(sys.argv[1] if len(sys.argv) > 1 else None))
