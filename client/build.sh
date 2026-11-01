#!/usr/bin/env bash
set -euo pipefail
client_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
client_editor="${REALMAN_UNITY_EDITOR:?请指定 REALMAN_UNITY_EDITOR 为 Unity 编辑器完整路径}"
client_source="$client_root/source"
client_commit=07a28333341629008187e988b73b48bced316092
test -x "$client_editor" || { echo 'Unity 编辑器不存在或不可执行'; exit 1; }
if [[ ! -d "$client_source" ]]; then
  git clone https://github.com/XR-Robotics/XRoboToolkit-Unity-Client-Quest.git "$client_source"
  git -C "$client_source" checkout --detach "$client_commit"
fi
test "$(git -C "$client_source" rev-parse HEAD)" = "$client_commit" || { echo '源码提交不匹配'; exit 1; }
for client_patch in native_client_tracking.patch native_client_compat.patch; do
  if git -C "$client_source" apply --check "$client_root/$client_patch" 2>/dev/null; then
    git -C "$client_source" apply "$client_root/$client_patch"
  elif ! git -C "$client_source" apply --reverse --check "$client_root/$client_patch" 2>/dev/null; then
    echo "补丁与源码不匹配：$client_patch"; exit 1
  fi
done
mkdir -p "$client_source/Assets/Editor" "$client_root/build"
cp -- "$client_root/unity/RealManQuestBuild.cs" "$client_source/Assets/Editor/RealManQuestBuild.cs"
env -u PYTHONPATH -u LD_LIBRARY_PATH REALMAN_QUEST_APK="$client_root/build/quest-realman.apk" \
  "$client_editor" -batchmode -nographics -quit -buildTarget Android \
  -projectPath "$client_source" -executeMethod RealManQuestBuild.Build \
  -logFile "$client_root/build/unity.log"
test -s "$client_root/build/quest-realman.apk"
sha256sum "$client_root/build/quest-realman.apk"
