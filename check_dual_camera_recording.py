"""Camera-only segmented RGB-D recording check; never connects to robot or Quest."""
import argparse
import asyncio
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import time

from camera_layout import camera_layout
from episode_recording import CameraSession, align_episode, load_rows, write_json
from realman_follow import FollowConfig
from recording_storage import data_root, check_recording_output


async def check(config, output, seconds=5., episodes=2):
    import cv2
    import numpy as np

    if not config.camera_external_serial:
        raise ValueError("This check requires two configured cameras")
    output.mkdir(parents=True, exist_ok=False)
    camera = CameraSession(output, config, "双相机落盘检查（非任务示范）", True)
    report = {"camera_only": True, "robot_connected": False, "robot_motion_commands_sent": 0,
              "output": str(output), "episodes": [], "status": "running"}
    try:
        await camera.start()
        for number in range(1, episodes+1):
            eid = f"episode_{number:06d}"
            directory = output/"episodes"/eid
            directory.mkdir(parents=True)
            (directory/"events.jsonl").write_text("")
            started = time.monotonic()
            await camera.set_episode(eid, started)
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                camera.check()
                await asyncio.sleep(.05)
            ended = time.monotonic()
            await camera.set_episode(None, ended)
            meta = {"follow_config": asdict(config), "camera_probe": True,
                    "started_monotonic": started, "ended_monotonic": ended,
                    "task_success": None, "status": "camera_probe"}
            manifest = await asyncio.to_thread(align_episode, directory, "双相机落盘检查（非任务示范）",
                                               (started, ended), meta)
            details = {}
            for name, spec in camera_layout(asdict(config)).items():
                rows = load_rows(directory/spec["directory"]/"frames.jsonl")
                if len(rows) < seconds*config.record_fps*.8:
                    raise RuntimeError(f"Too few saved frames: {name}: {len(rows)}")
                for row in rows:
                    rgb = cv2.imread(str(directory/row["rgb_path"]), cv2.IMREAD_UNCHANGED)
                    depth = cv2.imread(str(directory/row["depth_path"]), cv2.IMREAD_UNCHANGED)
                    if (rgb is None or rgb.shape != (480, 640, 3) or depth is None
                            or depth.shape != (480, 640) or depth.dtype != np.uint16):
                        raise RuntimeError(f"Invalid image on disk: {name}")
                span = rows[-1]["host_received"]-rows[0]["host_received"]
                details[name] = {"serial": spec["serial"], "saved_frames": len(rows),
                                 "measured_saved_fps": (len(rows)-1)/span,
                                 "all_rgb_depth_files_decoded": True}
            ratio = manifest["paired_camera_samples"]/max(1, manifest["images"])
            if not manifest["camera_complete"] or ratio < .9:
                raise RuntimeError(f"Incomplete cameras or insufficient pairing: {ratio:.1%}")
            record = {"episode": eid, "cameras": details, "paired_ratio": ratio,
                      "pair_skew_max_s": manifest["camera_pair_skew_max_s"]}
            report["episodes"].append(record)
            print(json.dumps(record, ensure_ascii=False), flush=True)
        report["status"] = "passed"
    except BaseException as error:
        report.update(status="failed", error=f"{type(error).__name__}: {error}")
        raise
    finally:
        report["camera_shutdown"] = await camera.close()
        if report["camera_shutdown"].get("complete") is not True:
            report.update(status="failed", shutdown_error="Camera shutdown was incomplete")
        write_json(output/"camera_probe_report.json", report)
    if report["status"] != "passed":
        raise RuntimeError(report.get("shutdown_error", "Camera probe failed"))
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return report


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", type=Path, default=Path(__file__).parent/"config/follow.json")
    p.add_argument("--output", type=Path)
    p.add_argument("--seconds", type=float, default=5.)
    p.add_argument("--episodes", type=int, default=2)
    a = p.parse_args()
    if not 2 <= a.seconds <= 60 or not 1 <= a.episodes <= 10:
        p.error("Use 2–60 seconds and 1–10 episodes")
    output = a.output or data_root()/"diagnostics"/datetime.now().strftime("dual_camera_probe_%Y%m%dT%H%M%S_%f")
    check_recording_output(output)
    asyncio.run(check(FollowConfig.load(a.config), output, a.seconds, a.episodes))
