"""Shared camera identities and strict image contracts; no hardware imports."""
import math


def camera_layout(config):
    """Keep the original wrist paths readable for old single-camera recordings."""
    result = {"wrist": {"serial": config["camera_serial"], "directory": "camera"}}
    if config.get("camera_external_serial"):
        result["external"] = {"serial": config["camera_external_serial"], "directory": "camera_external"}
    return result


def episode_layout(meta):
    config = meta.get("follow_config", {})
    if "camera_serial" in config:
        return camera_layout(config)
    return meta.get("camera_streams") or {"wrist": {"directory": "camera"}}


def sample_camera_frames(sample, meta):
    expected = episode_layout(meta)
    frames = sample.get("frames", {"wrist": sample["frame"]})
    if set(frames) != set(expected):
        raise ValueError("Missing or unexpected camera view in sample")
    if len(expected) > 1:
        limit = meta.get("follow_config", {}).get("camera_max_skew_s", .06)
        times = [f["host_received"] for f in frames.values()]
        skew = max(times)-min(times)
        if (sample.get("camera_alignment_valid") is not True
                or not all(math.isfinite(t) for t in times) or skew > limit):
            raise ValueError("Camera views are not time aligned")
    return frames


def camera_sources(sample, meta):
    return {name: {"rgb_source": frame["rgb_path"], "depth_source": frame["depth_path"],
                   "host_received": frame["host_received"], "source_frame_index": frame["index"]}
            for name, frame in sample_camera_frames(sample, meta).items()}


def source_cameras(source):
    return source.get("cameras") or {"wrist": source}


def image_key(name):
    return "observation.images." + name
