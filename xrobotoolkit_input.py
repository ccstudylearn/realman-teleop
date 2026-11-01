"""Read native XRoboToolkit callbacks. No device-control or robot-write APIs.

Use raw callbacks instead of SDK getters: getters cache old poses independently
of their timestamps. Stock Quest 1.0.1 does not send per-controller pose validity;
its coordinates are displayed but cannot authorize motion.
"""
import argparse
import asyncio
import ctypes as C
import fcntl
import json
import math
from pathlib import Path
import queue
import threading
import time

ROOT = Path(__file__).resolve().parent
SDK = ROOT / ".native/pc-service/opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so"
STATE = 1 << 25
DISCONNECT = (1 << 3) | (1 << 5)


class DeviceState(C.Structure):
    _fields_ = [("devID", C.c_char * 32), ("stateJson", C.c_char * 16352)]


Callback = C.CFUNCTYPE(None, C.c_void_p, C.c_uint, C.c_int, C.c_void_p)


def decode_state_json(raw):
    data = json.loads(raw)
    data = data.get("value", data)
    if isinstance(data, str):
        data = json.loads(data)
    if not isinstance(data, dict):
        raise ValueError("Native state must be an object")
    stamp = data.get("timeStampNs")
    if type(stamp) is not int or stamp <= 0:
        raise ValueError("Native state has no integer frame timestamp")
    return data


def right_hand(data, seq):
    """Output already uses OpenXR right-handed coordinates; never flip Z twice.

    poseValid is the explicit field in the provided Unity source patch. Neither
    a nonzero pose, headset status, nor Input==1 proves right-hand tracking.
    """
    c = data.get("Controller", {}).get("right", {})
    pose = c.get("pose", "0,0,0,0,0,0,1")
    pose = [float(v) for v in pose.split(",")] if isinstance(pose, str) else pose
    if (len(pose) != 7 or not all(type(v) in (int, float) and math.isfinite(v) for v in pose)
            or not .9 <= math.sqrt(sum(v*v for v in pose[3:])) <= 1.1):
        raise ValueError("Invalid native right controller pose")
    buttons = [c.get(k, 0) for k in ("trigger", "grip")]
    if not all(type(v) in (int, float) and math.isfinite(v) and 0 <= v <= 1 for v in buttons):
        raise ValueError("Invalid native buttons")
    explicit = type(c.get("poseValid")) is bool
    # Input is an aggregate mode: left bare-hand visibility can set it to 2
    # while the right physical controller remains tracked. Use per-hand fields.
    valid = (data.get("inputProtocol") == "realman-quest-v1" and explicit and c["poseValid"]
             and c.get("deviceValid") is True and c.get("isTracked") is True
             and c.get("controllerDevice") is True
             and type(c.get("trackingState")) is int and c["trackingState"] & 3 == 3
             and "pose" in c and data.get("appState", {}).get("focus") is True
             and data.get("appState", {}).get("userPresent") is True)
    return {"seq": seq, "valid": bool(valid), "position_m": pose[:3],
            "quaternion_xyzw": pose[3:], "trigger": buttons[0], "grip": buttons[1],
            "tracking_flag_present": explicit, "pose_present": "pose" in c,
            "input_protocol": data.get("inputProtocol"), "input_mode": data.get("Input"),
            "device_valid": c.get("deviceValid"), "is_tracked": c.get("isTracked"),
            "tracking_state": c.get("trackingState"), "focus": data.get("appState", {}).get("focus"),
            "user_present": data.get("appState", {}).get("userPresent"),
            "controller_device": c.get("controllerDevice"), "device_name": c.get("deviceName"),
            "pose_source": c.get("poseSource"), "presence_source": data.get("appState", {}).get("presenceSource")}


class NativeInput:
    """Bounded callback queue. Drain every frame, including button releases."""
    def __init__(self, device=None, grip_threshold=.8):
        if type(grip_threshold) not in (int, float) or not .2 < grip_threshold <= 1.:
            raise ValueError("Invalid Grip threshold")
        self.grip_threshold = grip_threshold
        self.device = device
        self.items = queue.Queue(maxsize=128)
        self.fault = threading.Event()
        self.lib = None
        self.lock = None
        self.callback = Callback(self._callback)  # Keep callback alive until Deinit.
        self.last_stamp = self.offset = self.last_arrival = None
        self.seq = self.frames = self.rejected = self.tracking_frames = 0
        self.last = None
        self.last_error = "waiting for native packets"

    def _callback(self, context, event, status, pointer):
        try:
            if event & DISCONNECT:
                self.fault.set()
            elif event == STATE and pointer:
                state = C.cast(pointer, C.POINTER(DeviceState)).contents
                device = bytes(state.devID).decode("utf-8")
                # Pin the first device; another headset cannot switch inputs.
                if self.device is None:
                    self.device = device
                if device != self.device:
                    return
                self.items.put_nowait((time.monotonic(), bytes(state.stateJson)))
        except Exception:
            self.fault.set()  # Overflow or malformed ABI payload fails closed.

    def start(self):
        if self.lib is not None or self.lock is not None:
            raise RuntimeError("Native input is already started")
        if not SDK.is_file():
            raise RuntimeError(f"Native SDK is missing: {SDK}")
        self.lock = (ROOT / ".xrobotoolkit-input.lock").open("a+")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            self.lock.close()
            self.lock = None
            raise RuntimeError("另一个原生采样/遥操进程正在使用 SDK；先退出它，再启动本命令") from error
        try:
            self.lib = C.CDLL(str(SDK))
            self.lib.PXREAInit.argtypes = [C.c_void_p, Callback, C.c_uint]
            self.lib.PXREAInit.restype = C.c_int
            self.lib.PXREADeinit.argtypes = []
            self.lib.PXREADeinit.restype = C.c_int
            result = self.lib.PXREAInit(None, self.callback, 0xffffffff)
            if result != 0:
                raise RuntimeError(f"PXREAInit failed: {result}")
        except BaseException:
            self.close()
            raise

    def close(self):
        try:
            if self.lib is not None:
                self.lib.PXREADeinit()
        finally:
            self.lib = None
            if self.lock is not None:
                self.lock.close()
                self.lock = None

    def drain(self, observe, trip, now=None):
        now = time.monotonic() if now is None else now
        if self.fault.is_set():
            self.fault.clear()
            while not self.items.empty():
                try:
                    self.items.get_nowait()
                except queue.Empty:
                    break
            self.last_stamp = self.offset = self.last_arrival = None
            self.last_error = "native device/service disconnected or callback overflow"
            trip(self.last_error)
            return
        for _ in range(128):
            try:
                arrival, raw = self.items.get_nowait()
            except queue.Empty:
                return
            try:
                data = decode_state_json(raw)
                stamp = data["timeStampNs"]
                if stamp == self.last_stamp:
                    duplicate = right_hand(data, self.seq)
                    if not duplicate["valid"] or (self.last and
                            ((self.last["grip"] >= self.grip_threshold and duplicate["grip"] < self.grip_threshold) or
                             (min(self.last["grip"], self.last["trigger"]) >= .8 and
                              min(duplicate["grip"], duplicate["trigger"]) < .8))):
                        trip("native duplicate frame lost tracking/focus or released control")
                    continue  # A repeated source frame never refreshes freshness.
                prior_stamp, prior_arrival = self.last_stamp, self.last_arrival
                offset = arrival - stamp / 1e9
                self.offset = offset if self.offset is None else min(offset, self.offset)
                self.last_stamp, self.last_arrival = stamp, arrival
                self.seq += 1
                hand = right_hand(data, self.seq)
                hand["source_timestamp_ns"] = stamp
                self.last = hand
                self.frames += 1
                self.tracking_frames += int(hand["valid"])
                if now - arrival > .03 or offset - self.offset > .05:
                    raise ValueError("native frame backlog")
                if prior_stamp is not None and (stamp < prior_stamp or
                        (stamp-prior_stamp)/1e9 > .1 or arrival-prior_arrival > .1):
                    self.offset = offset
                    raise ValueError("native frame clock reset or input gap")
                self.last_error = ("" if hand["valid"] else
                                   "right tracking invalid or explicit poseValid missing")
                observe(hand, arrival)
            except (ValueError, TypeError, KeyError, AttributeError, OverflowError) as error:
                self.rejected += 1
                self.last_error = str(error)
                trip("native input: " + str(error))

    async def pump(self, gate, pose):
        try:
            while True:
                self.drain(lambda hand, now: gate.observe(hand, now, pose()), gate.trip)
                await asyncio.sleep(.005)
        finally:
            if not getattr(self, 'defer_close', False):
                self.close()


async def monitor(seconds, output):
    source = NativeInput()
    stream = None
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        stream = output.open("w")
    def event(hand, now):
        if stream:
            stream.write(json.dumps({"host_monotonic": now, "source_ns": source.last_stamp,
                                     "device": source.device, **hand}) + "\n")
    source.start()
    began, printed = time.monotonic(), 0
    try:
        while time.monotonic() - began < seconds:
            source.drain(event, lambda reason: None)
            if time.monotonic() - printed >= 1:
                age = None if source.last_arrival is None else time.monotonic()-source.last_arrival
                print(json.dumps({"frames": source.frames, "device": source.device,
                                  "frame_age_s": age, "stream_fresh": age is not None and age <= .1,
                                  "right": source.last, "error": source.last_error}, ensure_ascii=False), flush=True)
                printed = time.monotonic()
            await asyncio.sleep(.005)
    finally:
        source.close()
        if stream:
            stream.close()
        print(json.dumps({"frames": source.frames, "valid_tracking_frames": source.tracking_frames,
                          "rejected": source.rejected, "robot_commands_sent": 0}), flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--seconds", type=float, default=15)
    p.add_argument("--output", type=Path)
    a = p.parse_args()
    if not math.isfinite(a.seconds) or not 0 < a.seconds <= 600:
        p.error("seconds must be in (0, 600]")
    try:
        asyncio.run(monitor(a.seconds, a.output))
    except KeyboardInterrupt:
        pass
