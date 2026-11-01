"""Robot profile checks, native input interlocks, read-only RPC and verified stop."""
from __future__ import annotations
import asyncio
from contextlib import suppress
from dataclasses import dataclass, fields
import ipaddress
import json
import math
from pathlib import Path
import time
ROOT = Path(__file__).resolve().parent

def parse_axes(text):
    parts = [part.strip() for part in text.split(",")]
    if len(parts) != 3 or any(part not in ("x", "y", "z", "+x", "+y", "+z", "-x", "-y", "-z") for part in parts):
        raise ValueError("axis-map must be a signed permutation, e.g. x,-z,y")
    if {part[-1] for part in parts} != {"x", "y", "z"}:
        raise ValueError("Each VR axis must occur exactly once")
    return [("xyz".index(part[-1]), -1 if part.startswith("-") else 1) for part in parts]

def decode_state(response):
    state = response["arm_state"]
    joints, pose, errors = state["joint"], state["pose"], state["err"]
    if len(joints) != 6 or len(pose) != 6 or not errors:
        raise ValueError("Incomplete robot feedback")
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
               and math.isfinite(v) for v in joints + pose + errors):
        raise ValueError("Invalid robot feedback values")
    if any(value != 0 for value in errors):
        raise ValueError("Robot reports an error: " + str(errors))
    degrees = [value / 1000 for value in joints]
    return {
        "joint_deg": degrees,
        "joint_rad": [math.radians(value) for value in degrees],
        "tcp_pose_m_rad": [value / 1e6 for value in pose[:3]]
                            + [value / 1000 for value in pose[3:]],
        "errors": errors,
    }
def norm(v):
    return math.sqrt(sum(x * x for x in v))

def distance(a, b):
    return norm([x - y for x, y in zip(a, b)])

@dataclass(frozen=True)
class Config:
    robot_ip: str = "192.168.0.18"
    robot_port: int = 8080
    feedback_timeout_s: float = .2
    expected_work_frame: str = "Base"
    expected_tool_name: str = "tcp"
    expected_tool_pose_raw: tuple = (0, 0, 145000, 0, 0, 0)
    expected_payload_raw: int = 500
    expected_collision_stage: int = 4
    expected_product: str = "RM65-BI"
    expected_firmware: str = "V1.7.4"

    def validate(self):
        ipaddress.ip_address(self.robot_ip)
        if type(self.robot_port) is not int or not 1 <= self.robot_port <= 65535:
            raise ValueError("Invalid robot port")
        if (type(self.feedback_timeout_s) not in (int, float)
                or not math.isfinite(self.feedback_timeout_s)
                or not .05 <= self.feedback_timeout_s <= .2):
            raise ValueError("feedback_timeout_s must be within [.05, .2]")
        if (self.expected_work_frame != "Base" or self.expected_tool_name != "tcp"
                or list(self.expected_tool_pose_raw) != [0, 0, 145000, 0, 0, 0]
                or self.expected_payload_raw != 500 or self.expected_collision_stage != 4
                or self.expected_product != "RM65-BI" or self.expected_firmware != "V1.7.4"):
            raise ValueError("Hardware profile changed; this version supports the checked RM65-BI profile only")
        return self

    @classmethod
    def load(cls, path):
        values = json.loads(Path(path).read_text())
        unknown = set(values) - {f.name for f in fields(cls)}
        if unknown:
            raise ValueError(f"Unknown config fields: {sorted(unknown)}")
        return cls(**values).validate()

class Gate:
    """Observe EVERY received packet, including releases between control ticks.

    A fault invalidates the anchor. Recovery while buttons remain held cannot
    resume motion; both controls must be released under valid tracking for 1 s.
    Freshness refers to host receipt of new sequence numbers, not sensor latency.
    """
    controls_label = "right Grip + Trigger"

    @property
    def dwell_s(self):
        return self.c.release_dwell_s

    def control_value(self, hand):
        return min(hand["grip"], hand["trigger"])

    def is_released(self, hand):
        return max(hand["grip"], hand["trigger"]) < .2

    def __init__(self, config):
        self.c = config
        self.axes = parse_axes(config.axis_map)
        self.seq = None
        self.received = None
        self.hand = None
        self.released_since = None
        self.ready = False
        self.active = False
        self.anchor = None
        self.target = None
        self.epoch = 0
        self.reason = "waiting for valid tracking and both controls released"
        self.packets = self.duplicates = self.rejected = 0

    def trip(self, reason):
        if self.active or self.ready or self.target is not None:
            self.epoch += 1
        self.active = self.ready = False
        self.anchor = self.target = self.released_since = None
        self.reason = reason

    def observe(self, hand, now, robot_pose):
        seq = hand["seq"]
        if seq < 0 or seq > 16000000:
            self.trip("invalid sequence")
            return
        if seq == self.seq:
            self.duplicates += 1
            # A safety transition is never hidden by sequence deduplication.
            if not hand["valid"] or (self.active and self.control_value(hand) < .8):
                self.trip("tracking lost or control released")
            return
        previous, prior_time, prior_seq = self.hand, self.received, self.seq
        self.seq, self.received, self.hand = seq, now, hand
        self.packets += 1
        if (prior_seq is not None and seq < prior_seq):
            self.trip("XR session/sequence reset; release both controls")
            return
        if prior_time is not None and now - prior_time > self.c.input_timeout_s:
            self.trip("input gap; release both controls")
            return
        if not hand["valid"]:
            self.trip("tracking lost; release both controls")
            return
        if self.active and self.control_value(hand) < .8:
            self.trip("control released")
            return
        if previous and previous["valid"] and prior_time is not None:
            moved = distance(hand["position_m"], previous["position_m"])
            # Reject discontinuous reacquisition and implausibly fast hand jumps.
            if moved > .03 or moved > 1.5 * max(now - prior_time, .002) + .003:
                self.trip("hand pose jump; release both controls")
                return
        if not self.active:
            if self.is_released(hand):
                if self.released_since is None:
                    self.released_since = now
                if now - self.released_since >= self.dwell_s:
                    self.ready, self.reason = True, f"READY: hold {self.controls_label}"
                else:
                    self.reason = (f"ARMING: both controls released "
                                   f"{now-self.released_since:.1f}/{self.dwell_s:.1f}s")
            elif self.ready and self.control_value(hand) >= .8:
                self.active, self.ready = True, False
                self.anchor = (hand["position_m"][:], robot_pose[:])
                self.reason = "following"
            elif not self.ready:
                self.released_since = None
                self.reason = f"WAIT RELEASE: fully release {self.controls_label}"
        if self.active:
            origin, tcp = self.anchor
            delta = [self.c.position_scale * sign * (hand["position_m"][index] - origin[index])
                     for index, sign in self.axes]
            self.target = [tcp[i] + delta[i] for i in range(3)] + tcp[3:]

    def fresh(self, now):
        return (self.received is not None and 0 <= now - self.received <= self.c.input_timeout_s
                and self.hand is not None and self.hand["valid"])

    def watchdog(self, now):
        if not self.fresh(now):
            self.trip("no fresh valid right-hand data; release both controls")

class SimpleGate(Gate):
    """Immediate Grip clutch with a zero-displacement startup anchor.

    The command authorizes startup. First valid input can anchor while held;
    faults still require release / squeeze and never resume a held Grip.
    """
    controls_label = "right Grip"

    def __init__(self, config):
        super().__init__(config)
        self.initial_valid_seen = False

    def observe(self, hand, now, robot_pose):
        if hand["valid"] and not self.initial_valid_seen:
            self.initial_valid_seen = True
            self.seq = self.received = self.hand = None
            self.ready = True
        super().observe(hand, now, robot_pose)

    @property
    def dwell_s(self):
        return 0.

    def control_value(self, hand):
        return hand["grip"]

    def is_released(self, hand):
        return hand["grip"] < .2

    def trip(self, reason):
        super().trip(reason.replace("both controls", "Grip"))

READS = {
    "get_current_arm_state", "get_current_work_frame", "get_current_tool_frame",
    "get_arm_software_info", "get_arm_run_mode", "get_collision_stage",
    "get_joint_en_state", "get_joint_min_pos", "get_joint_max_pos",
    "get_program_run_state", "get_arm_current_trajectory",
}


class MotionPermitExpired(RuntimeError):
    """Input interlock revoked before any motion bytes were sent."""

class ArmRPC:
    """Independent readback/STOP transport; no arm motion API."""
    def __init__(self, config, execute=False):
        self.c, self.execute = config, execute
        self.motion_attempts = 0
        self.stop_attempts = 0
        self.read_lock = asyncio.Lock()

    async def call(self, command, payload=None, permit=None):
        # Serialize ordinary queries. STOP uses an independent connection and
        # never waits for the feedback lock or a delayed query response.
        if command in READS:
            async with self.read_lock:
                return await self._exchange(command, payload, permit)
        return await self._exchange(command, payload, permit)

    async def _exchange(self, command, payload=None, permit=None):
        if command not in READS and not (self.execute and command == "set_arm_stop"):
            raise RuntimeError(f"Command forbidden by transport policy: {command}")
        reader, writer = await asyncio.wait_for(asyncio.open_connection(
            self.c.robot_ip, self.c.robot_port, limit=65536), .15)
        try:
            if command == "set_arm_stop":
                self.stop_attempts += 1
            writer.write(json.dumps({"command": command, **(payload or {})}).encode() + b"\r\n")
            await asyncio.wait_for(writer.drain(), .15)
            for _ in range(8):
                line = await asyncio.wait_for(reader.readline(), .15)
                if not line or len(line) > 65536:
                    raise RuntimeError("Incomplete controller reply")
                reply = json.loads(line)
                if not isinstance(reply, dict):
                    raise ValueError("Controller response must be a JSON object")
                if command == "set_arm_stop":
                    if "arm_stop" in reply:
                        if reply["arm_stop"] is True:
                            return reply
                        raise RuntimeError("Stop was not acknowledged")
                else:
                    # Controllers may emit status notifications before the reply.
                    # Never decode an unrelated message as a fresh measurement.
                    expected = {
                        "get_current_arm_state": "arm_state", "get_current_work_frame": "frame_name",
                        "get_current_tool_frame": "tool_name", "get_arm_software_info": "Product_version",
                        "get_arm_run_mode": "mode", "get_collision_stage": "collision_stage",
                        "get_joint_en_state": "en_state", "get_joint_min_pos": "min_pos",
                        "get_joint_max_pos": "max_pos", "get_program_run_state": "run_state",
                        "get_arm_current_trajectory": "type",
                    }[command]
                    if expected in reply:
                        return reply
            raise RuntimeError(f"Expected {command} reply not received; last message: {reply}")
        finally:
            writer.close()
            with suppress(Exception):
                await asyncio.wait_for(writer.wait_closed(), .1)

    async def snapshot(self):
        # Timestamp BEFORE queries, so time spent reading never refreshes stale data.
        stamp = time.monotonic()
        state = decode_state(await self.call("get_current_arm_state"))
        trajectory = await self.call("get_arm_current_trajectory")
        if not isinstance(trajectory.get("type"), str):
            raise ValueError("Unknown controller trajectory response")
        return {"received_at": stamp, "state": state, "trajectory": trajectory}

    async def profile(self):
        started = time.monotonic()
        result = {name: await self.call(name) for name in [
            "get_current_work_frame", "get_current_tool_frame", "get_arm_run_mode",
            "get_collision_stage", "get_joint_en_state", "get_program_run_state",
        ]}
        result["_started_at"] = started
        return result

def validate_profile(c, profile):
    work, tool = profile["get_current_work_frame"], profile["get_current_tool_frame"]
    if work.get("frame_name") != c.expected_work_frame or work.get("pose") != [0] * 6:
        raise ValueError("Current work frame must be Base with zero offset")
    if (tool.get("tool_name") != c.expected_tool_name
            or tool.get("pose") != list(c.expected_tool_pose_raw)
            or tool.get("payload") != c.expected_payload_raw or tool.get("position") != [0, 0, 0]):
        raise ValueError("Current tcp tool / 145 mm offset / 0.5 kg payload / load centre changed")
    if profile["get_arm_run_mode"].get("mode") != 1:
        raise ValueError("Controller is not in real robot mode")
    if profile["get_collision_stage"].get("collision_stage") != c.expected_collision_stage:
        raise ValueError("Expected collision stage 4; configuration is never changed automatically")
    if profile["get_joint_en_state"].get("en_state") != [1] * 6:
        raise ValueError("All six axes must already be enabled by the operator")
    if profile["get_program_run_state"].get("run_state") != 0:
        raise ValueError("An online program is running or paused")

async def preflight(rpc):
    c = rpc.c
    profile = await rpc.profile()
    validate_profile(c, profile)
    software = await rpc.call("get_arm_software_info")
    if software.get("Product_version") != c.expected_product or any(
            software.get(k, {}).get("version") != c.expected_firmware for k in ("ctrl_info", "plan_info")):
        raise ValueError("Unexpected robot product or firmware")
    low = (await rpc.call("get_joint_min_pos")).get("min_pos")
    high = (await rpc.call("get_joint_max_pos")).get("max_pos")
    if (not isinstance(low, list) or not isinstance(high, list) or len(low) != 6 or len(high) != 6
            or not all(type(v) is int for v in low + high)
            or any(a >= b for a, b in zip(low, high))):
        raise ValueError("Invalid controller joint limits")
    samples = []
    for _ in range(4):
        s = await rpc.snapshot()
        if s["trajectory"]["type"] != "none":
            raise ValueError("Robot has an active trajectory; do not share control")
        samples.append(s)
        await asyncio.sleep(.1)
    anchor = samples[-1]["state"]
    for s in samples:
        if (distance(s["state"]["tcp_pose_m_rad"][:3], anchor["tcp_pose_m_rad"][:3]) > .0001
                or max(abs(a - b) for a, b in zip(s["state"]["joint_deg"], anchor["joint_deg"])) > .03):
            raise ValueError("Robot is not stationary at startup")
    bounds = ([v / 1000 for v in low], [v / 1000 for v in high])
    check_startup_joints(samples[-1], bounds)
    return {"profile": profile, "software": software, "anchor": anchor, "bounds": bounds}

def check_startup_joints(snapshot, bounds):
    low, high = bounds
    if any(q < lo + 5 or q > hi - 5 for q, lo, hi in zip(snapshot["state"]["joint_deg"], low, high)):
        raise ValueError("Less than 5 degree margin to a joint limit")


async def controlled_stop(rpc, recorder, observer=None):
    acknowledged = False
    errors = []
    # Repeating STOP is allowed; repeating an ambiguous MoveL is never allowed.
    for _ in range(2):
        try:
            await rpc.call("set_arm_stop")
            acknowledged = True
            break
        except Exception as error:
            errors.append(str(error))
    if acknowledged:
        deadline, since, reference = time.monotonic() + 2., None, None
        while time.monotonic() < deadline:
            try:
                snap = await rpc.snapshot()
                if observer is not None:
                    observer(snap)
                q = snap["state"]["joint_deg"]
                if snap["trajectory"]["type"] == "none":
                    if reference is None or max(abs(a-b) for a,b in zip(q, reference)) > .03:
                        reference, since = q, time.monotonic()
                    if time.monotonic() - since >= .3:
                        recorder.event("stop_verified", stop_attempts=rpc.stop_attempts)
                        return
                else:
                    reference = since = None
            except Exception as error:
                errors.append(str(error))
            await asyncio.sleep(.05)
    recorder.event("stop_unverified", errors=errors)
    raise RuntimeError("Cannot verify robot stop. Use the physical E-stop and inspect the controller.")
