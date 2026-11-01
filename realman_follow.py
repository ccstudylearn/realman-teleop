"""Native Quest relative 6D clutch -> RealMan IK -> continuous joint targets.

Independent RealMan implementation of the arx-televr input/IK/backend pattern.
Default is preview; --direct authorizes real motion without a terminal prompt.
No Home, gravity compensation or automatic configuration/enable commands.
"""
from __future__ import annotations

import argparse
import asyncio
from collections import deque
from contextlib import suppress
from dataclasses import asdict, dataclass, fields
from datetime import datetime
import fcntl
import hashlib
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
from scipy.spatial.transform import Rotation

from robot_core import parse_axes
from robot_core import (ROOT, Config, SimpleGate, MotionPermitExpired,
                           controlled_stop, preflight, validate_profile)
from follow_recording import BufferedRecorder
from follow_transport import FollowRPC
from gripper_transport import GripperTransport

CONTROL_VERSION = 'scaled-wrist-v4.4-core.2'


def code_provenance():
    names = ('realman_follow.py', 'robot_core.py', 'follow_transport.py',
             'gripper_transport.py', 'follow_recording.py', 'xrobotoolkit_input.py', 'teleop.py')
    return dict(control_version=CONTROL_VERSION,
                source_sha256={name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in names})


@dataclass(frozen=True)
class FollowConfig:
    axis_map: str = "-x,z,y"
    axis_yaw_deg: float = 0.
    direction_configured: bool = False
    position_scale: float = 1.
    rotation_scale: float = 0.5
    control_hz: float = 50.
    workspace_radius_mm: float = 400.
    max_rotation_deg: float = 180.
    tcp_speed_mm_s: float = 100.
    rotation_speed_deg_s: float = 60.
    joint_speed_deg_s: float = 45.
    joint_accel_deg_s2: float = 120.
    joint_margin_deg: float = 3.
    joint_tracking_error_deg: float = 8.
    max_duration_s: float = 3600.
    gripper_enabled: bool = True
    gripper_speed: int = 300
    gripper_force: int = 100
    input_timeout_s: float = .1

    def validate(self):
        bounds = dict(position_scale=(.1, 2.), rotation_scale=(0., 1.5), control_hz=(20., 50.),
                      workspace_radius_mm=(10., 1000.), max_rotation_deg=(1., 180.),
                      tcp_speed_mm_s=(1., 200.), rotation_speed_deg_s=(1., 90.),
                      joint_speed_deg_s=(1., 60.), joint_accel_deg_s2=(1., 180.),
                      joint_margin_deg=(2., 10.), joint_tracking_error_deg=(1., 10.),
                      max_duration_s=(1., 7200.), input_timeout_s=(.03, .1))
        for name, (low, high) in bounds.items():
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{name}: expected finite value within [{low}, {high}]")
        if type(self.gripper_enabled) is not bool or type(self.direction_configured) is not bool:
            raise ValueError("gripper_enabled and direction_configured must be boolean")
        if (type(self.axis_yaw_deg) not in (int, float) or not math.isfinite(self.axis_yaw_deg)
                or not -180 <= self.axis_yaw_deg <= 180):
            raise ValueError("axis_yaw_deg must be a finite angle within [-180, 180]")
        for name, lo, hi in (("gripper_speed", 1, 1000), ("gripper_force", 50, 1000)):
            value = getattr(self, name)
            if type(value) is not int or not lo <= value <= hi:
                raise ValueError(f"Invalid {name}")
        matrix = axis_matrix(self.axis_map)
        if np.linalg.det(matrix) < .99:
            raise ValueError("6D axis mapping must be a proper rotation, not a reflection")
        return self

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text())
        if set(data) - {f.name for f in fields(cls)}:
            raise ValueError("Unknown follow configuration fields")
        return cls(**data).validate()


def axis_matrix(mapping, yaw_deg=0.):
    m = np.zeros((3, 3))
    for row, (index, sign) in enumerate(parse_axes(mapping)):
        m[row, index] = sign
    return Rotation.from_euler('z', yaw_deg, degrees=True).as_matrix() @ m


def continuous_rotvec(rotation, previous):
    """Choose the relative-angle branch nearest the previous accepted frame."""
    vector = rotation.as_rotvec()
    angle = np.linalg.norm(vector)
    if previous is None:
        return vector
    if angle < 1e-8:
        length = np.linalg.norm(previous)
        turns = round(length/(2*math.pi))
        return previous*(2*math.pi*turns/length) if length > 1e-8 else vector
    axis = vector/angle
    center = round(np.dot(previous, axis)/(2*math.pi))
    choices = [axis*(angle+2*math.pi*k) for k in range(center-1, center+2)]
    return min(choices, key=lambda candidate: np.linalg.norm(candidate-previous))


class PoseGate(SimpleGate):
    """Keep original native freshness/release gate; add relative SO(3) rotation."""
    def __init__(self, config):
        super().__init__(config)
        self.matrix = axis_matrix(config.axis_map, config.axis_yaw_deg)
        self.rotation_anchor = None
        self.rotation_vector = None
        self.trigger_previous = None
        self.gripper_edge = None
        self.gripper_latched = None
        self.gripper_generation = 0
        self.gripper_intent_stamp = None
        self.pending_interlock = None

    def control_value(self, hand):
        return 1. if hand["grip"] >= .7 else 0.

    def trip_motion(self, reason):
        epoch = self.epoch
        super().trip(reason)
        self.rotation_anchor = None
        self.rotation_vector = None
        if self.epoch != epoch and self.pending_interlock is None:
            # Several queued frames may arrive before the control loop runs.
            # Later WAIT RELEASE / READY messages must not hide the first fault.
            self.pending_interlock = dict(reason=self.reason, triggered_at=time.monotonic(),
                hand_at_trigger=({k: self.hand.get(k) for k in ('seq', 'grip', 'trigger', 'valid')}
                                 if self.hand else None))

    def trip(self, reason):
        self.trip_motion(reason)
        self.cancel_gripper_intent()

    def cancel_gripper_intent(self):
        self.gripper_edge = None
        self.gripper_latched = None
        self.gripper_intent_stamp = None
        self.gripper_generation += 1
        self.trigger_previous = None

    def observe(self, hand, now, robot_pose):
        if not hand['valid'] or hand['grip'] < .7:
            self.cancel_gripper_intent()
        if hand["seq"] == self.seq:
            super().observe(hand, now, robot_pose)
            return
        try:
            q = np.array(hand["quaternion_xyzw"], dtype=float)
            if q.shape != (4,) or not np.isfinite(q).all() or not .9 <= np.linalg.norm(q) <= 1.1:
                raise ValueError("invalid hand quaternion")
            rotation = Rotation.from_quat(q)
            if self.hand and self.hand["valid"] and hand["valid"] and hand["seq"] != self.seq:
                dt = max(.002, now - self.received)
                angle = (rotation * Rotation.from_quat(self.hand["quaternion_xyzw"]).inv()).magnitude()
                if angle > math.radians(30) or angle > math.radians(720)*dt + .035:
                    raise ValueError("hand orientation jump")
        except (ValueError, KeyError, TypeError) as error:
            self.trip(str(error))
            return
        was_active = self.active
        super().observe(hand, now, robot_pose)
        if self.active:
            if not was_active:
                self.rotation_anchor = (rotation, Rotation.from_euler("xyz", robot_pose[3:]))
                self.rotation_vector = np.zeros(3)
            origin, tcp_pose = self.anchor
            delta = self.matrix@(np.array(hand['position_m'])-np.array(origin))
            self.target[:3] = (np.array(tcp_pose[:3])+self.c.position_scale*delta).tolist()
            source, tcp = self.rotation_anchor
            delta = rotation * source.inv()
            mapped = Rotation.from_matrix(self.matrix @ delta.as_matrix() @ self.matrix.T)
            self.rotation_vector = continuous_rotvec(mapped, self.rotation_vector)
            target = Rotation.from_rotvec(self.rotation_vector*self.c.rotation_scale) * tcp
            self.target[3:] = target.as_euler("xyz").tolist()
        previous, self.trigger_previous = self.trigger_previous, hand["trigger"]
        if hand['valid'] and hand['grip'] >= .7 and previous is not None:
            if self.active and previous < .8 <= hand["trigger"] and self.gripper_latched != 'close':
                self.gripper_generation += 1
                self.gripper_edge = ("close", now, self.gripper_generation)
                self.gripper_latched = "close"
                self.gripper_intent_stamp = now
            elif previous > .2 >= hand["trigger"] and self.gripper_latched != 'open':
                # Grip is still the physical deadman, but a stopped/limited
                # arm planner must not suppress the operator's release intent.
                self.gripper_generation += 1
                self.gripper_edge = ("open", now, self.gripper_generation)
                self.gripper_latched = "open"
                self.gripper_intent_stamp = now


class PlanningConstraint(ValueError):
    """A candidate trajectory is blocked; this never permits an unsafe target."""
    def __init__(self, kind, message):
        super().__init__(message)
        self.kind = kind


class RealManModel:
    """Official offline SDK. No SDK network connection or actuation is created."""
    def __init__(self, low, high):
        from Robotic_Arm.rm_robot_interface import (Algo, rm_robot_arm_model_e,
                                                   rm_force_type_e, rm_frame_t)
        self.algo = Algo(rm_robot_arm_model_e.RM_MODEL_RM_65_E, rm_force_type_e.RM_MODEL_RM_B_E)
        self.algo.rm_algo_set_workframe(rm_frame_t("Base", [0]*6, 0., 0., 0., 0.))
        self.algo.rm_algo_set_toolframe(rm_frame_t("tcp", [0., 0., .145, 0., 0., 0.], .5, 0., 0., 0.))
        self.algo.rm_algo_set_joint_min_limit(low)
        self.algo.rm_algo_set_joint_max_limit(high)
        self.algo.rm_algo_set_redundant_parameter_traversal_mode(False)
        self.version = str(self.algo.rm_algo_version())

    def fk(self, q):
        pose = self.algo.rm_algo_forward_kinematics(list(q))
        if len(pose) != 6 or not np.isfinite(pose).all():
            raise ValueError("Invalid forward kinematics")
        return np.array(pose)

    def ik(self, seed, pose):
        from Robotic_Arm.rm_robot_interface import rm_inverse_kinematics_params_t
        ret, q = self.algo.rm_algo_inverse_kinematics(
            rm_inverse_kinematics_params_t(list(seed), list(pose), 1))
        if ret != 0 or len(q) != 6 or not np.isfinite(q).all():
            raise ValueError("Target is unreachable; release Grip and choose another pose")
        return np.array(q)

    def check(self, q):
        if self.algo.rm_algo_safety_robot_self_collision_detection(list(q)) != 0:
            raise PlanningConstraint('collision', "Model self-collision / joint limit check failed")
        singular, _ = self.algo.rm_algo_kin_robot_singularity_analyse(list(q))
        if singular != 0:
            raise PlanningConstraint('singularity', "Target near a kinematic singularity")


class JointPlanner:
    def __init__(self, config, model, q, bounds):
        self.c, self.model = config, model
        self.low, self.high = (np.array(v) for v in bounds)
        self.origin = model.fk(q)
        self.reset(q)

    def reset(self, q):
        self.q = np.array(q, dtype=float)
        self.velocity = np.zeros(6)
        self.measured = self.q.copy()
        self.measured_velocity = np.zeros(6)
        self.measured_at = None
        self.feedback_scale = 1.
        self.trajectory_scale = 1.
        self.limiting_reason = ''
        self.component = 'six_d'
        self.state_checked = False

    def observe(self, q, stamp):
        q = np.array(q, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all() or not math.isfinite(stamp):
            raise ValueError("Invalid measured joint feedback")
        if self.measured_at is not None and .005 <= stamp-self.measured_at <= .2:
            speed = np.clip((q-self.measured)/(stamp-self.measured_at),
                            -self.c.joint_speed_deg_s, self.c.joint_speed_deg_s)
            self.measured_velocity = .5*self.measured_velocity + .5*speed
        else:
            self.measured_velocity = np.zeros(6)
        self.measured, self.measured_at = q, stamp

    def check_pose(self, pose):
        if np.asarray(pose).shape != (6,) or not np.isfinite(pose).all():
            raise ValueError("Target pose is incomplete or nonfinite")
        if np.linalg.norm(np.array(pose[:3])-self.origin[:3]) > self.c.workspace_radius_mm/1000 + 1e-5:
            raise ValueError("Target outside configured workspace radius")
        angle = (Rotation.from_euler("xyz", pose[3:]) * Rotation.from_euler("xyz", self.origin[3:]).inv()).magnitude()
        if angle > math.radians(self.c.max_rotation_deg) + .002:
            raise ValueError("Target outside configured rotation range")

    def step(self, target, dt):
        # Fixed upper time step: scheduler delays never cause larger catch-up steps.
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Invalid planner timestep')
        dt = min(dt, 1/self.c.control_hz)
        target = np.array(target, dtype=float)
        if target.shape != (6,) or not np.isfinite(target).all():
            raise ValueError('Target pose is incomplete or nonfinite')
        offset = target[:3]-self.origin[:3]
        radius = self.c.workspace_radius_mm/1000
        workspace_limited = np.linalg.norm(offset) > radius
        if workspace_limited:
            target[:3] = self.origin[:3]+offset*(radius-1e-5)/np.linalg.norm(offset)
        self.check_pose(target)
        pose = self.model.fk(self.q)
        if not self.state_checked:
            # A soft candidate boundary is different from starting inside an
            # unsafe measured state. The latter still latches the stop gate.
            self.model.check(self.q)
            self._check_joint_margin(self.q)
            self.state_checked = True
        d = np.array(target[:3])-pose[:3]
        rotation = Rotation.from_euler("xyz", pose[3:])
        rv = (Rotation.from_euler("xyz", target[3:])*rotation.inv()).as_rotvec()
        # Translate and rotate along one 6D interpolation, rather than reaching
        # orientation first and changing the IK path while the hand retreats.
        task_fraction = min(1., self.c.tcp_speed_mm_s/1000*dt/max(np.linalg.norm(d), 1e-12),
                            math.radians(self.c.rotation_speed_deg_s)*dt/max(np.linalg.norm(rv), 1e-12))
        primary = self._proposal(pose, d*task_fraction, rv*task_fraction, dt)
        chosen = primary
        component = 'six_d'
        # At a wrist/elbow boundary, requested rotation can block a feasible
        # lift. Try the operator's translation with orientation held, then the
        # requested rotation with position held. Never invent an escape motion.
        if primary['reason'] in ('singularity', 'joint_limit', 'ik_boundary', 'workspace'):
            if np.linalg.norm(d) > 1e-6:
                translation = self._proposal(pose, d*min(1., self.c.tcp_speed_mm_s/1000*dt/
                                             np.linalg.norm(d)), np.zeros(3), dt)
                direction = d/np.linalg.norm(d)
                original_progress = (primary['pose'][:3]-pose[:3])@direction
                alternative_progress = (translation['pose'][:3]-pose[:3])@direction
                if alternative_progress > max(0., original_progress)+1e-5:
                    chosen = translation
                    chosen['reason'] = 'position_priority_'+primary['reason']
                    component = 'translation'
            # A small horizontal hand drift can make the complete translation
            # infeasible at the elbow wall even though the requested vertical
            # movement is feasible. Test that existing component explicitly;
            # do not add a tilt, horizontal escape, or a new target direction.
            if abs(d[2]) > .0002:
                vertical_delta = np.array([0., 0., np.clip(d[2],
                    -self.c.tcp_speed_mm_s/1000*dt, self.c.tcp_speed_mm_s/1000*dt)])
                vertical = self._proposal(pose, vertical_delta, np.zeros(3), dt)
                sign = np.sign(d[2])
                vertical_progress = (vertical['pose'][2]-pose[2])*sign
                current_progress = (chosen['pose'][2]-pose[2])*sign
                goal_progress = (vertical['pose'][:3]-pose[:3])@d/max(np.linalg.norm(d), 1e-12)
                if vertical_progress > max(0., current_progress)+1e-5 and goal_progress >= -1e-6:
                    chosen = vertical
                    chosen['reason'] = 'vertical_priority_'+primary['reason']
                    component = 'vertical'
            if chosen is primary and np.linalg.norm(rv) > 1e-5:
                turn = self._proposal(pose, np.zeros(3), rv*min(1., math.radians(
                    self.c.rotation_speed_deg_s)*dt/np.linalg.norm(rv)), dt)
                direction = rv/np.linalg.norm(rv)
                def angular_progress(result):
                    return (Rotation.from_euler('xyz', result['pose'][3:])*
                            rotation.inv()).as_rotvec()@direction
                if angular_progress(turn) > max(0., angular_progress(primary))+1e-4:
                    chosen = turn
                    chosen['reason'] = 'rotation_priority_'+primary['reason']
                    component = 'rotation'
        self.q, self.velocity = chosen['q'], chosen['velocity']
        self.feedback_scale, self.trajectory_scale = chosen['feedback_scale'], chosen['fraction']
        self.limiting_reason = chosen['reason'] or ('workspace' if workspace_limited else '')
        self.component = component
        return self.q, chosen['pose']

    def _proposal(self, pose, d, rv, dt):
        rotation = Rotation.from_euler('xyz', pose[3:])
        desired = None
        for attempt in range(7):
            fraction = .5**attempt
            candidate = np.r_[pose[:3]+d*fraction,
                              (Rotation.from_rotvec(rv*fraction)*rotation).as_euler("xyz")]
            try:
                solution = self.model.ik(self.q, candidate)
            except ValueError:
                continue
            if np.max(np.abs(solution-self.q)) <= 5.:
                desired = solution
                break
        requested = np.zeros(6) if desired is None else (desired-self.q)/dt
        # Common scaling preserves the IK joint direction; independent clipping
        # destroys cancellation between joints and can increase TCP speed.
        requested *= min(1., self.c.joint_speed_deg_s/max(np.max(np.abs(requested)), 1e-12))
        feedback_scale = 1.
        if self.measured_at is not None:
            # Brake the relative target lead, not the already-planned position.
            # This keeps velocity/acceleration continuous while the robot catches
            # up, instead of running an open-loop target into the fault threshold.
            direction = np.sign(requested)
            lead = (self.q-self.measured)*direction
            relative_speed = np.maximum(0., (self.velocity-self.measured_velocity)*direction)
            maximum_lead = min(3., self.c.joint_tracking_error_deg*.45)
            headroom = np.maximum(0., maximum_lead-lead-relative_speed*dt
                                  -.5*self.c.joint_accel_deg_s2*dt*dt)
            catchup = np.maximum(0., self.measured_velocity*direction)
            cap = catchup + np.sqrt(2*self.c.joint_accel_deg_s2*headroom)
            limited = np.minimum(np.abs(requested), cap)
            moving = np.abs(requested) > 1e-6
            if np.any(moving):
                feedback_scale = float(np.min(limited[moving]/np.abs(requested[moving])))
            requested *= feedback_scale
        acceleration_fraction = min(1., self.c.joint_accel_deg_s2*dt/
                                    max(np.max(np.abs(requested-self.velocity)), 1e-12))
        requested_velocity = self.velocity + acceleration_fraction*(requested-self.velocity)
        braking_velocity = self._brake(self.velocity, dt)
        reason = 'ik_boundary' if desired is None else ''
        # Both endpoints respect acceleration/speed limits. Blend toward a
        # previously validated stop trajectory, never crop an already planned
        # velocity discontinuously. Check the complete stop before committing.
        for fraction in (1., .5, .25, .125, .0625, .0):
            v = braking_velocity + fraction*(requested_velocity-braking_velocity)
            try:
                q, actual_pose = self._check_braking_path(pose, v, dt)
            except PlanningConstraint as error:
                reason = error.kind
                continue
            return dict(q=q, pose=actual_pose, velocity=v, fraction=fraction,
                        feedback_scale=feedback_scale,
                        reason=reason if fraction < 1. or desired is None else '')
        # This cannot be hidden as a normal hold: even the emergency braking
        # trajectory failed validation (e.g. an unsafe initial legacy velocity).
        raise ValueError('No safe braking trajectory: '+reason)

    def _brake(self, velocity, dt):
        fraction = max(0., 1.-self.c.joint_accel_deg_s2*dt/
                       max(np.max(np.abs(velocity)), 1e-12))
        return velocity*fraction

    def _check_joint_margin(self, q, planning=False):
        if np.any(q < self.low+self.c.joint_margin_deg) or np.any(q > self.high-self.c.joint_margin_deg):
            raise PlanningConstraint('joint_limit', 'Joint limit margin reached')
        if planning:
            # Keep planned stops 0.5 degrees inside the feedback stop threshold.
            # If a legacy stopped pose already occupies that buffer, permit
            # holding/retreating without allowing further approach to the limit.
            low = np.minimum(self.low+self.c.joint_margin_deg+.5,
                             np.minimum(self.q, np.round(self.q, 3)))
            high = np.maximum(self.high-self.c.joint_margin_deg-.5,
                              np.maximum(self.q, np.round(self.q, 3)))
            if np.any(q < low-1e-9) or np.any(q > high+1e-9):
                raise PlanningConstraint('joint_limit', 'Planning joint buffer reached')

    def _check_transition(self, previous_q, previous_pose, q, dt):
        self._check_joint_margin(q, planning=True)
        actual_pose = self.model.fk(q)
        try:
            self.check_pose(actual_pose)
        except ValueError as error:
            raise PlanningConstraint('workspace', str(error)) from error
        if np.linalg.norm(actual_pose[:3]-previous_pose[:3]) > self.c.tcp_speed_mm_s/1000*dt + 1e-6:
            raise PlanningConstraint('tcp_speed', 'Planned TCP speed exceeded configured limit')
        angle = (Rotation.from_euler('xyz', actual_pose[3:])*
                 Rotation.from_euler('xyz', previous_pose[3:]).inv()).magnitude()
        if angle > math.radians(self.c.rotation_speed_deg_s)*dt + 1e-5:
            raise PlanningConstraint('angular_speed', 'Planned angular speed exceeded configured limit')
        self.model.check(q)
        # The wire uses millidegrees. Validate the actual encoded target too;
        # rounding must not take a valid ideal target across a model boundary.
        encoded_q, encoded_previous = np.round(q, 3), np.round(previous_q, 3)
        self._check_joint_margin(encoded_q, planning=True)
        self.model.check(encoded_q)
        encoded_pose = self.model.fk(encoded_q)
        encoded_previous_pose = self.model.fk(encoded_previous)
        if np.linalg.norm(encoded_pose[:3]-encoded_previous_pose[:3]) > self.c.tcp_speed_mm_s/1000*dt + 1e-6:
            raise PlanningConstraint('tcp_speed', 'Encoded TCP speed exceeded configured limit')
        encoded_angle = (Rotation.from_euler('xyz', encoded_pose[3:])*
                         Rotation.from_euler('xyz', encoded_previous_pose[3:]).inv()).magnitude()
        if encoded_angle > math.radians(self.c.rotation_speed_deg_s)*dt + 1e-5:
            raise PlanningConstraint('angular_speed', 'Encoded angular speed exceeded configured limit')
        return actual_pose

    def _check_braking_path(self, pose, velocity, dt):
        q, previous_pose = self.q.copy(), pose
        first_q = first_pose = None
        approach_direction = velocity/max(np.max(np.abs(velocity)), 1e-12)
        # At the configured maximum this is <= 19 ticks at 50 Hz. No robot
        # commands are sent by this rollout; the SDK is offline kinematics only.
        ticks = int(math.ceil(np.max(np.abs(velocity))/self.c.joint_accel_deg_s2/dt))+2
        for index in range(ticks):
            previous_q = q
            q = q+velocity*dt
            actual_pose = self._check_transition(previous_q, previous_pose, q, dt)
            if index == 0:
                first_q, first_pose = q.copy(), actual_pose.copy()
            if np.max(np.abs(velocity)) < 1e-9:
                # A directional 0.5-degree joint-space reserve avoids parking
                # on the SDK's exact singularity threshold. This probe is never
                # commanded; actual braking states passed all model checks.
                try:
                    self.model.check(q+approach_direction*.5)
                except PlanningConstraint as error:
                    if error.kind == 'singularity':
                        raise
                return first_q, first_pose
            previous_pose = actual_pose
            velocity = self._brake(velocity, dt)
        raise RuntimeError('Braking rollout did not converge')


async def plan_step(planner, target, dt):
    """One offline computation at a time; native input continues while it runs.

    The owning loop does not call model/planner methods until this completes.
    Sending still requires the original Grip epoch and fresh hardware feedback.
    Cancellation joins the worker before shutdown can release its ownership.
    """
    operation = asyncio.create_task(asyncio.to_thread(planner.step, list(target), dt))
    try:
        return await asyncio.shield(operation)
    except asyncio.CancelledError:
        with suppress(Exception):
            await operation
        raise


class TrackingMonitor:
    """Match feedback against recent commands; distinguish delay from a fault."""
    def __init__(self, config):
        self.c = config
        self.commands = deque()
        self.bad_since = None
        self.latest_error = self.aligned_error = 0.

    def command(self, stamp, q):
        self.commands.append((stamp, np.array(q, dtype=float).copy()))
        while len(self.commands) > 1 and self.commands[1][0] < stamp-.4:
            self.commands.popleft()

    def observe(self, stamp, q):
        if not self.commands:
            return
        q = np.array(q)
        self.latest_error = float(np.max(np.abs(q-self.commands[-1][1])))
        recent = [v for t, v in self.commands if stamp-.2 <= t <= stamp+.01]
        if not recent:
            recent = [self.commands[-1][1]]
        self.aligned_error = min(float(np.max(np.abs(q-v))) for v in recent)
        if self.latest_error > self.c.joint_tracking_error_deg:
            raise RuntimeError("Robot lag exceeds joint tracking error threshold")
        if self.aligned_error > min(3., self.c.joint_tracking_error_deg*.5):
            if self.bad_since is None:
                self.bad_since = stamp
            elif stamp-self.bad_since >= .3:
                raise RuntimeError("Persistent joint tracking error; release Grip and inspect feedback")
        else:
            self.bad_since = None


class JointStream:
    """Controller-planned joint following on the shared ordered connection."""
    def __init__(self, rpc, recorder, minimum_period_s=.02):
        if not math.isfinite(minimum_period_s) or not .02 <= minimum_period_s <= .05:
            raise ValueError('Joint stream period must match a 20-50 Hz control rate')
        self.rpc, self.recorder = rpc, recorder
        self.minimum_period_s = minimum_period_s
        self.sent = 0
        self.periods = []
        self.last_sent_at = None

    async def send(self, q, permit):
        if self.last_sent_at is not None:
            remaining = self.minimum_period_s-(time.monotonic()-self.last_sent_at)
            if remaining > 0:
                await asyncio.sleep(remaining)
        # The transport revalidates the original permit after this wait.
        await self.rpc.send_joint(q, permit)
        self.sent += 1
        sent_at = time.monotonic()
        if self.last_sent_at is not None:
            self.periods.append(sent_at-self.last_sent_at)
        self.last_sent_at = sent_at
        self.recorder.event("joint_sent", joint_deg=[round(float(v), 3) for v in q],
                            interface="movej_follow")

    def check(self):
        self.rpc.check()

    async def close(self):
        # Revoke target generation; keep TCP open so STOP follows all old targets
        # in the same ordered byte stream. The RPC closes after stop readback.
        self.last_sent_at = None


async def read_gripper(robot):
    """Read health only; writes belong exclusively to GripperTransport."""
    command = "get_gripper_state"
    reader, writer = await asyncio.wait_for(asyncio.open_connection(robot.robot_ip, robot.robot_port), .2)
    try:
        writer.write(json.dumps(dict(command=command)).encode()+b"\r\n")
        await asyncio.wait_for(writer.drain(), .15)
        deadline = time.monotonic()+.4
        while time.monotonic() < deadline:
            line = await asyncio.wait_for(reader.readline(), max(.001, deadline-time.monotonic()))
            if not line:
                raise RuntimeError("Gripper socket disconnected")
            response = json.loads(line)
            if "gripper_state" in response or response.get("command") == command:
                return response.get("gripper_state", response)
        raise RuntimeError("Gripper response not recognized")
    finally:
        writer.close()
        with suppress(Exception):
            await asyncio.wait_for(writer.wait_closed(), .1)


def check_gripper(state):
    # Verified endpoint telemetry: open can read 1001 and closed -1, error=0.
    # An opening transition also reports mode=5 while actpos still reads -1.
    # Position/state updates therefore cannot be assumed to change atomically.
    # Allow the corresponding idle endpoint, and either endpoint while moving
    # or stopped mid-action, a five-count telemetry tolerance;
    # preserve raw feedback and never widen commanded position/force limits.
    mode = state.get('mode') if isinstance(state, dict) else None
    minimum = -5 if type(mode) is int and mode in (2, 3, 4, 5, 6) else 0
    maximum = 1005 if type(mode) is int and mode in (1, 3, 4, 5, 6) else 1000
    if (not isinstance(state, dict) or state.get("enable_state", state.get("enable")) != 1
            or state.get("status") != 1 or state.get("error") != 0
            or type(state.get("actpos")) is not int or not minimum <= state["actpos"] <= maximum):
        raise ValueError("Gripper unavailable / disabled / error; check end-effector connection")


async def run_follow(robot, config, output, *, direct=False, seconds=None):
    config.validate()
    recorder = BufferedRecorder(output)
    rpc, gate = FollowRPC(robot, direct, recorder), PoseGate(config)
    stream = JointStream(rpc, recorder, 1/config.control_hz)
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        loop.add_signal_handler(sig, stop.set)
    source = native_task = feedback = profile_task = gripper_task = gripper_read = lock = None
    gripper_lane = GripperTransport(robot, config, execute=direct, recorder=recorder)
    gripper_action = None
    report = dict(mode="direct" if direct else "preview", control_version=CONTROL_VERSION,
                  actual_hardware_acceptance=False,
                  motion_commands_sent=0, gripper_commands_acknowledged=0,
                  config=asdict(config), provenance=code_provenance())
    try:
        lock = (ROOT/f".realman-{robot.robot_ip}-{robot.robot_port}.lock").open("a+")
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        prepared = await preflight(rpc)
        recorder.event("preflight", **prepared, follow_config=asdict(config),
                       provenance=report['provenance'])
        model = RealManModel(*prepared["bounds"])
        state = prepared["anchor"]
        model_pose = model.fk(state["joint_deg"])
        error_mm = float(np.linalg.norm(model_pose[:3]-state["tcp_pose_m_rad"][:3])*1000)
        angle_error = (Rotation.from_euler("xyz", model_pose[3:])*Rotation.from_euler("xyz", state["tcp_pose_m_rad"][3:]).inv()).magnitude()
        if error_mm > 5. or angle_error > math.radians(2):
            raise RuntimeError(f"RM65 model / actual TCP mismatch: {error_mm:.2f} mm, {math.degrees(angle_error):.2f} deg")
        model.check(state["joint_deg"])
        planner = JointPlanner(config, model, state["joint_deg"], prepared["bounds"])
        tracking = TrackingMonitor(config)
        latest = await rpc.snapshot()
        gripper_state, gripper_stamp, gripper_reason = None, -math.inf, "disabled by configuration"
        if config.gripper_enabled:
            try:
                gripper_state = await read_gripper(robot)
                # A disabled gripper still returned a measured state. Keep its
                # receipt time even when it cannot authorize gripper commands.
                gripper_stamp = time.monotonic()
                check_gripper(gripper_state)
                gripper_reason = "ready"
            except Exception as error:
                gripper_reason = str(error)
                print("夹爪暂不可用，机械臂跟随可用："+gripper_reason, flush=True)
        gripper_available = gripper_reason == "ready"
        recorder.event("kinematics", sdk_version=model.version, fk_error_mm=error_mm,
                       fk_angle_error_deg=math.degrees(angle_error), gripper_reason=gripper_reason)
        from xrobotoolkit_input import NativeInput
        source = NativeInput(grip_threshold=.7)
        source.defer_close = True  # Persist logs before foreign SDK cleanup.
        source.start()
        # SDK/gripper startup can take longer than feedback
        # freshness. Never reuse the old preflight measurement as live feedback.
        checked = await rpc.profile()
        validate_profile(robot, checked)
        latest = await rpc.snapshot()
        if np.max(np.abs(np.array(latest["state"]["joint_deg"])-state["joint_deg"])) > .05:
            raise RuntimeError("Robot moved during startup; restart from a stationary pose")
        model_pose = model.fk(latest["state"]["joint_deg"])
        planner.reset(latest["state"]["joint_deg"])
        planner.observe(latest["state"]["joint_deg"], latest["received_at"])
        native_task = asyncio.create_task(source.pump(gate, lambda: model_pose.tolist()))
        print(f"{'真机' if direct else '只读预览'} 6D跟随 | 平移比例 {config.position_scale} | 转腕比例 {config.rotation_scale:g} | {config.control_hz:g} Hz | 范围半径 {config.workspace_radius_mm:g} mm", flush=True)
        print(f"{CONTROL_VERSION} | movej_follow | TCP上限 {config.tcp_speed_mm_s:g} mm/s | 关节上限 {config.joint_speed_deg_s:g} deg/s | 独立松爪和刹车预检开启", flush=True)
        print('方向：Base XYZ <- Quest '+config.axis_map+
              f'，Base Z 轴偏航修正 {config.axis_yaw_deg:g}°；每次运行须先用预览核对方向', flush=True)
        print("握住右 Grip：平移 / 转动；松开：停止。握 Grip 时按 Trigger：夹取；松 Trigger：张开。Ctrl+C 退出。", flush=True)
        print(f"记录目录：{output}", flush=True)
        recorder.event("started", direct=direct, gripper_available=gripper_available)
        start = deadline = time.monotonic()
        last_profile = last_print = last_gripper = -math.inf
        processed_feedback_stamp = None
        profile_stamp = checked["_started_at"]
        seen_epoch = gate.epoch
        active_before = False
        last_plan_reason = ''
        unchanged_since = None
        measured_q_reference = np.array(state["joint_deg"])
        async def poll_feedback():
            # Keep readback alive while the offline IK worker is running.
            # Only this immutable snapshot reference is replaced here; planner
            # and tracking state are updated exclusively by the control loop.
            nonlocal latest
            while True:
                latest = await rpc.snapshot()
                await asyncio.sleep(.05)

        def record_observation(snapshot):
            measured_at = time.monotonic()
            recorder.event("observation", query_started=snapshot["received_at"],
                           query_finished=measured_at, robot=snapshot["state"],
                           trajectory=snapshot["trajectory"], hand=gate.hand,
                           hand_received=gate.received, active=gate.active,
                           target_m_rad=gate.target, command_joint_deg=planner.q.tolist(),
                           tracking_error_deg=tracking.latest_error,
                           tracking_aligned_error_deg=tracking.aligned_error,
                           feedback_speed_scale=planner.feedback_scale,
                           trajectory_scale=planner.trajectory_scale,
                           planner_limiting_reason=planner.limiting_reason,
                           planner_component=planner.component,
                           native_frames=source.frames, native_device=getattr(source, 'device', None),
                           native_last_error=getattr(source, 'last_error', None),
                           gripper=gripper_state,
                           gripper_last_command=gripper_lane.last_command,
                           gripper_last_command_at=gripper_lane.last_sent_at,
                           gripper_age_s=(measured_at-gripper_stamp if gripper_state else None))
        while not stop.is_set() and time.monotonic()-start < (seconds or config.max_duration_s):
            now = time.monotonic()
            recorder.check()
            native_task.result() if native_task.done() else None
            stream.check()
            gripper_lane.check()
            if feedback is None:
                feedback = asyncio.create_task(poll_feedback())
            if feedback.done():
                feedback.result()
            if latest["received_at"] != processed_feedback_stamp:
                processed_feedback_stamp = latest["received_at"]
                q = np.array(latest["state"]["joint_deg"])
                low, high = (np.array(v) for v in prepared["bounds"])
                if np.any(q < low+config.joint_margin_deg) or np.any(q > high-config.joint_margin_deg):
                    raise RuntimeError("Measured joint at joint limit margin")
                model_pose = model.fk(q)
                planner.check_pose(model_pose)
                planner.observe(q, latest["received_at"])
                if direct and active_before:
                    tracking.observe(latest["received_at"], q)
                if direct and active_before and np.max(np.abs(q-planner.q)) > .3:
                    if np.max(np.abs(q-measured_q_reference)) > .05:
                        unchanged_since = now
                        measured_q_reference = q.copy()
                    elif unchanged_since is None:
                        unchanged_since = now
                    elif now-unchanged_since > 1.:
                        raise RuntimeError("No measured joint response to stream; check controller following compatibility")
                else:
                    unchanged_since = None
                    measured_q_reference = q.copy()
                record_observation(latest)
            if profile_task is None and now-last_profile >= .2:
                profile_task = asyncio.create_task(rpc.profile())
                last_profile = now
            if profile_task is not None and profile_task.done():
                p = profile_task.result()
                validate_profile(robot, p)
                profile_stamp = p["_started_at"]
                profile_task = None
            if now-latest["received_at"] > robot.feedback_timeout_s or now-profile_stamp > .5:
                raise RuntimeError("Robot feedback or hardware profile stale: "
                                   f"feedback_age={now-latest['received_at']:.3f}s, "
                                   f"profile_age={now-profile_stamp:.3f}s")
            gate.watchdog(now)
            if now-deadline > .08:
                gate.trip("control scheduler delayed; release Grip")
            if gate.epoch != seen_epoch:
                seen_epoch = gate.epoch
                interlock = gate.pending_interlock or dict(reason=gate.reason)
                gate.pending_interlock = None
                recorder.event("interlock", **interlock)
                print("已暂停跟随："+interlock['reason'], flush=True)
                await stream.close()
                if direct and rpc.motion_attempts:
                    await controlled_stop(rpc, recorder, observer=record_observation)
                if gripper_task is not None:
                    repeat_open = (gripper_action == 'open' and gate.gripper_latched == 'open')
                    gripper_task.cancel()
                    with suppress(asyncio.CancelledError, MotionPermitExpired):
                        await gripper_task
                    gripper_task = None
                    gripper_action = None
                    if repeat_open and gate.gripper_intent_stamp is not None:
                        gate.gripper_edge = ('open', gate.gripper_intent_stamp, gate.gripper_generation)
                        recorder.event('gripper_requeued_after_arm_stop', action='open')
                for pending in (feedback, profile_task):
                    if pending is not None:
                        pending.cancel()
                        with suppress(asyncio.CancelledError, Exception):
                            await pending
                feedback = profile_task = None
                latest = await rpc.snapshot()
                p = await rpc.profile()
                validate_profile(robot, p)
                profile_stamp = p["_started_at"]
                model_pose = model.fk(latest["state"]["joint_deg"])
                planner.reset(latest["state"]["joint_deg"])
                planner.observe(latest["state"]["joint_deg"], latest["received_at"])
                tracking = TrackingMonitor(config)
                gate.trip_motion("stopped; release Grip and squeeze to resume")
                gate.pending_interlock = None
                seen_epoch, active_before, last_plan_reason = gate.epoch, False, ''
                deadline = time.monotonic()
                continue
            if gripper_task is not None and gripper_task.done():
                try:
                    reply = gripper_task.result()
                except MotionPermitExpired as error:
                    recorder.event("gripper_cancelled", reason=str(error))
                else:
                    report["gripper_commands_acknowledged"] += 1
                    recorder.event("gripper_acknowledged", reply=reply)
                gripper_task = None
                gripper_action = None
            if gripper_available and gripper_read is None and now-last_gripper > .2:
                gripper_read = asyncio.create_task(read_gripper(robot))
                last_gripper = now
            if gripper_read is not None and gripper_read.done():
                gripper_state = gripper_read.result()
                check_gripper(gripper_state)
                gripper_stamp = time.monotonic()
                gripper_read = None
            def hardware_permit():
                t = time.monotonic()
                return (not stop.is_set() and gate.fresh(t)
                        and feedback is not None and not feedback.done()
                        and t-latest["received_at"] <= robot.feedback_timeout_s
                        and t-profile_stamp <= .5)
            def permit(epoch=gate.epoch):
                return hardware_permit() and gate.active and gate.epoch == epoch
            # Release is independent of arm planning. A fresh open intent may
            # supersede a close ACK waiter; writes retain order on their lane.
            if gate.gripper_edge:
                action, stamp, generation = gate.gripper_edge
                if action == 'open' and gripper_task is not None and gripper_action == 'close':
                    gripper_task.cancel()
                    with suppress(asyncio.CancelledError, MotionPermitExpired):
                        await gripper_task
                    gripper_task = None
                    gripper_action = None
                    recorder.event('gripper_close_superseded_by_open')
                if gripper_task is None:
                    if now-stamp >= 1. or generation != gate.gripper_generation:
                        gate.gripper_edge = None
                        recorder.event('gripper_intent_expired', action=action)
                    elif direct and gripper_available and now-gripper_stamp < .5:
                        gate.gripper_edge = None
                        command = 'set_gripper_pick' if action == 'close' else 'set_gripper_release'
                        recorder.event('gripper_requested', action=action, command=command,
                                       input_received=stamp, dispatch_delay_s=max(0., now-stamp))
                        def gripper_permit(generation=generation, action=action):
                            return (hardware_permit() and time.monotonic()-gripper_stamp < .5
                                    and gate.hand['grip'] >= .7
                                    and generation == gate.gripper_generation
                                    and gate.gripper_latched == action
                                    and (action == 'open' or gate.active))
                        gripper_task = asyncio.create_task(gripper_lane.send(command, gripper_permit))
                        gripper_action = action
                    elif not direct:
                        gate.gripper_edge = None
            if gate.active and gate.target is not None:
                if not active_before:
                    planner.reset(latest["state"]["joint_deg"])
                    planner.observe(latest["state"]["joint_deg"], latest["received_at"])
                    tracking = TrackingMonitor(config)
                if now >= deadline:
                    planned_target = gate.target[:]
                    try:
                        q, pose = await plan_step(planner, planned_target, 1/config.control_hz)
                    except ValueError as error:
                        gate.trip_motion(str(error))
                        continue
                    if planner.limiting_reason != last_plan_reason:
                        recorder.event('planner_limited' if planner.limiting_reason else 'planner_resumed',
                                       reason=planner.limiting_reason, scale=planner.trajectory_scale,
                                       component=planner.component,
                                       requested_delta_mm=((np.array(planned_target[:3])-pose[:3])*1000).tolist(),
                                       joint3_deg=float(q[2]), joint5_deg=float(q[4]))
                        if planner.limiting_reason:
                            print('规划减速/保持：'+planner.limiting_reason+'；Grip保持有效，可退回可行方向', flush=True)
                        last_plan_reason = planner.limiting_reason
                    if direct:
                        try:
                            await stream.send(q, permit)
                        except MotionPermitExpired:
                            if gate.active:
                                reason = ("no fresh valid right-hand data; release Grip"
                                          if not gate.fresh(time.monotonic()) else
                                          "motion authorization changed before send; release Grip")
                                gate.trip(reason)
                            continue
                        tracking.command(time.monotonic(), q)
                    else:
                        recorder.event("preview_joint", joint_deg=q.tolist(), tcp_model_m_rad=pose.tolist())
                    # Keep an absolute cadence; do not add planner/I/O time to every period.
                    deadline += 1/config.control_hz
                    if deadline < time.monotonic():
                        deadline = time.monotonic() + 1/config.control_hz
            else:
                deadline = now
            active_before = gate.active
            if now-last_print >= .5:
                print(f"{'FOLLOW' if gate.active else 'WAIT'} {gate.reason} R={gate.fresh(time.monotonic())} "
                      f"sent={stream.sent} Grip={gate.hand['grip'] if gate.hand else None} "
                      f"gripper={'ready' if gripper_available else 'unavailable'} "
                      f"lead={tracking.latest_error:.2f}deg rate={planner.feedback_scale:.0%} "
                      f"plan={planner.limiting_reason or 'normal'} "
                      f"actual_xyz={latest['state']['tcp_pose_m_rad'][:3]} "
                      f"target_xyz={gate.target[:3] if gate.target else None} "
                      f"target_rpy={gate.target[3:] if gate.target else None}", flush=True)
                last_print = now
            await asyncio.sleep(.002)
        report["status"] = "stopped_by_operator_or_duration"
    except BaseException as error:
        report.update(status="failed", error=f"{type(error).__name__}: {error}")
        raise
    finally:
        gate.trip("shutdown")
        # Revoke sends and stop the arm before SDK Deinit; native cleanup can
        # block in foreign code and must not delay the physical stop request.
        for pending in (feedback, profile_task, gripper_task, gripper_read):
            if pending is not None:
                pending.cancel()
        await stream.close()
        if direct and rpc.motion_attempts:
            try:
                await controlled_stop(rpc, recorder, observer=record_observation)
                report["stop_verified"] = True
            except Exception as error:
                report.update(status="stop_unverified", stop_verified=False, stop_error=str(error))
        for pending in (native_task, feedback, profile_task, gripper_task, gripper_read):
            if pending is not None:
                pending.cancel()
                with suppress(asyncio.CancelledError, Exception):
                    await pending
        await rpc.close()
        await gripper_lane.close()
        report.update(motion_interface="movej_follow", transport_connections=rpc.connections_opened,
                      motion_send_attempts=rpc.motion_attempts, motion_commands_sent=stream.sent,
                      stop_commands_sent=rpc.stop_attempts,
                      native_frames=getattr(source, "frames", 0),
                      native_valid_tracking_frames=getattr(source, "tracking_frames", 0),
                      native_last_error=getattr(source, "last_error", None))
        if stream.periods:
            report["active_send_interval_ms"] = dict(median=float(np.median(stream.periods)*1000),
                                                      maximum=max(stream.periods)*1000)
        recorder.event("finished", **report)
        report["event_recording"] = await asyncio.to_thread(recorder.close)
        from follow_recording import write_json
        report['native_cleanup'] = 'pending' if source else 'not_started'
        write_json(output/'report.json', report)
        try:
            if source:
                source.close()
                report['native_cleanup'] = 'complete'
        except Exception as error:
            report['native_cleanup'] = 'failed'
            report['native_cleanup_error'] = f'{type(error).__name__}: {error}'
        finally:
            write_json(output/'report.json', report)
            if lock:
                lock.close()
        print(json.dumps(report, ensure_ascii=False), flush=True)
        if report.get("stop_verified") is False:
            raise RuntimeError(report["stop_error"])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--direct", action="store_true", help="直接真机运行；默认仅预览")
    p.add_argument("--config", type=Path, default=ROOT/"config/follow.json")
    p.add_argument("--robot-config", type=Path, default=ROOT/"config/robot.json")
    p.add_argument("--seconds", type=float)
    p.add_argument("--output", type=Path)
    a = p.parse_args()
    c, robot = FollowConfig.load(a.config), Config.load(a.robot_config)
    if a.direct and not c.direction_configured:
        p.error("先运行 bash robot.sh direction 手动设置方向，再用 preview 核对")
    if a.seconds is not None and (not math.isfinite(a.seconds) or not 0 < a.seconds <= c.max_duration_s):
        p.error("seconds exceeds configured duration")
    out = a.output or ROOT/"run"/datetime.now().strftime("follow_%Y%m%dT%H%M%S_%f")
    asyncio.run(run_follow(robot, c, out, direct=a.direct, seconds=a.seconds))


if __name__ == "__main__":
    try:
        main()
    except (Exception, KeyboardInterrupt) as error:
        raise SystemExit(str(error))
