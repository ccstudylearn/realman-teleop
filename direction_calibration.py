"""Rotation-only, camera-screen direction mapping and a read-only hand gesture."""
import numpy as np
from scipy.spatial.transform import Rotation


def source_basis(right):
    right = np.array(right, dtype=float)
    if right.shape != (3,) or not np.isfinite(right).all():
        raise ValueError('Invalid VR right direction')
    right[1] = 0.
    length = np.linalg.norm(right)
    if length < .01:
        raise ValueError('Right gesture has no horizontal displacement')
    right /= length
    up = np.array([0., 1., 0.])
    forward = np.cross(up, right)
    return np.column_stack((right, forward, up))


def proper_rotation(value, label):
    matrix = np.array(value, dtype=float)
    if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
            or not np.allclose(matrix.T@matrix, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(matrix), 1., atol=1e-5)):
        raise ValueError(label+' must be a proper rotation')
    return matrix


def camera_target_basis(robot_euler, calibration):
    """Camera right projected onto the table, table forward, physical Base up."""
    mount = proper_rotation(calibration['camera_rotation_tool'], 'Camera mount')
    right = (Rotation.from_euler('xyz', robot_euler).as_matrix()@mount)[:, 0].copy()
    right[2] = 0.
    if np.linalg.norm(right) < .1:
        raise ValueError('Camera image-right is nearly vertical; choose another wrist pose')
    right /= np.linalg.norm(right)
    up = np.array([0., 0., 1.])
    forward = np.cross(up, right)
    return np.column_stack((right, forward, up))


def camera_axis_matrix(robot_euler, calibration):
    # Old recordings retain their original per-clutch camera reference. New
    # calibrations explicitly freeze the basis; rotating the wrist/re-gripping
    # must not silently change the operator's translation directions.
    proper_rotation(calibration['camera_rotation_tool'], 'Camera mount')
    if 'target_basis_base' in calibration:
        target = proper_rotation(calibration['target_basis_base'], 'Fixed target basis')
        if not np.allclose(target[:, 2], [0., 0., 1.], atol=1e-5):
            raise ValueError('Fixed target up must be Base +Z')
    else:
        target = camera_target_basis(robot_euler, calibration)
    return target@source_basis(calibration['vr_right_axis']).T


def verify_direction_gestures(deltas):
    """Check labelled right/up/forward recordings before changing a mapping."""
    deltas = np.array(deltas, dtype=float)
    if deltas.shape != (3, 3) or not np.isfinite(deltas).all():
        raise ValueError('Expected three finite right/up/forward displacements')
    lengths = np.linalg.norm(deltas, axis=1)
    if np.any(lengths < .05) or np.any(lengths > .4):
        raise ValueError('Each direction gesture must be 5–40 cm')
    basis = source_basis(deltas[0])
    expected = basis[:, [0, 2, 1]].T
    cosines = np.sum(deltas/lengths[:, None]*expected, axis=1)
    errors = np.rad2deg(np.arccos(np.clip(cosines, -1., 1.)))
    if np.any(errors > 20.):
        raise ValueError('Right/up/forward directions disagree: angular errors '+str(errors.round(1).tolist()))
    return basis[:, 0].tolist(), errors.tolist()


class AxisGestures:
    """Three labelled gestures in one uninterrupted read-only connection."""
    labels = ('向右平移', '竖直抬高', '向身体正前方水平推')

    def __init__(self, third_direction='forward'):
        if third_direction not in ('forward', 'backward'):
            raise ValueError('Third direction must be forward or backward')
        self.third_direction = third_direction
        if third_direction == 'backward':
            self.labels = ('向右平移', '竖直抬高', '朝身体方向水平拉回')
        self.ready = False
        self.samples = []
        self.groups = []
        self.result = self.error = None
        self.angular_errors_deg = None

    @property
    def stage(self):
        return self.labels[min(len(self.groups), 2)]

    def trip(self, reason):
        if self.samples and self.result is None:
            self.error = str(reason)
        self.ready = False

    def observe(self, hand, now):
        if self.result is not None or self.error:
            return
        if not hand['valid']:
            self.trip('Tracking lost during '+self.stage)
            return
        if not self.ready:
            self.ready = hand['grip'] < .2
            return
        if hand['grip'] >= .7:
            if self.samples and now-self.samples[-1][0] > .1:
                self.trip('Input gap during '+self.stage)
                return
            self.samples.append((now, list(hand['position_m'])))
        elif self.samples and hand['grip'] < .2:
            if len(self.samples) < 10 or self.samples[-1][0]-self.samples[0][0] < .3:
                self.samples = []  # A wake-up squeeze is not a calibration gesture.
                return
            points = np.array([p for _, p in self.samples])
            delta = np.median(points[-5:], axis=0)-np.median(points[:5], axis=0)
            length = np.linalg.norm(delta)
            if length < .05:
                self.samples = []
                return
            if length > .4:
                self.error = 'Move 5–40 cm and release at endpoint during '+self.stage
                return
            progress = (points-points[0])@(delta/length)
            if np.max(progress)-progress[-1] > .025:
                self.error = 'Release Grip before returning during '+self.stage
                return
            self.groups.append(dict(label=self.stage, samples=self.samples, delta_m=delta.tolist()))
            self.samples = []
            if len(self.groups) == 3:
                try:
                    deltas = [g['delta_m'] for g in self.groups]
                    if self.third_direction == 'backward':
                        deltas[-1] = (-np.array(deltas[-1])).tolist()
                    self.result, self.angular_errors_deg = verify_direction_gestures(
                        deltas)
                except ValueError as error:
                    self.error = str(error)


class RightGesture:
    """Release, Grip, move right once, release. Never connects to a robot."""
    def __init__(self):
        self.ready = False
        self.samples = []
        self.result = None
        self.error = None

    def trip(self, reason):
        if self.samples and self.result is None:
            self.error = str(reason)
        self.ready = False

    def observe(self, hand, now):
        if self.result is not None or self.error:
            return
        if not hand['valid']:
            self.trip('Right controller tracking invalid during direction capture')
            return
        if not self.ready:
            self.ready = hand['grip'] < .2
            return
        if hand['grip'] >= .7:
            self.samples.append((now, list(hand['position_m'])))
        elif self.samples and hand['grip'] < .2:
            if len(self.samples) < 10 or self.samples[-1][0]-self.samples[0][0] < .3:
                self.error = 'Right gesture too short; repeat a 10 cm rightward movement'
                return
            points = np.array([p for _, p in self.samples])
            delta = np.median(points[-5:], axis=0)-np.median(points[:5], axis=0)
            horizontal = delta.copy(); horizontal[1] = 0.
            length = np.linalg.norm(horizontal)
            if not .05 <= length <= .4 or abs(delta[1]) > .4*length:
                self.error = 'Move 5–40 cm horizontally right, without lifting or returning'
                return
            axis = horizontal/length
            progress = (points-points[0])@axis
            if np.max(progress)-progress[-1] > .025:
                self.error = 'Release Grip at the right endpoint before returning your hand'
                return
            self.result = axis.tolist()
