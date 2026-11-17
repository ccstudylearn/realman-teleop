"""Offline regression: synthetic input, configuration and mock robot feedback.

Run: .venv/bin/python -m unittest -v test_offline_sync
No robot, camera, ADB, native SDK startup or network connection is used.
"""
from contextlib import redirect_stdout
from dataclasses import replace
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import AsyncMock, patch

import numpy as np
from scipy.spatial.transform import Rotation

from direction_calibration import camera_axis_matrix
from realman_follow import FollowConfig, PoseGate, axis_matrix, code_provenance
from robot_core import ArmRPC, Config, controlled_stop, decode_state
from xrobotoolkit_input import NativeInput, frame_clock
import teleop


def packet(stamp=1_000_000, utc=1_800_000_000_000_000_000, grip=1.):
    return dict(timeStampNs=utc, predictTime=stamp, inputProtocol='realman-quest-v1',
                appState=dict(focus=True, userPresent=True),
                Controller=dict(right=dict(pose=[0., 0., 0., 0., 0., 0., 1.],
                    poseValid=True, deviceValid=True, isTracked=True, controllerDevice=True,
                    trackingState=3, primaryButton=False, secondaryButton=False,
                    trigger=0., grip=grip)))


def calibration():
    return dict(camera_serial='TEST_WRIST', camera_rotation_tool=np.eye(3).tolist(),
                target_basis_base=np.eye(3).tolist(), vr_right_axis=[1., 0., 0.])


class InputTests(unittest.TestCase):
    def drain(self, source, data, arrival=10., raw_ns=20_000_000_000, now=None):
        source.items.put_nowait((arrival, raw_ns, json.dumps(data).encode()))
        observed, faults = [], []
        with patch('xrobotoolkit_input.time.monotonic', return_value=arrival if now is None else now):
            source.drain(lambda hand, stamp: observed.append(hand), faults.append)
        return observed, faults

    def test_frame_clock_and_legacy(self):
        self.assertEqual(frame_clock(packet()), ('unity_unscaled', 1_000_000_000))
        self.assertEqual(frame_clock(dict(timeStampNs=123)), ('utc_legacy', 123))
        for value in (True, -1, float('nan'), float('inf'), '1'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                frame_clock(dict(predictTime=value))

    def test_utc_correction_does_not_trip_frame_clock(self):
        source = NativeInput()
        self.assertFalse(self.drain(source, packet())[1])
        observed, faults = self.drain(source, packet(1_010_000, utc=9_000_000_000_000_000_000),
                                      10.01, 20_010_000_000)
        self.assertFalse(faults)
        self.assertEqual(len(observed), 1)
        self.assertEqual(observed[0]['source_timestamp_ns'], 9_000_000_000_000_000_000)
        self.assertEqual(observed[0]['source_clock'], 'unity_unscaled')

    def test_duplicate_release_trips_without_refresh(self):
        source = NativeInput(grip_threshold=.7)
        self.drain(source, packet())
        observed, faults = self.drain(source, packet(grip=0.), 10.01, 20_010_000_000)
        self.assertFalse(observed)
        self.assertTrue(faults)
        self.assertEqual(source.last_arrival, 10.)

    def test_backlog_gap_and_clock_switch_fail_closed(self):
        cases = [
            (packet(1_010_000), 10.01, 20_010_000_000, 10.05),
            (packet(1_010_000), 10.01, 20_070_000_000, 10.01),
            (packet(1_200_000), 10.2, 20_200_000_000, 10.2),
        ]
        legacy = packet(1_010_000)
        del legacy['predictTime']
        cases.append((legacy, 10.01, 20_010_000_000, 10.01))
        for data, arrival, raw, now in cases:
            with self.subTest(data=data, now=now):
                source = NativeInput()
                self.drain(source, packet())
                observed, faults = self.drain(source, data, arrival, raw, now)
                self.assertFalse(observed)
                self.assertTrue(faults)


class DirectionTests(unittest.TestCase):
    def test_public_template_and_old_configuration(self):
        current = FollowConfig.load(Path(__file__).parent/'config/follow.example.json')
        self.assertEqual(current.rotation_scale, 1.)
        self.assertFalse(current.direction_configured)
        self.assertIsNone(current.direction_calibration)
        self.assertEqual(replace(current, rotation_scale=.5).validate().rotation_scale, .5)

    def test_fixed_calibration_does_not_rotate_with_wrist(self):
        c = calibration()
        np.testing.assert_allclose(camera_axis_matrix([0., 0., 0.], c),
                                   camera_axis_matrix([.3, -.2, 1.], c))
        for invalid in (dict(c, camera_rotation_tool=(-np.eye(3)).tolist()),):
            with self.assertRaises((ValueError, TypeError)):
                camera_axis_matrix([0., 0., 0.], invalid)

    def test_calibration_serial_and_manual_direction(self):
        with self.assertRaises(ValueError):
            FollowConfig(direction_calibration=calibration()).validate()
        matrix = axis_matrix('-x,z,y', 30.)
        np.testing.assert_allclose(matrix.T@matrix, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(matrix), 1.)
        for kwargs in (dict(axis_map='x,y,-z'), dict(axis_yaw_deg=float('nan'))):
            with self.assertRaises(ValueError):
                FollowConfig(**kwargs).validate()

    def test_grip_release_and_one_to_one_rotation(self):
        gate = PoseGate(FollowConfig(direction_configured=True))
        pose = [0., 0., .3, 0., 0., 0.]
        hand = dict(seq=1, valid=True, position_m=[0., 0., 0.],
                    quaternion_xyzw=[0., 0., 0., 1.], grip=1., trigger=0.)
        gate.observe(hand, 10., pose)
        self.assertTrue(gate.active)
        turn = Rotation.from_rotvec([.02, 0., 0.])
        gate.observe(dict(hand, seq=2, quaternion_xyzw=turn.as_quat()), 10.01, pose)
        self.assertAlmostEqual(Rotation.from_euler('xyz', gate.target[3:]).magnitude(), .02)
        gate.observe(dict(hand, seq=3, grip=0.), 10.02, pose)
        self.assertFalse(gate.active)
        self.assertIsNone(gate.target)

    def test_manual_direction_clears_calibration_without_hardware(self):
        current = FollowConfig(camera_serial='TEST_WRIST', direction_calibration=calibration())
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'config').mkdir()
            args = type('Args', (), dict(show=False, axis_map='-x,z,y', yaw_deg=15.))()
            with patch.object(teleop, 'ROOT', root), patch.object(teleop, 'load_config', return_value=(Config(), current)), redirect_stdout(io.StringIO()):
                teleop.direction(args)
            result = FollowConfig.load(root/'config/follow.json')
            self.assertTrue(result.direction_configured)
            self.assertIsNone(result.direction_calibration)
            self.assertEqual(result.axis_yaw_deg, 15.)

    def test_provenance_paths_exist(self):
        result = code_provenance()
        self.assertEqual(result['control_version'], 'frame-clock-wrist-v4.6')
        self.assertIn('direction_calibration.py', result['source_sha256'])


class RobotTests(unittest.IsolatedAsyncioTestCase):
    def test_fault_feedback_is_readable_only_when_explicit(self):
        reply = dict(arm_state=dict(joint=[0]*6, pose=[0]*6, err=[0x1000]))
        with self.assertRaises(ValueError):
            decode_state(reply)
        self.assertEqual(decode_state(reply, allow_fault=True)['errors'], [0x1000])

    async def test_base_transport_still_forbids_motion(self):
        for execute in (False, True):
            rpc = ArmRPC(Config(), execute=execute)
            for command in ('movej', 'movel', 'set_joint_en_state', 'set_tool_voltage'):
                with self.subTest(command=command, execute=execute), patch('asyncio.open_connection', new=AsyncMock(side_effect=AssertionError('Network used'))):
                    with self.assertRaises(RuntimeError):
                        await rpc.call(command)
        with self.assertRaises(RuntimeError):
            await ArmRPC(Config(), execute=False).call('set_arm_stop')

    async def test_stop_verifies_stationary_faulted_feedback(self):
        class MockRPC:
            stop_attempts = 0
            async def call(self, command):
                assert command == 'set_arm_stop'
                self.stop_attempts += 1
            async def snapshot(self, *, allow_fault=False):
                assert allow_fault
                return dict(state=dict(joint_deg=[0]*6, errors=[4096]), trajectory=dict(type='none'))
        events = []
        recorder = type('Recorder', (), dict(event=lambda _, name, **data: events.append((name, data))))()
        await controlled_stop(MockRPC(), recorder)
        self.assertEqual(events[-1][0], 'stop_verified')
        self.assertEqual(events[-1][1]['controller_errors'], [4096])

    async def test_stop_acknowledgement_failure_is_not_success(self):
        class MockRPC:
            stop_attempts = 0
            async def call(self, command):
                self.stop_attempts += 1
                raise RuntimeError('No ACK')
        events = []
        recorder = type('Recorder', (), dict(event=lambda _, name, **data: events.append((name, data))))()
        with self.assertRaisesRegex(RuntimeError, 'Cannot verify robot stop'):
            await controlled_stop(MockRPC(), recorder)
        self.assertEqual(events[-1][0], 'stop_unverified')


if __name__ == '__main__':
    unittest.main()
