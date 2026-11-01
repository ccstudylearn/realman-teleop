"""Ordered RealMan follow/query/stop transport, with independently routed replies.

No automatic motion retry, reconnect, enable, Home, or controller configuration.
An independent STOP/readback connection remains available after transport faults.
"""
import asyncio
from contextlib import suppress
import json
import math

from robot_core import ArmRPC, READS, MotionPermitExpired


READ_KEYS = {
    'get_current_arm_state': 'arm_state', 'get_current_work_frame': 'frame_name',
    'get_current_tool_frame': 'tool_name', 'get_arm_software_info': 'Product_version',
    'get_arm_run_mode': 'mode', 'get_collision_stage': 'collision_stage',
    'get_joint_en_state': 'en_state', 'get_joint_min_pos': 'min_pos',
    'get_joint_max_pos': 'max_pos', 'get_program_run_state': 'run_state',
    'get_arm_current_trajectory': 'type',
}


class FollowRPC(ArmRPC):
    def __init__(self, config, execute=False, recorder=None):
        super().__init__(config, execute)
        self.recorder = recorder
        self.writer = self.reader_task = None
        self.pending = {}
        self.connect_lock = asyncio.Lock()
        self.write_lock = asyncio.Lock()
        self.failure = None
        self.emergency_readback = False
        self.closing = False
        self.connections_opened = 0

    def event(self, event, **data):
        if self.recorder:
            self.recorder.event(event, **data)

    def check(self):
        if self.failure:
            raise self.failure
        if self.closing:
            raise RuntimeError('Follow transport closed')

    def _fail(self, error):
        if self.failure is None:
            self.failure = RuntimeError(str(error))
            self.event('transport_fault', error=str(error))
        if self.writer:
            self.writer.transport.abort()
        for future in list(self.pending.values()):
            if not future.done():
                future.set_exception(self.failure)

    async def _connect(self):
        self.check()
        async with self.connect_lock:
            self.check()
            if self.writer is None:
                try:
                    reader, self.writer = await asyncio.wait_for(asyncio.open_connection(
                        self.c.robot_ip, self.c.robot_port, limit=65536), .15)
                except Exception as error:
                    self._fail(error)
                    raise
                self.connections_opened += 1
                self.reader_task = asyncio.create_task(self._read(reader))
                self.event('transport_open', connections=self.connections_opened)

    @staticmethod
    def _matches(command, reply):
        if command in READ_KEYS:
            if READ_KEYS[command] not in reply:
                return False
            identifiers = {command, command.removeprefix('get_')}
            # Controllers can broadcast another connection's replies. In
            # particular, gripper "mode" is not the robot's real/sim run mode.
            for field in ('command', 'state'):
                if isinstance(reply.get(field), str) and reply[field] not in identifiers:
                    return False
            return True
        if command == 'set_arm_stop':
            return 'arm_stop' in reply
        return False

    async def _read(self, reader):
        try:
            while True:
                line = await reader.readline()
                if not line:
                    raise RuntimeError('Continuous control socket disconnected')
                reply = json.loads(line)
                if not isinstance(reply, dict):
                    raise ValueError('Controller reply must be an object')
                self.event('transport_reply', reply=reply)
                if reply.get('command') in ('movej_follow', 'movej_canfd'):
                    if reply.get('receive_state') is False or reply.get('state') is False:
                        raise RuntimeError('Controller rejected continuous control')
                    if reply.get('trajectory_state') is False or reply.get('error', 0):
                        raise RuntimeError('Controller reported a motion failure')
                for command, future in list(self.pending.items()):
                    if not future.done() and self._matches(command, reply):
                        future.set_result(reply)
                        break
        except asyncio.CancelledError:
            raise
        except Exception as error:
            if not self.closing:
                self._fail(error)

    async def _send(self, payload, permit=None):
        await self._connect()
        async with self.write_lock:
            self.check()
            if permit is not None and not permit():
                raise MotionPermitExpired('Continuous motion permit expired before write')
            if self.writer.transport.get_write_buffer_size() > 2048:
                raise RuntimeError('Continuous motion send backlog')
            name = payload['command']
            if name == 'movej_follow':
                self.motion_attempts += 1
                self.event('joint_send_attempt', request=payload)
            elif name == 'set_arm_stop':
                self.stop_attempts += 1
                self.event('stop_requested', ordered_connection=True)
            self.writer.write(json.dumps(payload).encode()+b'\r\n')
            try:
                await asyncio.wait_for(self.writer.drain(), .02)
            except Exception as error:
                self._fail(error)
                raise

    async def _request(self, payload, permit=None, timeout=.15):
        name = payload['command']
        if name in self.pending:
            await asyncio.shield(self.pending[name])
        self.check()
        future = asyncio.get_running_loop().create_future()
        self.pending[name] = future
        # A cancelled caller must not let a late reply satisfy a newer request.
        # Keep the old slot until its reply or transport timeout is consumed.
        def finished(done):
            if self.pending.get(name) is done:
                self.pending.pop(name)
            if not done.cancelled():
                done.exception()
        future.add_done_callback(finished)
        try:
            await self._send(payload, permit)
        except BaseException:
            self.pending.pop(name, None)
            future.cancel()
            raise
        timer = asyncio.get_running_loop().call_later(
            timeout, lambda: self._fail('Controller request timeout: '+name)
            if not future.done() else None)
        future.add_done_callback(lambda _: timer.cancel())
        return await asyncio.shield(future)

    async def _exchange(self, command, payload=None, permit=None):
        if command not in READS and not (self.execute and command == 'set_arm_stop'):
            raise RuntimeError('Command forbidden by follow transport: '+command)
        if command in READS and self.emergency_readback:
            return await super()._exchange(command, payload, permit)
        if command == 'set_arm_stop':
            try:
                reply = await self._request(dict(command=command))
                if reply.get('arm_stop') is not True:
                    raise RuntimeError('Stop was not acknowledged')
                return reply
            except Exception as error:
                self.event('stop_fallback', error=str(error))
                self._fail(error)
                self.emergency_readback = True
                return await super()._exchange(command, payload, permit)
        return await self._request(dict(command=command, **(payload or {})))

    async def send_joint(self, joint_deg, permit):
        if not self.execute:
            raise RuntimeError('Preview cannot send motion')
        if len(joint_deg) != 6 or not all(math.isfinite(v) for v in joint_deg):
            raise ValueError('Expected six finite joint angles')
        await self._send(dict(command='movej_follow',
                              joint=[round(float(v)*1000) for v in joint_deg]), permit)

    async def close(self):
        self.closing = True
        if self.writer:
            self.writer.transport.abort()
            self.writer.close()
            with suppress(Exception):
                await asyncio.wait_for(self.writer.wait_closed(), .1)
        if self.reader_task:
            self.reader_task.cancel()
            with suppress(asyncio.CancelledError, Exception):
                await self.reader_task
        for future in list(self.pending.values()):
            if not future.done():
                future.cancel()
        self.pending.clear()
