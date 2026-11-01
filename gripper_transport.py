"""Ordered gripper command lane, separate from arm motion and stop queries.

Nonblocking controller commands are pipelined on one TCP connection so a fresh
open intent can follow a slow close ACK without waiting or overtaking it.
No automatic reconnect, retry, configuration, or arm motion commands.
"""
import asyncio
from collections import deque
from contextlib import suppress
import json
import time

from robot_core import MotionPermitExpired


class GripperTransport:
    def __init__(self, robot, config, execute=False, recorder=None):
        self.robot, self.config, self.execute = robot, config, execute
        self.writer = self.reader_task = None
        self.connect_lock, self.write_lock = asyncio.Lock(), asyncio.Lock()
        self.pending = deque()
        self.failure = None
        self.closing = False
        self.recorder = recorder
        self.last_command = self.last_sent_at = None

    def check(self):
        if self.failure:
            raise self.failure
        if self.closing:
            raise RuntimeError('Gripper transport closed')

    def _fail(self, error):
        if self.failure is None:
            self.failure = RuntimeError(str(error))
        if self.writer:
            self.writer.transport.abort()
        for future in self.pending:
            if not future.done():
                future.set_exception(self.failure)

    async def _connect(self):
        self.check()
        async with self.connect_lock:
            if self.writer is None:
                self.check()
                try:
                    reader, self.writer = await asyncio.wait_for(asyncio.open_connection(
                        self.robot.robot_ip, self.robot.robot_port), .2)
                    self.reader_task = asyncio.create_task(self._read(reader))
                except Exception as error:
                    self._fail(error)
                    raise

    async def _read(self, reader):
        try:
            while True:
                line = await reader.readline()
                if not line:
                    raise RuntimeError('Gripper command connection disconnected')
                reply = json.loads(line)
                if (reply.get('command') not in ('set_gripper', 'set_gripper_pick', 'set_gripper_release')
                        or type(reply.get('state')) is not bool):
                    continue
                if not self.pending:
                    continue
                future = self.pending.popleft()
                if not future.done():
                    if reply['state']:
                        future.set_result(reply)
                    else:
                        future.set_exception(RuntimeError('Gripper command rejected'))
        except asyncio.CancelledError:
            raise
        except Exception as error:
            if not self.closing:
                self._fail(error)

    async def send(self, command, permit):
        if not self.execute or command not in ('set_gripper_pick', 'set_gripper_release'):
            raise RuntimeError('Gripper command forbidden in this mode')
        await self._connect()
        async with self.write_lock:
            self.check()
            if not permit():
                raise MotionPermitExpired('Gripper intent expired before write')
            if len(self.pending) >= 4 or self.writer.transport.get_write_buffer_size() > 2048:
                raise RuntimeError('Gripper command backlog')
            payload = dict(command=command, speed=self.config.gripper_speed, block=False)
            if command == 'set_gripper_pick':
                payload['force'] = self.config.gripper_force
            future = asyncio.get_running_loop().create_future()
            future.add_done_callback(lambda done: done.exception() if not done.cancelled() else None)
            self.pending.append(future)
            # A cancelled close caller leaves its FIFO slot until the old ACK
            # is consumed; it must never satisfy a newer open request.
            timer = asyncio.get_running_loop().call_later(
                .6, lambda: self._fail('Gripper command acknowledgement timed out')
                if not future.done() else None)
            future.add_done_callback(lambda _: timer.cancel())
            self.writer.write(json.dumps(payload).encode()+b'\r\n')
            try:
                await asyncio.wait_for(self.writer.drain(), .02)
            except Exception as error:
                self._fail(error)
                raise
            self.last_command, self.last_sent_at = dict(payload), time.monotonic()
            if self.recorder:
                self.recorder.event('gripper_sent', request=payload, sent_at=self.last_sent_at)
        return await asyncio.shield(future)

    async def close(self):
        self.closing = True
        if self.reader_task:
            self.reader_task.cancel()
            with suppress(asyncio.CancelledError):
                await self.reader_task
        for future in self.pending:
            future.cancel()
        self.pending.clear()
        if self.writer:
            self.writer.close()
            with suppress(Exception):
                await asyncio.wait_for(self.writer.wait_closed(), .1)
