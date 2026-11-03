"""Controller-button episode boundaries; robot and camera connections stay alive."""
import asyncio
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from episode_recording import CameraSession, align_episode, write_json
from recording_storage import check_recording_output


class EpisodeButtons:
    """Fresh rising edges only; held, duplicate and ambiguous A+B never repeat."""
    def __init__(self, callback):
        self.callback = callback
        self.seq = self.received = None
        self.armed = False
        self.last_action = -float("inf")

    def invalidate(self):
        self.armed = False

    def observe(self, hand, now):
        seq = hand.get("seq")
        if type(seq) is not int or seq < 0:
            self.armed = False
            return
        if seq == self.seq:
            return
        gap = self.received is not None and (now-self.received > .1 or seq < self.seq)
        self.seq, self.received = seq, now
        a, b = hand.get("primary_button"), hand.get("secondary_button")
        if gap or not hand.get("valid") or type(a) is not bool or type(b) is not bool:
            self.armed = False
            return
        if not a and not b:
            self.armed = True
            return
        ready, self.armed = self.armed, False
        if ready and a != b and now-self.last_action >= .25:
            self.last_action = now
            self.callback("A" if a else "B", now, seq)


class EpisodeSession:
    def __init__(self, directory, config, task, recorder, stop_motion):
        self.directory, self.task, self.recorder = Path(directory), task, recorder
        self.config = asdict(config)
        self.camera = CameraSession(directory, config, task, True)
        self.stop_motion = stop_motion
        self.buttons = EpisodeButtons(self.request)
        self.phase = "waiting"
        self.current = self.transition = None
        self.requested_result = None
        self.counter = 0
        self.completed = []
        self.closing = False
        self.failure = None
        self.provenance = {}

    async def start(self):
        check_recording_output(self.directory)
        await self.camera.start()
        await asyncio.to_thread(self._save_session)
        print("采集待机：右 A 开始；录制中 A=成功结束，B=失败结束。", flush=True)

    def _save_session(self, state=None):
        write_json(self.directory/"session.json", dict(schema="realman_episode_session_v1",
                   task=self.task, follow_config=self.config, state=state or self.phase,
                   active_episode=self.current["episode_id"] if self.current else None,
                   completed_episodes=self.completed,
                   buttons=dict(A="start / finish success", B="finish failure"),
                   provenance=self.provenance,
                   error=self.failure))

    def request(self, button, received, seq):
        if self.closing:
            return
        self.recorder.event("episode_button", button=button, source_received=received, source_seq=seq,
                            recording_state=self.phase)
        if self.phase == "waiting" and button == "A":
            self.phase = "starting"
            self.transition = asyncio.create_task(self._begin())
        elif self.phase == "recording":
            self.requested_result = (button == "A", button, received)
            self.phase = "end_requested"
            self.stop_motion("episode completion requested; release Grip")
        else:
            self.recorder.event("episode_button_ignored", button=button, recording_state=self.phase)

    def check(self):
        self.camera.check()
        if self.transition is not None and self.transition.done():
            transition, self.transition = self.transition, None
            transition.result()
        if self.failure:
            raise RuntimeError("Episode recording failed: "+self.failure)

    def finish_after_stop(self, verified):
        if self.phase == "end_requested":
            success, button, received = self.requested_result
            self.phase = "saving"
            self.transition = asyncio.create_task(self._finish(success, "controller_"+button,
                                                               verified, received))

    async def _begin(self):
        self.requested_result = None
        self.counter += 1
        episode_id = f"episode_{self.counter:06d}"
        path = self.directory/"episodes"/episode_id
        await asyncio.to_thread(path.mkdir, parents=True, exist_ok=False)
        started = time.monotonic()
        self.current = dict(episode_id=episode_id, task=self.task, started_monotonic=started,
                            utc_started=datetime.now(timezone.utc).isoformat(), task_success=None,
                            status="recording", label_source=None, training_ready=False,
                            provenance=self.provenance,
                            follow_config=self.config)
        await asyncio.to_thread(write_json, path/"episode.json", self.current)
        self.recorder.route_episode(episode_id)
        self.recorder.event("episode_started", task=self.task, started_monotonic=started)
        await self.camera.set_episode(episode_id, started)
        await asyncio.to_thread(self._save_session, "recording")
        self.phase = "recording"
        print(f"\a● 正在录制 {episode_id}：{self.task}；A 成功结束 / B 失败结束。", flush=True)

    async def _finish(self, success, reason, stop_verified, button_received=None):
        if self.current is None:
            return
        metadata = dict(self.current)
        ended = metadata.get("ended_monotonic", time.monotonic())
        metadata.update(ended_monotonic=ended, utc_ended=datetime.now(timezone.utc).isoformat(),
                        task_success=success, label_source=reason, button_received=button_received,
                        stop_verified=stop_verified,
                        status="completed" if success is not None else "interrupted")
        self.current.update(metadata)
        self.recorder.event("episode_finished", task_success=success, label_source=reason,
                            ended_monotonic=ended, stop_verified=stop_verified)
        self.recorder.route_episode(None)
        path = self.directory/"episodes"/metadata["episode_id"]
        await asyncio.to_thread(write_json, path/"episode.json",
                                {**metadata, "status": "saving", "recording_complete": False})
        camera_error = None
        try:
            await self.camera.set_episode(None, ended)
        except Exception as error:
            camera_error = str(error)
            # Close a stalled/dead worker before reading its incomplete files.
            await self.camera.close()
        try:
            await asyncio.to_thread(self.recorder.flush)
            manifest = await asyncio.to_thread(align_episode, path, self.task,
                                               (metadata["started_monotonic"], ended), metadata)
            manifest["recording_complete"] = bool(manifest["camera_complete"] and not camera_error)
            manifest["recording_error"] = camera_error
            await asyncio.to_thread(write_json, path/"episode.json", manifest)
            await asyncio.to_thread(write_json, path/"report.json", dict(
                task_success=success, label_source=reason, stop_verified=stop_verified,
                recording_complete=manifest["recording_complete"],
                images=manifest["images"], aligned_samples=manifest["aligned_samples"],
                recording_error=camera_error))
        except Exception as error:
            metadata.update(recording_complete=False, recording_error=str(error))
            await asyncio.to_thread(write_json, path/"episode.json", metadata)
            raise
        self.completed.append(dict(episode_id=metadata["episode_id"], task_success=success,
                                   label_source=reason, directory=str(path),
                                   recording_complete=manifest["recording_complete"],
                                   images=manifest["images"]))
        self.current = None
        phase = "closed" if self.closing else "waiting"
        self.failure = camera_error
        await asyncio.to_thread(self._save_session, phase)
        self.phase = phase
        label = "成功" if success is True else "失败" if success is False else "未标记（中断）"
        print(f"\a已保存 {metadata['episode_id']}：{label}，{manifest['images']} 帧 → {path}", flush=True)
        if not self.closing and not camera_error:
            print("采集待机；摆好下一轮场景，松开 A/B 后再次按 A 开始。遥操仍在运行。", flush=True)

    async def close(self, reason="operator_exit", stop_verified=None):
        self.closing = True
        errors = []
        if self.transition is not None:
            try:
                await self.transition
            except Exception as error:
                errors.append(str(error))
            self.transition = None
        if self.current is not None:
            try:
                if self.requested_result is not None and self.phase == "end_requested":
                    result, button, received = self.requested_result
                    await self._finish(result, "controller_"+button, stop_verified, received)
                elif "ended_monotonic" in self.current:
                    await self._finish(self.current["task_success"], self.current["label_source"],
                                       stop_verified, self.current.get("button_received"))
                else:
                    await self._finish(None, reason, stop_verified)
            except Exception as error:
                errors.append(str(error))
        self.recorder.route_episode(None)
        camera_report = await self.camera.close()
        self.phase = "closed"
        if errors:
            self.failure = "; ".join(errors)
        await asyncio.to_thread(self._save_session)
        return dict(episodes=self.completed, episode_count=len(self.completed),
                    session_manifest=str(self.directory/"session.json"),
                    camera=camera_report, error=self.failure)
