"""Bounded asynchronous JSONL writer for the continuous control loop."""
from datetime import datetime, timezone
import json
from pathlib import Path
import queue
import threading
import time


class BufferedRecorder:
    def __init__(self, directory, capacity=2048):
        Path(directory).mkdir(parents=True, exist_ok=False)
        self.directory = Path(directory)
        self.episode_id = None
        self.stream = (Path(directory)/"events.jsonl").open("x", encoding="utf-8")
        self.queue = queue.Queue(capacity)
        self.error = None
        self.written = self.submitted = 0
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def event(self, event, **values):
        if self.error:
            return  # Never prevent the independent stop/verification path.
        try:
            episode_id = self.episode_id
            row = json.dumps(dict(utc=datetime.now(timezone.utc).isoformat(),
                                  monotonic=time.monotonic(), event=event,
                                  episode_id=episode_id, **values),
                             ensure_ascii=False, allow_nan=False)
            self.queue.put_nowait((row, episode_id))
            self.submitted += 1
        except Exception as error:
            self.error = str(error) or type(error).__name__

    def _write(self):
        episode_stream = None
        current_id = None
        try:
            while True:
                item = self.queue.get()
                if item is None:
                    break
                if isinstance(item, threading.Event):
                    self.stream.flush()
                    if episode_stream:
                        episode_stream.flush()
                        episode_stream.close()
                        episode_stream = current_id = None
                    item.set()
                    continue
                line, episode_id = item
                if episode_id != current_id:
                    if episode_stream:
                        episode_stream.close()
                    episode_stream = None
                    current_id = episode_id
                    if episode_id is not None:
                        episode_stream = (self.directory/"episodes"/episode_id/"events.jsonl").open("a", encoding="utf-8")
                self.stream.write(line+"\n")
                if episode_stream:
                    episode_stream.write(line+"\n")
                self.written += 1
                if self.queue.empty():
                    self.stream.flush()
                    if episode_stream:
                        episode_stream.flush()
        except Exception as error:
            self.error = str(error)
        finally:
            if episode_stream:
                episode_stream.close()
            self.stream.close()

    def route_episode(self, episode_id):
        if episode_id is not None and (Path(episode_id).name != episode_id or not episode_id.startswith("episode_")):
            raise ValueError("Invalid episode id")
        self.episode_id = episode_id

    def flush(self, timeout=5.):
        self.check()
        barrier = threading.Event()
        self.queue.put(barrier, timeout=timeout)
        if not barrier.wait(timeout):
            self.check()
            raise RuntimeError("Event recorder flush timed out")
        self.check()

    def check(self):
        if self.error:
            raise RuntimeError("Continuous recording failed: "+self.error)

    def close(self):
        while self.thread.is_alive():
            try:
                self.queue.put(None, timeout=.1)
                break
            except queue.Full:
                pass
        self.thread.join(timeout=5)
        complete = not self.error and not self.thread.is_alive() and self.written == self.submitted
        return dict(complete=complete, submitted=self.submitted, written=self.written, error=self.error)


# Shared atomic writer used by local configuration commands.
from episode_recording import write_json
