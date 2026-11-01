"""Bounded JSONL writer and atomic reports; no camera or dataset dependencies."""
from datetime import datetime, timezone
import json
from pathlib import Path
import queue
import threading
import time


def write_json(path, data):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    temporary.replace(path)


class BufferedRecorder:
    def __init__(self, directory, capacity=2048):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.stream = (self.directory/'events.jsonl').open('x', encoding='utf-8')
        self.queue = queue.Queue(capacity)
        self.error = None
        self.written = self.submitted = 0
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def event(self, event, **values):
        if self.error:
            return  # Logging failure must never block the independent STOP path.
        try:
            row = json.dumps(dict(utc=datetime.now(timezone.utc).isoformat(),
                monotonic=time.monotonic(), event=event, **values), ensure_ascii=False, allow_nan=False)
            self.queue.put_nowait(row)
            self.submitted += 1
        except Exception as error:
            self.error = str(error) or type(error).__name__

    def _write(self):
        try:
            while True:
                line = self.queue.get()
                if line is None:
                    break
                self.stream.write(line+'\n')
                self.written += 1
                if self.queue.empty():
                    self.stream.flush()
        except Exception as error:
            self.error = str(error)
        finally:
            self.stream.close()

    def check(self):
        if self.error:
            raise RuntimeError('Event logging failed: '+self.error)

    def close(self):
        while self.thread.is_alive():
            try:
                self.queue.put(None, timeout=.1)
                break
            except queue.Full:
                pass
        self.thread.join(timeout=5)
        return dict(complete=not self.error and not self.thread.is_alive() and self.written==self.submitted,
                    submitted=self.submitted, written=self.written, error=self.error)
