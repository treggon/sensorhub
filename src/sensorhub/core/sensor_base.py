
import threading
import time
from abc import ABC, abstractmethod
from collections import deque
from datetime import datetime, timezone
from typing import Any, Optional, Deque, Callable

class AbstractSensorAdapter(ABC):
    def __init__(self, sensor_id: str, kind: str, ring_size: int = 1024):
        self.sensor_id = sensor_id
        self.kind = kind
        self.ring: Deque[dict] = deque(maxlen=ring_size)
        self.latest: Optional[dict] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.on_sample: Optional[Callable[[dict], None]] = None

    def publish(self, data: Any):
        sample = {
            'sensor_id': self.sensor_id,
            'ts': datetime.now(timezone.utc).isoformat(),
            'data': data,
        }
        self.latest = sample
        self.ring.append(sample)
        if self.on_sample:
            try:
                self.on_sample(sample)
            except Exception:
                pass

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run_wrapper, name=f"{self.sensor_id}-reader", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run_wrapper(self):
        try:
            self.run()
        except Exception as e:
            print(f"[ERROR] {self.sensor_id} adapter crashed: {e}")

    @abstractmethod
    def run(self):
        ...
