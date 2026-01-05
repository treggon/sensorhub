
# src/sensorhub/core/sensor_base.py
import threading
import time
from abc import ABC, abstractmethod
from collections import deque
from datetime import datetime, timezone
from typing import Any, Optional, Deque, Callable


class AbstractSensorAdapter(ABC):
    """
    Base class for sensor adapters.

    Thread-safety:
      - Writes to `latest` and `ring` are protected by `_lock` (RLock).
      - Readers should use `snapshot_latest()` for a thread-safe shallow copy.

    Lifecycle:
      - Call `start()` to spawn the adapter's read loop thread.
      - Call `stop()` to signal termination and join the thread.
      - `is_running()` returns True while the thread is alive and not stopped.
    """

    def __init__(self, sensor_id: str, kind: str, ring_size: int = 1024) -> None:
        self.sensor_id = sensor_id
        self.kind = kind

        # Data buffers
        self.ring: Deque[dict] = deque(maxlen=ring_size)
        self.latest: Optional[dict] = None

        # Threading / control
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.RLock()  # <-- thread-safety for latest/ring

        # Optional per-sample callback (e.g., metrics, side effects)
        self.on_sample: Optional[Callable[[dict], None]] = None

    # ------------------------------------------------------------------
    # Publishing (writer)
    # ------------------------------------------------------------------
    def publish(self, data: Any) -> None:
        """
        Create a sample dict and append to ring; update latest.
        Runs callbacks outside the lock to avoid contention.
        """
        sample = {
            "sensor_id": self.sensor_id,
            "ts": datetime.now(timezone.utc).isoformat(),
            "data": data,
        }

        # Write under lock
        with self._lock:
            self.latest = sample
            self.ring.append(sample)

        # Callback outside lock
        if self.on_sample:
            try:
                self.on_sample(sample)
            except Exception:
                # Never let callback issues break the adapter write path
                pass

    def snapshot_latest(self) -> Optional[dict]:
        """
        Thread-safe shallow copy of the latest sample.
        Use this from readers (e.g., HTTP routes) to avoid races.
        """
        with self._lock:
            if self.latest is None:
                return None
            # Shallow copy is sufficient; inner `data` should be treated as immutable by readers
            return dict(self.latest)

    # ------------------------------------------------------------------
    # Lifecycle (thread management)
    # ------------------------------------------------------------------
    def start(self) -> None:
        """
        Start the adapter's run loop in a background thread.
        No-op if already running.
        """
        if self._thread and self._thread.is_alive():
            return

        self._stop.clear()

        # Normalize a short, helpful thread name
        name = f"{self.sensor_id}-{self.__class__.__name__}-reader"
        self._thread = threading.Thread(target=self._run_wrapper, name=name, daemon=True)
        self._thread.start()

    def stop(self, join_timeout: float = 2.0, yield_after_stop: float = 0.0) -> None:
        """
        Signal stop and join the thread.
        `yield_after_stop` optionally sleeps a tiny amount to yield CPU (useful when many adapters stop together).
        """
        self._stop.set()
        if self._thread:
            try:
                self._thread.join(timeout=join_timeout)
            except Exception:
                # If join fails, we still let the process continue; thread is daemon=True
                pass

        if yield_after_stop > 0.0:
            try:
                time.sleep(yield_after_stop)
            except Exception:
                pass

    def is_running(self) -> bool:
        """Return True if the adapter thread is alive and not stopped."""
        return bool(self._thread and self._thread.is_alive() and not self._stop.is_set())

    def set_on_sample(self, cb: Optional[Callable[[dict], None]]) -> None:
        """Register or clear a per-sample callback."""
        self.on_sample = cb

    # ------------------------------------------------------------------
    # Run wrapper (exception safety)
    # ------------------------------------------------------------------
    def _run_wrapper(self) -> None:
        """
        Calls `run()` and protects against uncaught exceptions so
        an adapter crash doesn't propagate and kill the process.
        """
        try:
            self.run()
        except Exception as e:
            # Keep this simple and robust; logging framework can be used in concrete adapters
            print(f"[ERROR] {self.sensor_id} adapter crashed: {e}")

    # ------------------------------------------------------------------
    # Implement in concrete adapters
    # ------------------------------------------------------------------
    @abstractmethod
    def run(self) -> None:
        """
        Adapter read loop. Implement polling, I/O, and publish() calls here.
        Respect `self._stop.is_set()` to exit cleanly.
        """
        ...
