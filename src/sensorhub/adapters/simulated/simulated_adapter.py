
# src/sensorhub/adapters/simulated/simulated_adapter.py
import math
import time
from typing import Optional, Dict, Any

from sensorhub.core.sensor_base import AbstractSensorAdapter

# Optional import of Transform helper (works even if not yet added to your repo)
try:
    from sensorhub.core.transform import Transform
except Exception:
    Transform = None  # type: ignore


class SimulatedAdapter(AbstractSensorAdapter):
    """
    Minimal simulated sensor adapter that publishes a sine wave value at 'hz'.
    Now accepts an optional 'transform' dict in __init__, so YAML with params.transform
    won't crash the manager when it forwards constructor args.
    """

    def __init__(
        self,
        sensor_id: str,
        kind: str = "sim",
        hz: float = 20.0,
        transform: Optional[Dict[str, Any]] = None,
        **kwargs,
    ):
        super().__init__(sensor_id, kind)
        self.hz = float(hz)

        # Absorb any extra params without crashing (future-proof)
        for k, v in kwargs.items():
            setattr(self, k, v)

        # Apply transform if provided
        if transform:
            try:
                if Transform is not None:
                    self.set_transform(Transform.from_dict(transform))
                elif hasattr(self, "set_transform"):
                    # Fallback: pass dict directly if your base supports it
                    self.set_transform(transform)  # type: ignore
                else:
                    setattr(self, "_transform", transform)
            except Exception:
                pass

    def run(self) -> None:
        t = 0.0
        period = 1.0 / max(1e-6, self.hz)
        while not self._stop.is_set():
            val = {"value": math.sin(t), "phase": t}
            self.publish(val)
            t += period
            time.sleep(period)
