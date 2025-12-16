
import math
import time
from sensorhub.core.sensor_base import AbstractSensorAdapter

class SimulatedAdapter(AbstractSensorAdapter):
    def __init__(self, sensor_id: str, kind: str = 'sim', hz: float = 20.0):
        super().__init__(sensor_id, kind)
        self.hz = hz

    def run(self):
        t = 0.0
        period = 1.0 / self.hz
        while not self._stop.is_set():
            val = {
                'value': math.sin(t),
                'phase': t,
            }
            self.publish(val)
            t += period
            time.sleep(period)
