
from sensorhub.core.sensor_base import AbstractSensorAdapter

class Dummy(AbstractSensorAdapter):
    def run(self):
        self.publish({'foo': 'bar'})

def test_ring_and_latest():
    d = Dummy('x', 'dummy')
    d.start()
    d.stop()
    assert d.latest is not None
    assert len(d.ring) >= 1
