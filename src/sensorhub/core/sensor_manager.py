
import importlib
import yaml
from typing import Dict, List, Optional
from pathlib import Path
from .sensor_base import AbstractSensorAdapter
from .schemas import SensorInfo, Sample

class SensorManager:
    def __init__(self):
        self.adapters: Dict[str, AbstractSensorAdapter] = {}

    def load_from_config(self, cfg_path: Path):
        cfg = yaml.safe_load(Path(cfg_path).read_text())
        for entry in cfg.get('sensors', []):
            module = entry['module']
            class_name = entry['class']
            sensor_id = entry['id']
            kind = entry.get('kind', sensor_id)
            params = entry.get('params', {})
            mod = importlib.import_module(module)
            cls = getattr(mod, class_name)
            adapter: AbstractSensorAdapter = cls(sensor_id=sensor_id, kind=kind, **params)
            self.register(adapter)

    def register(self, adapter: AbstractSensorAdapter):
        self.adapters[adapter.sensor_id] = adapter
        adapter.start()

    def list(self) -> List[SensorInfo]:
        return [SensorInfo(id=a.sensor_id, kind=a.kind) for a in self.adapters.values()]

    def latest(self, sensor_id: str) -> Optional[Sample]:
        a = self.adapters.get(sensor_id)
        if not a or not a.latest:
            return None
        return Sample(**a.latest)

    def history(self, sensor_id: str, limit: int = 100) -> List[Sample]:
        a = self.adapters.get(sensor_id)
        if not a:
            return []
        return [Sample(**s) for s in list(a.ring)[-limit:]]

manager = SensorManager()
