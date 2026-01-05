
# src/sensorhub/core/sensor_manager.py
import importlib
import logging
import threading
import yaml
from typing import Dict, List, Optional
from pathlib import Path

from .sensor_base import AbstractSensorAdapter
from .schemas import SensorInfo, Sample

log = logging.getLogger("sensorhub.core.sensor_manager")


class SensorManager:
    """
    Thread-safe registry and lifecycle manager for sensor adapters.

    Responsibilities:
      - Load and register adapters from a YAML config.
      - Expose latest sample and ring history (thread-safe snapshots).
      - Cache human-readable descriptions from YAML or adapter.
      - Provide utilities for unregistering and stopping all adapters.
    """

    def __init__(self) -> None:
        # All running adapters keyed by sensor_id
        self.adapters: Dict[str, AbstractSensorAdapter] = {}
        # Cache descriptions read from YAML (or adapter-supplied)
        self.descriptions: Dict[str, Optional[str]] = {}
        # Manager-level lock to protect adapters/descriptions
        self._lock = threading.RLock()

    # -------------------------------------------------------------------------
    # CONFIG LOADING
    # -------------------------------------------------------------------------
    def load_from_config(self, cfg_path: Path, replace_existing: bool = True) -> None:
        """
        Load sensors from a YAML configuration file and register them.

        YAML structure (example):
            sensors:
              - id: rplidar1
                kind: lidar2d
                description: "Slamtec RPLidar S2 on /dev/ttyUSB0"
                module: sensorhub.adapters.rplidar_s2.rplidar_adapter
                class: RPLidarS2Adapter
                params:
                  hz: 12

        Args:
            cfg_path: Path to YAML file.
            replace_existing: If True, replace an already-registered adapter with the same id.
                              If False, skip duplicates and keep the existing adapter.
        """
        cfg_text = Path(cfg_path).read_text()
        cfg = yaml.safe_load(cfg_text) or {}
        entries = cfg.get("sensors", [])

        if not isinstance(entries, list):
            log.warning("Config %s has no 'sensors' list; nothing to load.", cfg_path)
            return

        for entry in entries:
            try:
                module = entry["module"]
                class_name = entry["class"]
                sensor_id = entry["id"]
            except Exception:
                log.warning("Skipping sensor entry missing required keys: %s", entry)
                continue

            kind = entry.get("kind", sensor_id)
            params = entry.get("params", {}) or {}
            description = entry.get("description")  # read description from YAML

            # Import adapter class
            try:
                mod = importlib.import_module(module)
                cls = getattr(mod, class_name)
            except Exception as e:
                log.warning("Skipping %s: import failed (%s)", sensor_id, e)
                continue

            adapter: AbstractSensorAdapter = cls(sensor_id=sensor_id, kind=kind, **params)

            # Prefer adapter-provided description; otherwise set from YAML
            if getattr(adapter, "description", None) is None:
                setattr(adapter, "description", description)

            # Cache description even if adapter doesn't expose it later
            with self._lock:
                self.descriptions[sensor_id] = getattr(adapter, "description", description)

            # Register (replace or skip duplicates as configured)
            self.register(adapter, replace_existing=replace_existing)

    # -------------------------------------------------------------------------
    # REGISTRATION / LIFECYCLE
    # -------------------------------------------------------------------------
    def register(self, adapter: AbstractSensorAdapter, replace_existing: bool = True) -> None:
        """Register an adapter and start its run loop (thread-safe)."""
        with self._lock:
            old = self.adapters.get(adapter.sensor_id)
            if old and not replace_existing:
                log.info("Adapter '%s' already exists; keeping existing.", adapter.sensor_id)
                return

            if old and replace_existing:
                try:
                    log.info("Replacing adapter '%s' with new instance.", adapter.sensor_id)
                    old.stop()
                except Exception as e:
                    log.warning("Stopping old adapter '%s' failed: %s", adapter.sensor_id, e)

            self.adapters[adapter.sensor_id] = adapter

        # Start outside the manager lock to avoid holding locks during thread creation
        try:
            adapter.start()
            log.info("Adapter '%s' started (kind='%s').", adapter.sensor_id, adapter.kind)
        except Exception as e:
            log.error("Adapter '%s' failed to start: %s", adapter.sensor_id, e)

    def unregister(self, sensor_id: str, stop: bool = True) -> bool:
        """
        Unregister an adapter by id. Optionally stop the adapter first.
        Returns True if an adapter was removed.
        """
        with self._lock:
            adapter = self.adapters.pop(sensor_id, None)
        if not adapter:
            return False
        if stop:
            try:
                adapter.stop()
                log.info("Adapter '%s' stopped and unregistered.", sensor_id)
            except Exception as e:
                log.warning("Stopping adapter '%s' during unregister failed: %s", sensor_id, e)
        return True

    def stop_all(self) -> None:
        """Stop all registered adapters (thread-safe)."""
        with self._lock:
            ids = list(self.adapters.keys())
        for sid in ids:
            try:
                self.unregister(sid, stop=True)
            except Exception:
                pass

    # -------------------------------------------------------------------------
    # QUERIES
    # -------------------------------------------------------------------------
    def list(self) -> List[SensorInfo]:
        """
        Return SensorInfo for all registered sensors, including descriptions
        (from adapter.description if present, otherwise from YAML cache).
        """
        out: List[SensorInfo] = []
        with self._lock:
            adapters = list(self.adapters.values())

        for a in adapters:
            desc = getattr(a, "description", None)
            if desc is None:
                with self._lock:
                    desc = self.descriptions.get(a.sensor_id)
            out.append(SensorInfo(id=a.sensor_id, kind=a.kind, description=desc))
        return out

    def info(self, sensor_id: str) -> Optional[SensorInfo]:
        """Return SensorInfo for a single sensor_id."""
        with self._lock:
            a = self.adapters.get(sensor_id)
            desc = self.descriptions.get(sensor_id)
        if not a:
            return None
        if getattr(a, "description", None) is not None:
            desc = a.description
        return SensorInfo(id=a.sensor_id, kind=a.kind, description=desc)

    def ids(self) -> List[str]:
        """Return a list of registered sensor IDs."""
        with self._lock:
            return list(self.adapters.keys())

    def latest(self, sensor_id: str) -> Optional[Sample]:
        """Return the latest Sample for a given sensor_id, or None if empty/not found (thread-safe)."""
        with self._lock:
            a = self.adapters.get(sensor_id)
        if not a:
            return None

        # Use adapter's thread-safe snapshot if available
        snap = getattr(a, "snapshot_latest", None)
        raw = snap() if callable(snap) else getattr(a, "latest", None)

        if not raw:
            return None
        # Pydantic will parse ISO timestamps into datetime automatically
        return Sample(**raw)

    def history(self, sensor_id: str, limit: int = 100) -> List[Sample]:
        """Return up to `limit` recent samples for a given sensor_id (thread-safe copy)."""
        with self._lock:
            a = self.adapters.get(sensor_id)
        if not a:
            return []

        # Copy ring under the adapter's lock if available for consistency
        ring_list: List[dict] = []
        lock = getattr(a, "_lock", None)
        if lock:
            try:
                with lock:
                    ring_list = list(a.ring)
            except Exception:
                ring_list = list(a.ring)
        else:
            ring_list = list(a.ring)

        return [Sample(**s) for s in ring_list[-limit:]]

    def stats(self) -> dict:
        """
        Return a lightweight snapshot of manager/adapter state.
        Useful for debugging without transferring large payloads.
        """
        with self._lock:
            counts = {sid: len(a.ring) for sid, a in self.adapters.items()}
            kinds = {sid: a.kind for sid, a in self.adapters.items()}
        return {"count": len(kinds), "by_kind": kinds, "ring_sizes": counts}


# Singleton instance used by the app
manager = SensorManager()
