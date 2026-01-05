
# src/sensorhub/api/sensors_latest.py
"""
Responsive 'latest' endpoints for all sensors.
- Metadata-only by default
- Optional points with decimation + trimming + quantization
- Uses ORJSONResponse when available
- Adds ETag/Cache-Control for better client polling behavior
"""
from typing import Any, Dict, Iterable, List, Tuple
from fastapi import APIRouter, HTTPException, Query
from fastapi import Response
try:
    from fastapi.responses import ORJSONResponse as JSONResp
except Exception:
    from fastapi.responses import JSONResponse as JSONResp

from ..core.sensor_manager import manager

router = APIRouter(prefix="/sensors", tags=["latest"])

LARGE_KEYS = {"points", "angles", "ranges"}
VALID_KEEP: Tuple[str, ...] = ("xy", "xyz", "xyi", "xyzi")


def _validate_keep(keep: str) -> str:
    k = (keep or "").lower()
    if k not in VALID_KEEP:
        # default to xyz, which is most common for map views
        return "xyz"
    return k


def _compact_points(
    pts: Any,
    keep: str,
    decimals: int,
    max_points: int,
) -> List[Tuple]:
    """
    Decimate, trim fields, and quantize points for smaller, faster JSON responses.

    pts: list of [x, y, z, i] (tuples/lists) — extra values ignored; missing z/i padded
    keep: one of VALID_KEEP
    decimals: rounding decimals for x, y, z
    max_points: decimation cap
    """
    if not isinstance(pts, list) or not pts:
        return []

    # Decimate early to reduce work
    n = len(pts)
    if n > max_points:
        step = max(1, n // max_points)
        pts = pts[::step]

    idxs = {
        "xy": (0, 1),
        "xyz": (0, 1, 2),
        "xyi": (0, 1, 3),
        "xyzi": (0, 1, 2, 3),
    }[_validate_keep(keep)]

    dec = max(0, int(decimals))
    out: List[Tuple] = []

    for p in pts:
        # Accept tuples/lists only; skip malformed rows quickly
        if not isinstance(p, (list, tuple)):
            continue
        # Pad to at least 4 elements to simplify index access
        vals: List[float] = list(p) + [0.0, 0.0, 0.0, 0]
        row: List[Any] = []
        for j in idxs:
            if j in (0, 1, 2):  # x,y,z → float rounding
                try:
                    row.append(round(float(vals[j]), dec))
                except Exception:
                    row.append(0.0)
            else:  # intensity → int
                try:
                    row.append(int(vals[j]))
                except Exception:
                    row.append(0)
        out.append(tuple(row))

    return out


def _set_cache_headers(resp: Response, etag: str) -> None:
    # Short max-age since frames change frequently; clients can leverage ETag for 304 handling
    resp.headers["Cache-Control"] = "private, max-age=1, must-revalidate"
    resp.headers["ETag"] = etag


@router.get("/{sensor_id}/latest", response_class=JSONResp)
def latest_sensor(
    sensor_id: str,
    include_points: bool = Query(False, description="Include point array if available"),
    max_points: int = Query(20000, ge=1000, description="Decimation cap when including points"),
    keep: str = Query("xyz", description="Point fields: xy|xyz|xyi|xyzi"),
    decimals: int = Query(2, ge=0, le=6, description="Quantization decimals for x,y,z"),
    include_meta: bool = Query(False, description="Include small diagnostic fields (e.g., decode_diag)"),
    response: Response = None,
):
    """
    Fast 'latest' endpoint:
    - By default returns metadata only (no large arrays)
    - With include_points=true, returns decimated & compacted points
    - include_meta=true includes small meta keys (e.g., 'decode_diag') without bulk arrays
    """
    sample = manager.latest(sensor_id)
    if not sample:
        raise HTTPException(status_code=404, detail="No sample")

    sdict = sample.dict()
    ts = sdict.get("ts")
    data = sdict.get("data") or {}

    # Shallow copy only when we need to mutate; avoid duplicating large payloads
    out: Dict[str, Any] = {}

    # Always include small, constant‑size keys (status, counters, etc.)
    # Copy everything except LARGE_KEYS first; then optionally add points
    for k, v in data.items():
        if k in LARGE_KEYS:
            continue
        # include_meta controls small meta fields like 'decode_diag'
        if not include_meta and k in {"decode_diag", "points_format", "frame_points"}:
            continue
        out[k] = v

    if include_points:
        pts = data.get("points")
        out["points"] = _compact_points(pts, keep, decimals, max_points)
        out["frame_points"] = len(out["points"])

    # Prepare response
    payload = {
        "sensor_id": sdict.get("sensor_id"),
        "ts": ts,
        "data": out,
    }

    resp = JSONResp(content=payload)
    # Add weak ETag based on sensor_id + ts; clients can skip downloads if unchanged
    etag = f'W/"{sdict.get("sensor_id")}-{ts}"'
    _set_cache_headers(resp, etag)
    return resp


@router.get("/livox/frame", response_class=JSONResp)
def livox_frame(
    max_points: int = Query(20000, ge=1000, description="Decimation cap for Livox points"),
    keep: str = Query("xyz", description="Point fields: xy|xyz|xyi|xyzi"),
    decimals: int = Query(2, ge=0, le=6, description="Quantization decimals for x,y,z"),
    response: Response = None,
):
    """
    Livox-specific frame endpoint:
    - Returns points-only for clients that explicitly need raw points
    - Decimates, trims, and quantizes server-side
    """
    sample = manager.latest("livox")
    if not sample:
        raise HTTPException(status_code=404, detail="No Livox sample")

    sdict = sample.dict()
    ts = sdict.get("ts")
    data = sdict.get("data") or {}

    pts = data.get("points")
    compact = _compact_points(pts, keep, decimals, max_points)

    resp = JSONResp(content={"points": compact, "frame_points": len(compact)})
    etag = f'W/"livox-{ts}"'
    _set_cache_headers(resp, etag)
    return resp
