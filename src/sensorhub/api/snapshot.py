
# src/sensorhub/api/snapshot.py
import io
import math
from typing import Optional, List, Dict, Any

from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import Response
from PIL import Image, ImageDraw  # pip install pillow

from ..core.sensor_manager import manager

router = APIRouter(prefix="/sensors", tags=["sensors"])


def _get_points(sensor_id: str) -> List[Dict[str, Any]]:
    """
    Try in order:
      1) manager.latest(sensor_id) -> data.points
      2) adapter.latest_raw_points() cache (method or property)
      3) manager.latest(sensor_id) -> legacy data.angles/ranges -> synthesize points
    Returns a list of {"angle_deg": float, "distance_mm": float, "quality": int?}
    Raises HTTPException 404/204 appropriately if nothing is available yet.
    """
    # 1) Latest sample with points
    sample = manager.latest(sensor_id)
    if sample and isinstance(sample.data, dict):
        pts = sample.data.get("points")
        if isinstance(pts, list) and pts:
            return pts

    # 2) Adapter raw cache
    adapter = manager.get_adapter(sensor_id)
    if adapter:
        raw_accessor = getattr(adapter, "latest_raw_points", None)
        try:
            if callable(raw_accessor):
                cache_pts = raw_accessor()
                if isinstance(cache_pts, list) and cache_pts:
                    return cache_pts
            elif raw_accessor is not None:
                cache_pts = raw_accessor
                if isinstance(cache_pts, list) and cache_pts:
                    return cache_pts
        except Exception:
            # ignore cache errors and continue
            pass

    # 3) Legacy angles/ranges/qualities -> synthesize points
    if sample and isinstance(sample.data, dict):
        angles = sample.data.get("angles")
        ranges = sample.data.get("ranges")
        qualities = sample.data.get("qualities", [])
        if isinstance(angles, list) and isinstance(ranges, list) and len(angles) == len(ranges) and len(angles) > 0:
            pts: List[Dict[str, Any]] = []
            n = len(angles)
            # qualities may be missing or length mismatch; treat missing as 0
            for i in range(n):
                ang = float(angles[i])
                dist = float(ranges[i])
                qual = int(qualities[i]) if i < len(qualities) else 0
                pts.append({"angle_deg": ang, "distance_mm": dist, "quality": qual})
            return pts

    # Nothing yet
    raise HTTPException(status_code=204, detail="no points available yet")


@router.get("/{sensor_id}/snapshot.png", response_class=Response)
def snapshot_png(
    sensor_id: str,
    size: int = Query(800, ge=256, le=4096),
    range_scale: float = Query(0.001, description="Scale ranges to meters (default: mm→m)"),
    max_points: Optional[int] = Query(None, description="Decimate to at most N points"),
    theme: str = Query("dark", description="Theme: 'dark' or 'light'"),
    draw_lines: bool = Query(False, description="Connect points with lines instead of dots"),
):
    # Resolve points with robust fallbacks
    points = _get_points(sensor_id)  # raises 204 if none

    if not points:
        raise HTTPException(status_code=204, detail="no points in latest sample")

    # Optional decimation
    if isinstance(max_points, int) and max_points > 0 and len(points) > max_points:
        step = max(1, len(points) // max_points)
        points = points[::step]

    # Theme colors
    if theme.lower() == "dark":
        bg = (14, 15, 18); fg = (0, 229, 255); grid = (58, 63, 75); tick = (184, 193, 204)
    else:
        bg = (255, 255, 255); fg = (0, 119, 204); grid = (220, 220, 220); tick = (51, 51, 51)

    # Prepare canvas
    img = Image.new("RGB", (size, size), bg)
    draw = ImageDraw.Draw(img)
    margin = int(size * 0.06)
    cx = cy = size // 2
    max_r_px = cx - margin

    # Convert angles (deg -> rad) and distances (mm -> meters via range_scale)
    theta = []
    rho_m = []
    for p in points:
        try:
            a_deg = float(p.get("angle_deg"))
            d_mm = float(p.get("distance_mm"))
        except Exception:
            # skip malformed entries
            continue
        theta.append(math.radians(a_deg))
        rho_m.append(d_mm * range_scale)

    if not rho_m:
        raise HTTPException(status_code=204, detail="no valid points to render")

    # Fit to canvas
    rmax = max(rho_m) if rho_m else 1.0
    scale = max_r_px / rmax if rmax > 0 else 1.0

    # Grid (rings + crosshair)
    for frac in (0.25, 0.5, 0.75, 1.0):
        r = int(max_r_px * frac)
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), outline=grid, width=1)
    draw.line((cx - max_r_px, cy, cx + max_r_px, cy), fill=grid, width=1)
    draw.line((cx, cy - max_r_px, cx, cy + max_r_px), fill=grid, width=1)

    # Plot points or line path
    last_xy = None
    for t, r_m in zip(theta, rho_m):
        r_px = r_m * scale
        x = cx + int(r_px * math.cos(t))
        y = cy - int(r_px * math.sin(t))
        if draw_lines and last_xy:
            draw.line((last_xy[0], last_xy[1], x, y), fill=fg, width=2)
        else:
            draw.ellipse((x - 1, y - 1, x + 1, y + 1), fill=fg)
        last_xy = (x, y)

    # Outer frame ring
    draw.ellipse((cx - max_r_px, cy - max_r_px, cx + max_r_px, cy + max_r_px), outline=tick, width=1)

    # Encode PNG
    buf = io.BytesIO()
    img.save(buf, format="PNG", optimize=True)
    return Response(content=buf.getvalue(), media_type="image/png")
