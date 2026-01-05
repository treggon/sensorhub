
# src/sensorhub/api/snapshot.py
import io
import math
from typing import Optional

from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import Response
from PIL import Image, ImageDraw  # pip install pillow

from ..core.sensor_manager import manager

router = APIRouter(prefix="/sensors", tags=["sensors"])

@router.get("/{sensor_id}/snapshot.png", response_class=Response)
def snapshot_png(
    sensor_id: str,
    size: int = Query(800, ge=256, le=4096),
    range_scale: float = Query(0.001, description="Scale ranges to meters (default: mm→m)"),
    max_points: Optional[int] = Query(None, description="Decimate to at most N points"),
    theme: str = Query("dark", description="Theme: 'dark' or 'light'"),
    draw_lines: bool = Query(False, description="Connect points with lines instead of dots")
):
    sample = manager.latest(sensor_id)
    if not sample or not isinstance(sample.data, dict):
        raise HTTPException(status_code=404, detail="No sample available")

    angles = sample.data.get("angles")
    ranges = sample.data.get("ranges")
    if not angles or not ranges or len(angles) != len(ranges):
        raise HTTPException(status_code=400, detail="Sample missing angles/ranges or length mismatch")

    # Decimation
    if max_points and max_points > 0 and len(angles) > max_points:
        step = max(1, len(angles) // max_points)
        angles = angles[::step]
        ranges = ranges[::step]

    # Theme
    if theme.lower() == "dark":
        bg = (14, 15, 18); fg = (0, 229, 255); grid = (58, 63, 75); tick = (184, 193, 204)
    else:
        bg = (255, 255, 255); fg = (0, 119, 204); grid = (220, 220, 220); tick = (51, 51, 51)

    img = Image.new("RGB", (size, size), bg)
    draw = ImageDraw.Draw(img)
    margin = int(size * 0.06)
    cx = cy = size // 2
    max_r_px = cx - margin

    # Convert to radians; scale to meters by default
    theta = [math.radians(a) for a in angles]
    rho_m = [float(r) * range_scale for r in ranges]
    rmax = max(rho_m) if rho_m else 1.0
    scale = max_r_px / rmax if rmax > 0 else 1.0

    # Grid
    for frac in (0.25, 0.5, 0.75, 1.0):
        r = int(max_r_px * frac)
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), outline=grid, width=1)
    draw.line((cx - max_r_px, cy, cx + max_r_px, cy), fill=grid, width=1)
    draw.line((cx, cy - max_r_px, cx, cy + max_r_px), fill=grid, width=1)

    # Plot
    last_xy = None
    for t, r_m in zip(theta, rho_m):
        r_px = r_m * scale
        x = cx + int(r_px * math.cos(t))
        y = cy - int(r_px * math.sin(t))
        if draw_lines and last_xy:
            draw.line((last_xy[0], last_xy[1], x, y), fill=fg, width=2)
        else:
            draw.ellipse((x-1, y-1, x+1, y+1), fill=fg)
        last_xy = (x, y)

    # Frame ring
    draw.ellipse((cx - max_r_px, cy - max_r_px, cx + max_r_px, cy + max_r_px), outline=tick, width=1)

    buf = io.BytesIO()
    img.save(buf, format="PNG", optimize=True)
    return Response(content=buf.getvalue(), media_type="image/png")
