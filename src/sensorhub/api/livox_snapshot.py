
# src/sensorhub/api/livox_snapshot.py
import io
import math
from typing import List, Tuple
from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import Response
from PIL import Image, ImageDraw

from ..core.sensor_manager import manager

router = APIRouter(prefix="/sensors", tags=["livox"])

def _polar_to_points(angles: List[float], ranges: List[float]) -> List[Tuple[float,float,float,int]]:
    """
    Convert polar scan data into XY points with z=0 and intensity placeholder.
    Returns list of (x, y, z, intensity).
    """
    if not isinstance(angles, list) or not isinstance(ranges, list):
        raise HTTPException(status_code=400, detail="Sample missing angles/ranges or length mismatch")
    if len(angles) != len(ranges) or len(angles) == 0:
        raise HTTPException(status_code=400, detail="Sample missing angles/ranges or length mismatch")

    pts = []
    for a, r in zip(angles, ranges):
        x = r * math.cos(a)
        y = r * math.sin(a)
        # z and intensity unknown; set sensible defaults
        pts.append((x, y, 0.0, 255))
    return pts

@router.get("/livox/snapshot.png", response_class=Response)
def livox_snapshot(
    size: int = Query(800, ge=256, le=4096),
    max_points: int = Query(30000, ge=1000),
    theme: str = Query("dark"),
    z_mode: str = Query("none", description="none | height | intensity"),
):
    sample = manager.latest("livox")
    if not sample or not isinstance(sample.data, dict):
        raise HTTPException(status_code=404, detail="No Livox sample available")

    data = sample.data

    # Accept either direct cartesian points or polar angles/ranges
    pts = None

    if "points" in data and isinstance(data["points"], list) and len(data["points"]) > 0:
        # Expected shape: [x, y, z, intensity]
        pts = data["points"]
        # Normalize tuples, and pad missing elements
        fixed = []
        for p in pts:
            if not isinstance(p, (list, tuple)) or len(p) < 2:
                continue
            x = float(p[0])
            y = float(p[1])
            z = float(p[2]) if len(p) >= 3 else 0.0
            i = int(p[3]) if len(p) >= 4 else 255
            fixed.append((x, y, z, i))
        pts = fixed

    elif "angles" in data and "ranges" in data:
        pts = _polar_to_points(data["angles"], data["ranges"])

    else:
        raise HTTPException(status_code=404, detail="No Livox points or angles/ranges in sample")

    if not pts:
        raise HTTPException(status_code=404, detail="Empty point set")

    # Decimate
    if len(pts) > max_points:
        step = max(1, len(pts) // max_points)
        pts = pts[::step]

    # Split components
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    iset = [p[3] for p in pts]  # currently unused, but available

    # Bounds -> square
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    span = max(xmax - xmin, ymax - ymin) or 1.0
    cx = (xmin + xmax) / 2.0
    cy = (ymin + ymax) / 2.0
    scale = (size * 0.45) / (span / 2.0)

    # Theme
    if theme.lower() == "dark":
        bg = (14, 14, 18); fg = (0, 220, 255); grid = (60, 60, 70)
    else:
        bg = (255, 255, 255); fg = (0, 100, 200); grid = (200, 200, 200)

    img = Image.new("RGB", (size, size), bg)
    draw = ImageDraw.Draw(img)

    # Crosshairs
    draw.line((size//2, 0, size//2, size), fill=grid)
    draw.line((0, size//2, size, size//2), fill=grid)

    # Height/intensity coloring
    zmin, zmax = min(zs), max(zs)
    for (x, y, z, i) in pts:
        X = size//2 + int((x - cx) * scale)
        Y = size//2 - int((y - cy) * scale)

        if z_mode == "height":
            t = 0.0 if zmax == zmin else (z - zmin) / (zmax - zmin)
            color = (int(255 * t), 64, 255 - int(255 * t))
        elif z_mode == "intensity":
            # If intensity is not 0-255, clamp
            t = min(max(float(i) / 255.0, 0.0), 1.0)
            color = (int(255 * t), int(255 * (1.0 - t)), 40)
        else:
            color = fg

        draw.point((X, Y), fill=color)

    buf = io.BytesIO()
    img.save(buf, format="PNG", optimize=True)
    return Response(content=buf.getvalue(), media_type="image/png")
