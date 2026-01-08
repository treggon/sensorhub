
# src/sensorhub/api/livox_snapshot.py
import io
import math
from typing import List, Tuple

from fastapi import APIRouter, HTTPException, Query, Request
from fastapi.responses import Response
from PIL import Image, ImageDraw, ImageFilter

from ..core.sensor_manager import manager

router = APIRouter(prefix="/sensors", tags=["livox"])


def _polar_to_points_deg(angles_deg: List[float], ranges_m: List[float]) -> List[Tuple[float, float, float, int]]:
    """Convert polar (degrees, meters) to (x,y,z,i) tuples; z=0, i=255."""
    if not isinstance(angles_deg, list) or not isinstance(ranges_m, list):
        raise HTTPException(status_code=400, detail="Sample missing angles/ranges or length mismatch")
    if len(angles_deg) != len(ranges_m) or len(angles_deg) == 0:
        raise HTTPException(status_code=400, detail="Sample missing angles/ranges or length mismatch")

    pts: List[Tuple[float, float, float, int]] = []
    for a_deg, r_m in zip(angles_deg, ranges_m):
        rad = math.radians(a_deg)
        x = r_m * math.cos(rad)
        y = r_m * math.sin(rad)
        pts.append((x, y, 0.0, 255))
    return pts


# --- Colormap helpers (lightweight) --------------------------------------------
def _turbo(t: float) -> Tuple[int, int, int]:
    t = max(0.0, min(1.0, t))
    r = int(255 * (0.5 + 0.5 * math.sin(6.28 * t + 0.0)))
    g = int(255 * (0.5 + 0.5 * math.sin(6.28 * t - 2.1)))
    b = int(255 * (0.5 + 0.5 * math.sin(6.28 * t - 4.2)))
    return (r, g, b)


def _viridis(t: float) -> Tuple[int, int, int]:
    t = max(0.0, min(1.0, t))
    r = int(255 * (0.267 + 0.584 * t))
    g = int(255 * (0.005 + 0.866 * t))
    b = int(255 * (0.329 + 0.604 * (1.0 - t)))
    return (r, g, b)


@router.options("/livox/snapshot.png", response_class=Response)
def livox_snapshot_options(request: Request) -> Response:
    """Explicit CORS preflight handler: reply 200 with CORS headers."""
    origin = request.headers.get("origin", "*")
    acrh = request.headers.get("access-control-request-headers", "*")
    headers = {
        "Access-Control-Allow-Origin": origin,
        "Vary": "Origin",
        "Access-Control-Allow-Methods": "GET, OPTIONS",
        "Access-Control-Allow-Headers": acrh,
        "Access-Control-Max-Age": "600",
    }
    return Response(status_code=200, headers=headers)


@router.get("/livox/snapshot.png", response_class=Response)
def livox_snapshot(
    request: Request,
    size: int = Query(800, ge=256, le=4096),
    max_points: int = Query(30000, ge=1000),
    theme: str = Query("dark"),
    z_mode: str = Query("none", description="none | height | intensity"),
    # Styling knobs
    point_size: int = Query(3, ge=1, le=20, description="Radius in pixels for each point"),
    alpha: float = Query(0.85, ge=0.1, le=1.0, description="Opacity of points"),
    # Grid & centering controls
    grid_mode: str = Query("off", description="off | image | origin"),
    center_mode: str = Query("auto", description="auto | origin"),
    padding: float = Query(0.05, ge=0.0, le=0.25, description="Extra fractional padding around bounds"),
    clip_circle: bool = Query(False, description="Clip rendering to circular FOV"),
    cmap: str = Query("turbo", description="turbo | viridis"),
    show_origin: bool = Query(True, description="Draw a marker at (0,0) origin"),
    # NEW: origin thickness affects dot radius and origin cross width
    origin_thickness_px: int = Query(3, ge=1, le=20, description="Origin dot radius & origin cross width (pixels)"),
    # Metric rings
    show_rings: bool = Query(True, description="Draw metric distance rings centered at origin"),
    ring_radii_m: str = Query("5,10,20", description="Comma-separated radii in meters"),
    ring_width_px: int = Query(2, ge=1, le=6, description="Ring outline width in pixels"),
    ring_alpha: float = Query(0.75, ge=0.1, le=1.0, description="Ring outline opacity"),
    ring_labels: bool = Query(True, description="Label rings with meters"),
    # Legacy/polar inputs handling
    angles_in_degrees: bool = Query(True, description="If using angles/ranges, angles are degrees"),
    ranges_in_mm: bool = Query(False, description="If using angles/ranges, ranges are millimeters"),
) -> Response:
    """
    Render a square PNG snapshot of the latest Livox frame.

    - grid_mode: 'off' (no cross), 'image' (cross at image center), 'origin' (cross at LiDAR origin).
      When 'origin', the stroke width equals origin_thickness_px.
    - center_mode: 'auto' (center on point cloud bounds), 'origin' (center on 0,0).
    - origin_thickness_px: controls both origin dot radius and origin cross width.
    """
    # --- fetch sample ---
    sample = manager.latest("livox")
    if not sample or not isinstance(sample.data, dict):
        raise HTTPException(status_code=404, detail="No Livox sample available")

    data = sample.data
    pts: List[Tuple[float, float, float, int]] = []

    # --- build (x,y,z,i) ---
    if "points" in data and isinstance(data["points"], list) and len(data["points"]) > 0:
        raw = data["points"]
        if isinstance(raw[0], (list, tuple)):
            fixed: List[Tuple[float, float, float, int]] = []
            for p in raw:
                if not isinstance(p, (list, tuple)) or len(p) < 2:
                    continue
                x = float(p[0]); y = float(p[1])
                z = float(p[2]) if len(p) >= 3 else 0.0
                i = int(p[3]) if len(p) >= 4 else 255
                # plausibility filter
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                if max(abs(x), abs(y)) > 50.0 or abs(z) > 10.0:
                    continue
                fixed.append((x, y, z, max(0, min(i, 255))))
            pts = fixed
        else:
            raise HTTPException(status_code=400, detail="Flat 'points' not supported by snapshot; use adapter output.")
    elif "angles" in data and "ranges" in data:
        angles = list(data["angles"])
        ranges = list(data["ranges"])
        if ranges_in_mm:
            ranges = [float(r) / 1000.0 for r in ranges]
        if angles_in_degrees:
            pts = _polar_to_points_deg(angles, ranges)
        else:
            pts = [(r * math.cos(a), r * math.sin(a), 0.0, 255) for a, r in zip(angles, ranges)]
    else:
        raise HTTPException(status_code=404, detail="No Livox points or angles/ranges in sample")

    if not pts:
        raise HTTPException(status_code=404, detail="Empty point set")

    # --- decimation ---
    if len(pts) > max_points:
        step = max(1, len(pts) // max_points)
        pts = pts[::step]

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    iset = [p[3] for p in pts]

    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    span_x = xmax - xmin
    span_y = ymax - ymin
    span = max(span_x, span_y)
    if not math.isfinite(span) or span <= 0.0:
        span = 1e-6

    pad = padding * span
    xmin -= pad; xmax += pad
    ymin -= pad; ymax += pad

    # --- centering: auto vs origin ---
    if center_mode.lower() == "origin":
        cx, cy = 0.0, 0.0
        span = max(xmax - xmin, ymax - ymin)
    else:
        cx = (xmin + xmax) / 2.0
        cy = (ymin + ymax) / 2.0
        span = max(xmax - xmin, ymax - ymin)

    scale = (size * 0.45) / (span / 2.0)

    # --- theme ---
    if theme.lower() == "dark":
        bg = (14, 14, 18, 255)
        fg = (0, 220, 255, int(255 * alpha))
        grid_color = (60, 60, 70, 255)
        hud = (200, 200, 210, 180)
        ring_base = (180, 180, 190)
        origin_color = (255, 140, 0, 220)
    else:
        bg = (255, 255, 255, 255)
        fg = (0, 100, 200, int(255 * alpha))
        grid_color = (200, 200, 200, 255)
        hud = (40, 40, 40, 160)
        ring_base = (80, 80, 90)
        origin_color = (255, 120, 0, 220)

    base = Image.new("RGBA", (size, size), bg)
    draw = ImageDraw.Draw(base)

    # --- colormap & normalization ---
    cm = _turbo if cmap.lower() == "turbo" else _viridis
    imin, imax = (min(iset), max(iset)) if iset else (0, 255)
    zmin, zmax = (min(zs), max(zs)) if zs else (0.0, 1.0)

    # --- map origin to image coords ---
    X0 = size // 2 + int((0.0 - cx) * scale)
    Y0 = size // 2 - int((0.0 - cy) * scale)

    # --- optional circular mask ---
    if clip_circle:
        mask = Image.new("L", (size, size), 0)
        mdraw = ImageDraw.Draw(mask)
        mdraw.ellipse((size*0.05, size*0.05, size*0.95, size*0.95), fill=255)
    else:
        mask = None

    # --- grid/crosshair ---
    gm = grid_mode.lower()
    if gm == "image":
        # cross at image center (old behavior)
        draw.line((size // 2, 0, size // 2, size), fill=grid_color)
        draw.line((0, size // 2, size, size // 2), fill=grid_color)
    elif gm == "origin":
        # cross at true origin; width equals origin_thickness_px
        if 0 <= X0 < size and 0 <= Y0 < size:
            w = max(1, int(origin_thickness_px))
            draw.line((X0, 0, X0, size), fill=grid_color, width=w)
            draw.line((0, Y0, size, Y0), fill=grid_color, width=w)
    # else: off (no cross)

    # --- rings (draw before points) ---
    if show_rings:
        try:
            radii = [float(s.strip()) for s in ring_radii_m.split(",") if s.strip()]
        except Exception:
            radii = [5.0, 10.0, 20.0]
        ring_layer = Image.new("RGBA", (size, size), (0, 0, 0, 0))
        rdraw = ImageDraw.Draw(ring_layer)
        ring_color = (ring_base[0], ring_base[1], ring_base[2], int(255 * ring_alpha))

        for r_m in radii:
            if r_m <= 0:
                continue
            rp = r_m * scale
            bbox = (X0 - rp, Y0 - rp, X0 + rp, Y0 + rp)
            try:
                rdraw.ellipse(bbox, outline=ring_color, width=int(ring_width_px))
            except TypeError:
                rdraw.ellipse(bbox, outline=ring_color)
            if ring_labels:
                label = f"{int(r_m)} m" if abs(r_m - int(r_m)) < 1e-6 else f"{r_m:.1f} m"
                tx = int(X0 + rp + 6)
                ty = int(Y0 - 10)
                tx = max(4, min(size - 60, tx))
                ty = max(4, min(size - 18, ty))
                rdraw.rectangle((tx - 2, ty - 2, tx + 46, ty + 14),
                                fill=(bg[0], bg[1], bg[2], int(255 * 0.35)))
                rdraw.text((tx, ty), label, fill=(hud[0], hud[1], hud[2], 220))

        if mask:
            base.alpha_composite(ring_layer, dest=(0, 0), source=(0, 0), mask=mask)
        else:
            base.alpha_composite(ring_layer)

    # --- origin marker (dot) ---
    if show_origin and (0 <= X0 < size and 0 <= Y0 < size):
        o_layer = Image.new("RGBA", (size, size), (0, 0, 0, 0))
        odraw = ImageDraw.Draw(o_layer)
        # use origin_thickness_px for dot radius
        orad = max(1, int(origin_thickness_px))
        odraw.ellipse((X0 - orad, Y0 - orad, X0 + orad, Y0 + orad), fill=origin_color)
        base.alpha_composite(o_layer)

    # --- points layer ---
    layer = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    ldraw = ImageDraw.Draw(layer)
    r = max(1, int(point_size))

    for (x, y, z, i) in pts:
        X = size // 2 + int((x - cx) * scale)
        Y = size // 2 - int((y - cy) * scale)
        if not (0 <= X < size and 0 <= Y < size):
            continue
        if z_mode == "height":
            t = 0.0 if zmax == zmin else (z - zmin) / (zmax - zmin)
            color = _turbo(t) + (int(255 * alpha),) if cmap.lower() == "turbo" else _viridis(t) + (int(255 * alpha),)
        elif z_mode == "intensity":
            t = 0.0 if imax == imin else (i - imin) / float(imax - imin)
            color = _turbo(t) + (int(255 * alpha),) if cmap.lower() == "turbo" else _viridis(t) + (int(255 * alpha),)
        else:
            color = fg
        ldraw.ellipse((X - r, Y - r, X + r, Y + r), fill=color)

    if r <= 2:
        layer = layer.filter(ImageFilter.GaussianBlur(radius=0.4))

    if mask:
        base.alpha_composite(layer, dest=(0, 0), source=(0, 0), mask=mask)
    else:
        base.alpha_composite(layer)

    # --- HUD ---
    hud_text = f"{len(pts)} pts | mode={z_mode} | size={point_size}px | alpha={alpha:.2f}"
    box_w = int(min(size, 10 + 8 * len(hud_text)))
    box_h = 24
    draw.rectangle((8, 8, 8 + box_w, 8 + box_h), fill=(hud[0], hud[1], hud[2], 90))
    draw.text((14, 12), hud_text, fill=(hud[0], hud[1], hud[2], 220))

    # --- Encode PNG ---
    buf = io.BytesIO()
    out = base.convert("RGB")
    out.save(buf, format="PNG", optimize=True)

    origin_hdr = request.headers.get("origin", "*")
    headers = {
        "Access-Control-Allow-Origin": origin_hdr,
        "Vary": "Origin",
        "X-Snapshot-Points": str(len(pts)),
        "X-MinMax-X": f"{xmin:.3f},{xmax:.3f}",
        "X-MinMax-Y": f"{ymin:.3f},{ymax:.3f}",
    }
    return Response(content=buf.getvalue(), status_code=200, media_type="image/png", headers=headers)
