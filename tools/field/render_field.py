"""Render the rectangular-prism field model (field_boxes.json) to PNG.

A small z-buffered software rasteriser (numpy + Pillow) so the only
dependencies are numpy and pillow. Flat Lambert shading, dark box outlines,
translucent polycarbonate panels, 2x supersampled.

    python render_field.py field_boxes.json out_dir [--views iso top ...]
                           [--exclude-dynamic] [--width 1600] [--height 1000]
    python render_field.py field_boxes.json out_dir --custom AZ EL [--persp FOV]

Angles are in the field frame (z up): azimuth measured from +X towards +Y,
elevation above the floor plane. The camera sits in that direction from the
field centre, looking back at it.
"""

import argparse
import json
import math
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

# name: (azimuth deg, elevation deg, perspective fov deg or None, aspect h/w, title)
VIEWS = {
    # STEP is Y-up, so a CAD "isometric" looks from STEP (+1,+1,+1) = field (+1,-1,+1).
    "iso": (-45.0, 35.264, None, 0.625, "Isometric (CAD default, from STEP +X+Y+Z)"),
    "iso_red": (-135.0, 35.264, None, 0.625, "Isometric from red-alliance corner"),
    "persp": (-120.0, 28.0, 40.0, 0.625, "Perspective, red-alliance side"),
    "top": (-90.0, 90.0, None, 0.625, "Top (+Y up on page)"),
    "front": (-90.0, 0.0, None, 0.3, "Front elevation (looking +Y)"),
    "side": (180.0, 0.0, None, 0.3, "Side elevation (looking +X, red side nearest)"),
}

# Box face corner indices; corners are ordered by bits (x, y, z) of local axes.
FACES = [(0, 2, 6, 4), (1, 5, 7, 3), (0, 4, 5, 1), (2, 3, 7, 6), (0, 1, 3, 2), (4, 6, 7, 5)]
EDGES = [(i, j) for i in range(8) for j in range(i + 1, 8) if bin(i ^ j).count("1") == 1]
TRANSLUCENT = {"perimeter_panel": 0.35}


def box_corners(b):
    signs = np.array([[(i >> 0) & 1, (i >> 1) & 1, (i >> 2) & 1] for i in range(8)]) - 0.5
    if "rotation" in b:
        r = np.array(b["rotation"])
        return np.array(b["center"]) + (signs * np.array(b["size"])) @ r.T
    lo, hi = np.array(b["min"]), np.array(b["max"])
    return (lo + hi) / 2 + signs * (hi - lo)


def hex_rgb(h):
    h = h.lstrip("#")
    return np.array([int(h[i:i + 2], 16) for i in (0, 2, 4)], dtype=float) / 255.0


class Camera:
    def __init__(self, az, el, fov, target, radius, w, h):
        az, el = math.radians(az), math.radians(el)
        d = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        self.fwd = -d
        up = np.array([0, 0, 1.0]) if abs(d[2]) < 0.999 else np.array([0, 1.0, 0])
        self.right = np.cross(self.fwd, up)
        self.right /= np.linalg.norm(self.right)
        self.up = np.cross(self.right, self.fwd)
        self.w, self.h = w, h
        self.persp = fov is not None
        if self.persp:
            f = math.radians(fov)
            dist = radius / math.sin(f / 2)
            self.eye = target + d * dist
            self.focal = (min(w, h) / 2) / math.tan(f / 2)
        else:
            self.eye = target + d * radius * 3
            self.scale = min(w, h) / (2 * radius)

        self.zoom, self.shift = 1.0, np.zeros(2)

    def project(self, p):
        """World points (N,3) -> screen x, y (pixels) and depth (larger = farther)."""
        v = p - self.eye
        xc, yc, zc = v @ self.right, v @ self.up, v @ self.fwd
        if self.persp:
            s = self.focal / np.maximum(zc, 1e-6)
            sx, sy, depth = xc * s, -yc * s, -1.0 / zc
        else:
            sx, sy, depth = xc * self.scale, -yc * self.scale, zc
        sx = self.w / 2 + sx * self.zoom + self.shift[0]
        sy = self.h / 2 + sy * self.zoom + self.shift[1]
        return np.stack([sx, sy, depth], axis=1)


def fit_camera(points, az, el, fov, w, h, margin=0.94):
    """Camera looking at the model centre, zoomed so the model fills the frame."""
    target = (points.min(0) + points.max(0)) / 2
    radius = np.linalg.norm(points - target, axis=1).max()
    cam = Camera(az, el, fov, target, radius, w, h)
    s = cam.project(points)
    lo, hi = s[:, :2].min(0), s[:, :2].max(0)
    cam.zoom = margin * min(w / (hi[0] - lo[0]), h / (hi[1] - lo[1]))
    s = cam.project(points)
    lo, hi = s[:, :2].min(0), s[:, :2].max(0)
    cam.shift = np.array([w / 2, h / 2]) - (lo + hi) / 2
    return cam


def raster_tri(tri, color, zbuf, cbuf, alpha=None):
    """Fill one screen-space triangle (3x3: x, y, depth)."""
    h, w = zbuf.shape
    x0 = max(int(math.floor(tri[:, 0].min())), 0)
    x1 = min(int(math.ceil(tri[:, 0].max())), w - 1)
    y0 = max(int(math.floor(tri[:, 1].min())), 0)
    y1 = min(int(math.ceil(tri[:, 1].max())), h - 1)
    if x0 > x1 or y0 > y1:
        return
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = tri
    area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    if abs(area) < 1e-9:
        return
    px, py = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
    w0 = ((bx - px) * (cy - py) - (by - py) * (cx - px)) / area
    w1 = ((cx - px) * (ay - py) - (cy - py) * (ax - px)) / area
    w2 = 1.0 - w0 - w1
    inside = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
    if not inside.any():
        return
    z = w0 * az + w1 * bz + w2 * cz
    zs = zbuf[y0:y1 + 1, x0:x1 + 1]
    win = inside & (z < zs)
    if alpha is None:
        zs[win] = z[win]
        cbuf[y0:y1 + 1, x0:x1 + 1][win] = color
    else:
        cs = cbuf[y0:y1 + 1, x0:x1 + 1]
        cs[win] = cs[win] * (1 - alpha) + color * alpha


def raster_line(p, q, color, zbuf, cbuf, bias):
    h, w = zbuf.shape
    n = int(max(abs(q[0] - p[0]), abs(q[1] - p[1]))) + 2
    t = np.linspace(0, 1, n)
    pts = p[None, :] + (q - p)[None, :] * t[:, None]
    xi = np.floor(pts[:, 0]).astype(int)
    yi = np.floor(pts[:, 1]).astype(int)
    ok = (xi >= 0) & (xi < w) & (yi >= 0) & (yi < h)
    xi, yi, z = xi[ok], yi[ok], pts[ok, 2]
    vis = z <= zbuf[yi, xi] + bias
    cbuf[yi[vis], xi[vis]] = color


def render(model, az, el, fov, width, height, exclude_dynamic=False, ss=2):
    boxes = [b for b in model["boxes"] if not (exclude_dynamic and b["dynamic"])]
    corners = [box_corners(b) for b in boxes]
    allpts = np.vstack(corners)
    W, H = width * ss, height * ss
    cam = fit_camera(allpts, az, el, fov, W, H)

    light = -cam.fwd * 0.55 + cam.right * -0.35 + np.array([0, 0, 0.75])
    light /= np.linalg.norm(light)

    zbuf = np.full((H, W), np.inf)
    cbuf = np.ones((H, W, 3)) * np.array([0.97, 0.97, 0.98])
    depth_span = None

    opaque, clear = [], []
    for b, c in zip(boxes, corners):
        (clear if b["category"] in TRANSLUCENT else opaque).append((b, c))

    def shaded_faces(b, c):
        base = hex_rgb(b["color"])
        center = c.mean(0)
        scr = cam.project(c)
        for f in FACES:
            q = c[list(f)]
            n = np.cross(q[1] - q[0], q[3] - q[0])
            nn = np.linalg.norm(n)
            if nn < 1e-12:
                n = q.mean(0) - center
                nn = np.linalg.norm(n)
                if nn < 1e-12:
                    continue
            n = n / nn
            if np.dot(n, q.mean(0) - center) < 0:
                n = -n
            shade = 0.38 + 0.62 * max(0.0, float(np.dot(n, light)))
            col = np.clip(base * shade, 0, 1)
            s = scr[list(f)]
            yield s, col

    for b, c in opaque:
        for s, col in shaded_faces(b, c):
            raster_tri(s[[0, 1, 2]], col, zbuf, cbuf)
            raster_tri(s[[0, 2, 3]], col, zbuf, cbuf)

    finite = zbuf[np.isfinite(zbuf)]
    depth_span = finite.max() - finite.min() if finite.size else 1.0
    bias = depth_span * 2e-3

    for b, c in clear:
        for s, col in shaded_faces(b, c):
            raster_tri(s[[0, 1, 2]], col, zbuf, cbuf, alpha=TRANSLUCENT[b["category"]])
            raster_tri(s[[0, 2, 3]], col, zbuf, cbuf, alpha=TRANSLUCENT[b["category"]])

    edge_col = np.array([0.12, 0.13, 0.15])
    for b, c in opaque + clear:
        scr = cam.project(c)
        ec = edge_col if b["category"] not in TRANSLUCENT else hex_rgb(b["color"]) * 0.55
        for i, j in EDGES:
            raster_line(scr[i], scr[j], ec, zbuf, cbuf, bias)

    img = Image.fromarray((np.clip(cbuf, 0, 1) * 255).astype(np.uint8))
    return img.resize((width, height), Image.LANCZOS)


def label(img, text):
    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("arial.ttf", 18)
    except OSError:
        font = ImageFont.load_default()
    d.text((12, 10), text, fill=(30, 30, 35), font=font)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("model")
    ap.add_argument("out_dir")
    ap.add_argument("--views", nargs="+", default=list(VIEWS), choices=list(VIEWS))
    ap.add_argument("--custom", nargs=2, type=float, metavar=("AZ", "EL"))
    ap.add_argument("--persp", type=float, metavar="FOV", help="perspective fov for --custom")
    ap.add_argument("--exclude-dynamic", action="store_true", help="hide game elements")
    ap.add_argument("--width", type=int, default=1600)
    ap.add_argument("--height", type=int, default=1000, help="only used with --custom")
    ap.add_argument("--no-labels", action="store_true")
    args = ap.parse_args()

    with open(args.model) as f:
        model = json.load(f)
    os.makedirs(args.out_dir, exist_ok=True)

    if args.custom:
        az, el = args.custom
        aspect = args.height / args.width
        jobs = [(f"custom_az{az:g}_el{el:g}", az, el, args.persp, aspect, f"az {az:g}, el {el:g}")]
    else:
        jobs = [(n, *VIEWS[n]) for n in args.views]

    suffix = "_static" if args.exclude_dynamic else ""
    n_boxes = sum(not (args.exclude_dynamic and b["dynamic"]) for b in model["boxes"])
    for name, az, el, fov, aspect, title in jobs:
        height = args.height if args.custom else round(args.width * aspect)
        img = render(model, az, el, fov, args.width, height, args.exclude_dynamic)
        if not args.no_labels:
            label(img, f"{title}  -  {n_boxes} prisms")
        path = os.path.join(args.out_dir, f"field_{name}{suffix}.png")
        img.save(path)
        print("wrote", path)


if __name__ == "__main__":
    main()
