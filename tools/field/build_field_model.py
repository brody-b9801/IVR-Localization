"""Build a rectangular-prism model of the V5RC field from the field STEP file.

Every box is axis-aligned in the field frame, given by its min and max
corners, and bounds the actual CAD geometry in that frame (not the part's own
bounding box rotated into it, which oversizes tilted parts).

What becomes a box:
  * floor     one slab covering the foam tiles inside the walls.
  * wall      one prism per side of the perimeter, from its field-facing
              surface out to its outer edge and from its lowest to its highest
              point. The ends of the +-Y walls cover the corners.
  * alliance_bar  one box per alliance bar assembly.
  * toggle    one box per toggle stack (its blocks and the AprilTags on them).
  * loader    each match loader is split so no box enters the wall: the tube
              above the gap elements are pulled out through, the base behind
              that gap, and the hook that sits over the wall top.
The toggles' base plates (which sit inside the foam tiles) and floor tape are
left out, as are fasteners. Game elements are left out by default too, since they move during
a match; with --include-game-elements each element becomes one box, flagged
"dynamic".

Output frame ("field frame"), inches:
    origin  field centre, on the top surface of the foam tiles
    +X      towards the blue alliance station   (STEP +X)
    +Y      STEP -Z
    +Z      up                                    (STEP +Y)

    python build_field_model.py <field.STEP> <out.json> [--include-game-elements]
"""

import argparse
import json
import re
from collections import OrderedDict

import numpy as np

from extract_step import extract, ref
from step_parse import Step

MM_PER_IN = 25.4

# The main body of a match loader; its field-facing wall defines the floor gap.
LOADER_BODY = re.compile(r"^276-9250-031")
# Points within this distance of a face count as lying on it (inches).
LOADER_TOL_IN = 3.0 / MM_PER_IN

# STEP (x, y-up, z) -> field (x, y, z-up); proper rotation (det = +1).
STEP_TO_FIELD_ROT = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])

# Fasteners, springs, washers, and the clips on the outside of the wall behind
# the loaders: too small to matter for a field model.
HARDWARE = re.compile(
    r"^(SBT0832|THF0832|NNY_|SPTM3|94629A270|276-7596-003$|276-7596-004|276-9250-027$"
    r"|276-8354-404$)"
)

# (regex on the leaf part name, category)
CATEGORIES = [
    (r"^276-6904-001$", "floor"),
    # panels, rails, posts and brackets of the perimeter
    (r"^(276-7596-01[45]|276-7596-35[01]_|276-7596-31[67]|276-7596-0(05|06|08|09|10)$)",
     "wall"),
    # toggle blocks and the AprilTags on them
    (r"^276-9250-00[3-6]", "toggle"),
    # the plates under the toggles, inside the foam tiles
    (r"^276-4847-011$", "toggle_base"),
    (r"^276-9250-00[12]_", "game_element"),
    (r"^276-9250-03[1-6]", "loader"),
    (r"^276-9250-02[1-4]", "alliance_bar"),
    (r"^(Tape-|Wall Alliance Stations)", "tape"),
]
EXCLUDED_CATEGORIES = {"toggle_base", "tape"}

# Perimeter parts whose field-facing side is the wall surface (panels and
# rails); posts and corner brackets only stick out locally.
WALL_FACE = re.compile(r"^(276-7596-01[45]|276-7596-35[01]_)")

# Categories bounded as one box per enclosing assembly instance, found by the
# innermost assembly in the part's path matching the regex.
GROUP_BY = {
    "game_element": r"^276-9250-8[01]x",
    "alliance_bar": r"^276-9250-120",
    "loader": r"^276-9250-830",
    "toggle": r"^276-9250-840",
}

# Categories that a range sensor on the robot could actually hit.
SOLID_CATEGORIES = {"wall", "loader", "alliance_bar", "toggle", "game_element"}
# Game elements move during a match; flag them so consumers can drop them.
DYNAMIC_CATEGORIES = {"game_element"}

COLORS = {
    "floor": "#6b6f75",
    "wall": "#bcd7e0",
    "loader": "#9aa0a6",
    "alliance_bar": "#8f8f8f",
    "toggle": "#d9d9d9",
}
GAME_ELEMENT_COLORS = {
    "276-9250-80x_Red-Neutral": "#d0312d",
    "276-9250-80x_Blue-Neutral": "#1f5fbf",
    "276-9250-80x_Red-Blue": "#8e44ad",
    "276-9250-80x_Neutral-Neutral": "#f2c230",
}


def categorize(leaf):
    for pat, cat in CATEGORIES:
        if re.search(pat, leaf):
            return cat
    return None


def color_for(category, name, names):
    """Display colour (the STEP file itself carries no real colours). `names`
    are the box's assembly and part names."""
    if category == "game_element":
        return GAME_ELEMENT_COLORS.get(name, "#4a4d52")
    if category in ("loader", "alliance_bar", "toggle"):
        joined = " ".join(names)
        if "Red" in joined:
            return "#d0312d"
        if "Blue" in joined:
            return "#1f5fbf"
    return COLORS.get(category, "#999999")


def group_of(record, category):
    """(key, name, path) of the box a part belongs to."""
    path, occ = record["path"], record["occ"]
    pattern = GROUP_BY.get(category)
    if pattern is None:
        return (category, tuple(occ)), path[-1], path
    for i in range(len(path) - 1, 0, -1):
        if re.search(pattern, path[i]):
            # occ[i - 1] is the occurrence of path[i]
            return (category, tuple(occ[:i])), path[i], path[:i + 1]
    raise ValueError(f"{category} part outside a {pattern} assembly: {' / '.join(path)}")


def wall_faces(parts):
    """{(axis, sign): coordinate of that wall's field-facing surface}, where the
    wall on side `sign` of `axis` holds the parts centred on that side."""
    faces = {}
    for ax in (0, 1):
        for sign in (-1, 1):
            side = [(lo, hi) for leaf, lo, hi in parts if WALL_FACE.search(leaf)
                    and sign * (lo + hi)[ax] > abs((lo + hi)[1 - ax])]
            faces[ax, sign] = (min(lo[ax] for lo, _ in side) if sign > 0
                               else max(hi[ax] for _, hi in side))
    return faces


def wall_boxes(parts, faces):
    """One (name, min, max) box per side of the perimeter."""
    lo = np.min([p[1] for p in parts], axis=0)
    hi = np.max([p[2] for p in parts], axis=0)
    out = []
    for ax, sign in [(0, 1), (0, -1), (1, 1), (1, -1)]:
        b_lo, b_hi = lo.copy(), hi.copy()
        if sign > 0:
            b_lo[ax] = faces[ax, sign]
        else:
            b_hi[ax] = faces[ax, sign]
        if ax == 0:  # the +-Y walls run the full length and cover the corners
            b_lo[1], b_hi[1] = faces[1, -1], faces[1, 1]
        out.append((f"{'+' if sign > 0 else '-'}{'XY'[ax]} wall", b_lo, b_hi))
    return out


def loader_boxes(pts, body_pts, faces):
    """Split a match loader into (name, min, max) boxes that stay out of the wall.

    pts: field-frame points of all the loader's parts; body_pts: those of its
    main body, whose field-facing wall stops short of the floor, leaving the
    gap elements are pulled out through.
    """
    c = pts.mean(0)
    ax = 0 if abs(c[0]) > abs(c[1]) else 1  # the loader backs onto the wall on this axis
    lat = 1 - ax
    sign = 1 if c[ax] > 0 else -1
    tol = LOADER_TOL_IN

    def into(p):  # distance into the field, measured from the wall surface
        return sign * (faces[ax, sign] - p[:, ax])

    body_into = into(body_pts)
    front = body_into >= body_into.max() - tol
    gap_top = body_pts[front, 2].min()

    d = into(pts)
    z = pts[:, 2]
    inside = d > tol

    def box(name, sel, d_lo, d_hi, z_lo, z_hi):
        lo, hi = np.zeros(3), np.zeros(3)
        lo[ax], hi[ax] = sorted((faces[ax, sign] - sign * d_lo, faces[ax, sign] - sign * d_hi))
        lo[lat], hi[lat] = pts[sel, lat].min(), pts[sel, lat].max()
        lo[2], hi[2] = z_lo, z_hi
        return name, lo, hi

    tube = inside & (z > gap_top - tol)
    out = [box("tube", tube, 0.0, d.max(), gap_top, z[tube].max())]
    base = inside & (z < gap_top - tol)
    if base.any():
        out.append(box("base", base, 0.0, d[base].max(), z.min(), gap_top))
    hook = d < -tol
    if hook.any():
        out.append(box("hook", hook, d[hook].min(), 0.0, z[hook].min(), z[hook].max()))
    return out


def build(step_path, include_game_elements=False):
    s = Step(step_path)
    records = extract(step_path, s)

    # Floor height = top of the foam tiles, in STEP coordinates (STEP +Y is up).
    tile_tops = [r["world_max"][1] for r in records if categorize(r["path"][-1]) == "floor"]
    floor_y = float(np.median(tile_tops))

    step_to_field = np.eye(4)
    step_to_field[:3, :3] = STEP_TO_FIELD_ROT / MM_PER_IN
    step_to_field[:3, 3] = STEP_TO_FIELD_ROT @ np.array([0, -floor_y, 0]) / MM_PER_IN

    def to_field(lo, hi):
        # step_to_field only permutes and scales axes, so boxes map to boxes
        c = np.array([[x, y, z, 1] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                      for z in (lo[2], hi[2])])
        f = (step_to_field @ c.T)[:3].T
        return f.min(0), f.max(0)

    def field_points(r):
        p = np.array(s.shell_points(ref(s.get(r["solid_id"])[1][1])))
        m = step_to_field @ np.array(r["xform"])
        return p @ m[:3, :3].T + m[:3, 3]

    # Group the parts into boxes.
    groups = OrderedDict()
    skipped, excluded = {}, {}
    for r in records:
        leaf = r["path"][-1]
        if HARDWARE.search(leaf):
            skipped[leaf] = skipped.get(leaf, 0) + 1
            continue
        cat = categorize(leaf)
        if cat is None:
            raise ValueError(f"uncategorised part: {leaf}")
        if cat in EXCLUDED_CATEGORIES or (cat in DYNAMIC_CATEGORIES and not include_game_elements):
            excluded[cat] = excluded.get(cat, 0) + 1
            continue
        key, name, path = group_of(r, cat)
        g = groups.setdefault(key, {"category": cat, "name": name, "path": path, "parts": []})
        g["parts"].append((r, *to_field(r["world_min"], r["world_max"])))

    def make_box(cat, part, assembly, color, lo, hi):
        return {
            "category": cat,
            "part": part,
            "assembly": assembly,
            "solid": cat in SOLID_CATEGORIES,
            "dynamic": cat in DYNAMIC_CATEGORIES,
            "color": color,
            "min": np.round(lo, 4).tolist(),
            "max": np.round(hi, 4).tolist(),
        }

    by_cat = {}
    for g in groups.values():
        by_cat.setdefault(g["category"], []).append(g)

    walls = [(r["path"][-1], lo, hi) for g in by_cat["wall"] for r, lo, hi in g["parts"]]
    faces = wall_faces(walls)
    perimeter = by_cat["wall"][0]["path"][1]
    boxes = []

    tiles = [(lo, hi) for g in by_cat["floor"] for _, lo, hi in g["parts"]]
    floor_lo = np.array([faces[0, -1], faces[1, -1], min(lo[2] for lo, _ in tiles)])
    floor_hi = np.array([faces[0, 1], faces[1, 1], max(hi[2] for _, hi in tiles)])
    boxes.append(make_box("floor", "foam tiles", perimeter, COLORS["floor"], floor_lo, floor_hi))

    for name, lo, hi in wall_boxes(walls, faces):
        boxes.append(make_box("wall", name, perimeter, COLORS["wall"], lo, hi))

    for cat in ("alliance_bar", "loader", "toggle", "game_element"):
        for g in by_cat.get(cat, []):
            names = [g["name"]] + [r["path"][-1] for r, _, _ in g["parts"]]
            color = color_for(cat, g["name"], names)
            assembly = " / ".join(g["path"][1:-1])
            if cat == "loader":
                pts = [(LOADER_BODY.search(r["path"][-1]) is not None, field_points(r))
                       for r, _, _ in g["parts"]]
                body = [p for is_body, p in pts if is_body]
                if not body:
                    raise ValueError(f"match loader without a loader body: {g['name']}")
                for piece, lo, hi in loader_boxes(np.vstack([p for _, p in pts]),
                                                  np.vstack(body), faces):
                    boxes.append(make_box(cat, f"{g['name']} {piece}", assembly, color, lo, hi))
            else:
                lo = np.min([lo for _, lo, _ in g["parts"]], axis=0)
                hi = np.max([hi for _, _, hi in g["parts"]], axis=0)
                boxes.append(make_box(cat, g["name"], assembly, color, lo, hi))

    boxes = [{"id": i, **b} for i, b in enumerate(boxes)]

    return {
        "description": "V5RC field approximated by rectangular prisms: the floor, one prism per "
                       "wall, alliance bars, match loaders and toggles. Toggle base plates, floor "
                       "tape and hardware are excluded; "
                       + ("game elements are one box each (dynamic)." if include_game_elements
                          else "so are game elements."),
        "source": step_path.replace("\\", "/").split("/")[-1],
        "units": "in",
        "frame": {
            "origin": "field centre, top surface of the foam tiles",
            "x": "towards the blue alliance station (STEP +X)",
            "y": "STEP -Z",
            "z": "up (STEP +Y)",
        },
        "step_to_field": step_to_field.round(8).tolist(),
        "box_format": "every box is axis-aligned in the field frame: min = [x, y, z] of its "
                      "lowest corner, max = [x, y, z] of its highest corner",
        "skipped_hardware": dict(sorted(skipped.items())),
        "excluded_parts": dict(sorted(excluded.items())),
        "boxes": boxes,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step")
    ap.add_argument("out")
    ap.add_argument("--include-game-elements", action="store_true",
                    help="also emit one box per game element (flagged dynamic)")
    args = ap.parse_args()
    out_path = args.out
    model = build(args.step, args.include_game_elements)
    with open(out_path, "w") as f:
        json.dump(model, f, indent=1)
    counts = {}
    for b in model["boxes"]:
        counts[b["category"]] = counts.get(b["category"], 0) + 1
    print(f"{len(model['boxes'])} boxes -> {out_path}")
    for k, v in sorted(counts.items()):
        print(f"  {k:18s} {v}")


if __name__ == "__main__":
    main()
