"""Build a rectangular-prism model of the V5RC field from the field STEP file.

Every box is axis-aligned in the field frame, given by its min and max
corners. Each non-hardware part instance in the STEP assembly becomes one box:
the field-frame bounding box of the part's own CAD bounding box. Parts that sit
at an angle therefore come out a little oversized; that is accepted as an
approximation.

Special cases:
  * Floor markings (tape lines, alliance-station outlines) are thin sheets
    whose bounding box would be meaningless, so they are split into one box
    per straight strip. Diagonal strips are further cut into short pieces so
    their axis-aligned boxes stay close to the line.
  * Each match loader is a single box. The loader's field-facing wall stops
    short of the floor, leaving a gap elements are pulled out through, so the
    box's bottom is raised to the bottom edge of that wall (measured from the
    CAD for each loader).

Output frame ("field frame"), inches:
    origin  field centre, on the top surface of the foam tiles
    +X      towards the blue alliance station   (STEP +X)
    +Y      STEP -Z
    +Z      up                                    (STEP +Y)

Game elements are left out by default: they move during a match, so they
don't belong in a localisation map. With --include-game-elements each element
(two coloured halves in the CAD) becomes one box, flagged "dynamic".

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

# Diagonal strips are cut into pieces about this many strip-widths long.
DIAGONAL_PIECE_WIDTHS = 4

# The main body of a match loader; its field-facing wall defines the floor gap.
LOADER_BODY = re.compile(r"^276-9250-031")
# Points within this distance of the loader's field-facing extreme count as
# being on its front wall (mm).
LOADER_FRONT_TOL_MM = 3.0

# STEP (x, y-up, z) -> field (x, y, z-up); proper rotation (det = +1).
STEP_TO_FIELD_ROT = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])

# Fasteners, springs and washers: too small to matter for a field model.
HARDWARE = re.compile(
    r"^(SBT0832|THF0832|NNY_|SPTM3|94629A270|276-7596-003$|276-7596-004|276-9250-027$)"
)

# (regex on the leaf part name, category)
CATEGORIES = [
    (r"^276-6904-001$", "floor_tile"),
    (r"^276-7596-01[45]$", "perimeter_panel"),
    (r"^276-7596-35[01]_", "perimeter_rail"),
    (r"^276-7596-31[67]", "perimeter_post"),
    (r"^(276-7596-0(05|06|08|09|10)|276-8354-404)$", "perimeter_foot"),
    (r"^276-4847-011$", "goal_base"),
    (r"^276-9250-00[345]", "goal"),
    (r"^276-9250-006_AprilTag", "apriltag"),
    (r"^276-9250-00[12]_", "game_element"),
    (r"^276-9250-03[1-6]", "loader"),
    (r"^276-9250-02[1-4]", "alliance_bar"),
    (r"^Tape-", "tape"),
    (r"^Wall Alliance Stations", "alliance_station"),
]

# Categories that a range sensor on the robot could actually hit.
SOLID_CATEGORIES = {
    "perimeter_panel", "perimeter_rail", "perimeter_post", "perimeter_foot",
    "goal_base", "goal", "game_element", "loader", "alliance_bar",
}
# Game elements move during a match; flag them so consumers can drop them.
DYNAMIC_CATEGORIES = {"game_element"}

# Floor markings are decomposed into strips rather than bounded as a whole.
STRIP_CATEGORIES = {"tape", "alliance_station"}

MIN_THICKNESS_IN = 0.02  # give zero-thickness surfaces (tape, tags) some depth


def categorize(leaf):
    for pat, cat in CATEGORIES:
        if re.search(pat, leaf):
            return cat
    return None


def color_for(path, category):
    """Display colour (the STEP file itself carries no real colours)."""
    leaf = path[-1]
    joined = " ".join(path)
    if category == "game_element":
        # one box per element; colour it after the element's non-neutral half
        return {
            "276-9250-80x_Red-Neutral": "#d0312d",
            "276-9250-80x_Blue-Neutral": "#1f5fbf",
            "276-9250-80x_Red-Blue": "#8e44ad",
            "276-9250-80x_Neutral-Neutral": "#f2c230",
        }.get(leaf, "#4a4d52")
    if category in ("tape", "alliance_station", "loader", "alliance_bar", "goal"):
        if "Red" in joined:
            return "#d0312d" if category != "tape" else "#e8e8e8"
        if "Blue" in joined:
            return "#1f5fbf" if category != "tape" else "#e8e8e8"
    return {
        "floor_tile": "#6b6f75",
        "perimeter_panel": "#bcd7e0",
        "perimeter_rail": "#c4c8cc",
        "perimeter_post": "#a9adb2",
        "perimeter_foot": "#3a3c40",
        "goal_base": "#8a8e93",
        "goal": "#d9d9d9",
        "apriltag": "#111111",
        "loader": "#9aa0a6",
        "alliance_bar": "#8f8f8f",
        "tape": "#e8e8e8",
    }.get(category, "#999999")


def signed_permutation(r, tol=1e-6):
    return np.allclose(np.abs(r).max(axis=0), 1.0, atol=tol) and np.allclose(
        np.abs(r).sum(axis=0), 1.0, atol=tol * 3
    )


def box_aabbs(center, rot, size):
    """Axis-aligned (min, max) boxes covering an oriented box (field frame, in).

    A long, thin box lying diagonally (a tape strip) is cut along its length
    first, so the axis-aligned boxes hug the strip instead of covering a
    square the size of its whole diagonal.
    """
    n, long_ax = 1, int(np.argmax(size))
    if not signed_permutation(rot):
        mid = np.sort(size)[1]
        if np.abs(rot[:, long_ax]).max() < 0.99 and size[long_ax] > DIAGONAL_PIECE_WIDTHS * mid > 0:
            n = int(np.ceil(size[long_ax] / (DIAGONAL_PIECE_WIDTHS * mid)))
    piece = size.copy()
    piece[long_ax] = size[long_ax] / n
    half = np.maximum(np.abs(rot) @ piece / 2, MIN_THICKNESS_IN / 2)
    out = []
    for k in range(n):
        c = center + rot[:, long_ax] * (-size[long_ax] / 2 + (k + 0.5) * piece[long_ax])
        out.append((c - half, c + half))
    return out


def loader_gap_top(s, solids, step_to_field):
    """Height (field frame, in) of the bottom edge of a match loader's
    field-facing wall, i.e. the top of the gap under the loader."""
    pts = []
    for sid, xf in solids:
        p = np.array(s.shell_points(ref(s.get(sid)[1][1])))
        m = step_to_field @ xf
        pts.append(p @ m[:3, :3].T + m[:3, 3])
    pts = np.vstack(pts)
    c = pts.mean(0)
    ax = 0 if abs(c[0]) > abs(c[1]) else 1  # the loader backs onto the wall on this axis
    into_field = -np.sign(c[ax]) * pts[:, ax]
    front = into_field >= into_field.max() - LOADER_FRONT_TOL_MM / MM_PER_IN
    return float(pts[front, 2].min())


def overlaps(a, b, tol=0.05):
    return bool(np.all(a[0] <= b[1] + tol) and np.all(b[0] <= a[1] + tol))


def build(step_path, include_game_elements=False):
    s = Step(step_path)
    records = extract(step_path, s)

    # Group solids into box instances: normally one per part occurrence, but a
    # game element (a sub-assembly of two coloured halves) becomes a single box
    # bounding all of its parts in the element assembly's frame.
    instances = OrderedDict()
    for r in records:
        leaf = r["path"][-1]
        lo, hi = np.array(r["local_min"]), np.array(r["local_max"])
        if categorize(leaf) == "game_element" and len(r["occ"]) >= 2:
            key = tuple(r["occ"][:-1])
            frame = np.array(r["parent_xform"])
            name = r["path"][-2]
            # re-express this part's bounds in the element assembly's frame
            t = np.linalg.inv(frame) @ np.array(r["xform"])
            c = np.array([[x, y, z, 1] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                          for z in (lo[2], hi[2])])
            pc = (t @ c.T)[:3].T
            lo, hi = pc.min(0), pc.max(0)
            path = r["path"][:-1]
        else:
            key = tuple(r["occ"])
            frame = np.array(r["xform"])
            name = leaf
            path = r["path"]
        inst = instances.setdefault(key, {"path": path, "leaf": leaf, "name": name,
                                          "occ_root": r["occ"][0], "xform": frame,
                                          "lo": [], "hi": [], "strips": [], "solids": []})
        inst["lo"].append(lo)
        inst["hi"].append(hi)
        inst["strips"].extend(r.get("strips", []))
        inst["solids"].append((r["solid_id"], np.array(r["xform"])))

    # Floor height = top of the foam tiles, in STEP coordinates.
    tile_tops = []
    for inst in instances.values():
        if categorize(inst["leaf"]) == "floor_tile":
            lo, hi = np.min(inst["lo"], 0), np.max(inst["hi"], 0)
            c = np.array([[x, y, z, 1] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                          for z in (lo[2], hi[2])])
            tile_tops.append((inst["xform"] @ c.T)[1].max())
    floor_y = float(np.median(tile_tops))

    step_to_field = np.eye(4)
    step_to_field[:3, :3] = STEP_TO_FIELD_ROT / MM_PER_IN
    step_to_field[:3, 3] = STEP_TO_FIELD_ROT @ np.array([0, -floor_y, 0]) / MM_PER_IN

    def make_box(cat, part, path, color, lo, hi):
        return {
            "category": cat,
            "part": part,
            "assembly": " / ".join(path[1:-1]),
            "solid": cat in SOLID_CATEGORIES,
            "dynamic": cat in DYNAMIC_CATEGORIES,
            "color": color,
            "min": np.round(lo, 4).tolist(),
            "max": np.round(hi, 4).tolist(),
        }

    boxes = []
    loader_parts = []  # (inst, (lo, hi)); merged into one box per loader below
    skipped = {}
    for inst in instances.values():
        leaf = inst["leaf"]
        if HARDWARE.search(leaf):
            skipped[leaf] = skipped.get(leaf, 0) + 1
            continue
        cat = categorize(leaf)
        if cat is None:
            raise ValueError(f"uncategorised part: {leaf}")
        if cat in DYNAMIC_CATEGORIES and not include_game_elements:
            continue

        m = step_to_field @ inst["xform"]  # part frame (mm) -> field frame (in)
        if cat in STRIP_CATEGORIES and inst["strips"]:
            local = [(np.array(st["center"]), np.array(st["axes"]), np.array(st["size"]))
                     for st in inst["strips"]]
        else:
            lo, hi = np.min(inst["lo"], 0), np.max(inst["hi"], 0)
            local = [((lo + hi) / 2, np.eye(3), hi - lo)]

        for c_local, axes, size_mm in local:
            rot = m[:3, :3] * MM_PER_IN @ axes  # box axes in the field frame (orthonormal)
            center = m[:3, :3] @ c_local + m[:3, 3]
            for lo, hi in box_aabbs(center, rot, size_mm / MM_PER_IN):
                if cat == "loader":
                    loader_parts.append((inst, (lo, hi)))
                else:
                    boxes.append(make_box(cat, inst["name"], inst["path"],
                                          color_for(inst["path"], cat), lo, hi))

    # One box per match loader: group the loader's parts (touching boxes within
    # the same loader assembly), then lift the bottom to the top of the gap.
    groups = []
    for inst, bb in loader_parts:
        hits = [g for g in groups if g["root"] == inst["occ_root"]
                and any(overlaps(bb, b) for _, b in g["parts"])]
        merged = {"root": inst["occ_root"], "parts": [(inst, bb)]}
        for g in hits:
            merged["parts"] += g["parts"]
        groups = [g for g in groups if not any(g is h for h in hits)] + [merged]
    for g in groups:
        body = [inst for inst, _ in g["parts"] if LOADER_BODY.search(inst["leaf"])]
        if not body:
            raise ValueError("match loader group without a loader body")
        lo = np.min([b[0] for _, b in g["parts"]], axis=0)
        hi = np.max([b[1] for _, b in g["parts"]], axis=0)
        lo[2] = loader_gap_top(s, [sd for inst in body for sd in inst["solids"]], step_to_field)
        boxes.append(make_box("loader", body[0]["name"], body[0]["path"],
                              color_for(body[0]["path"], "loader"), lo, hi))

    boxes = [{"id": i, **b} for i, b in enumerate(boxes)]

    return {
        "description": "V5RC field approximated by rectangular prisms, one per part instance "
                       "of the field STEP (hardware excluded; "
                       + ("one per game element)." if include_game_elements
                          else "game elements excluded)."),
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
