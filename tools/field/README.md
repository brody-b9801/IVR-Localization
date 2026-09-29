# Field prism model

Builds a rectangular-prism approximation of the V5RC field from the field CAD
(`Resources/v5rc-override-fieldcad/*.STEP`) and renders it.

```
pip install numpy pillow
python build_field_model.py "../../Resources/v5rc-override-fieldcad/276-9250-000 (2026-04-26).STEP" ../../Resources/field-model/field_boxes.json
python render_field.py ../../Resources/field-model/field_boxes.json ../../Resources/field-model/renders
python render_field.py ../../Resources/field-model/field_boxes.json out --custom -60 30 --persp 35   # match another camera
```

- `step_parse.py` / `extract_step.py` read the STEP assembly directly (no CAD kernel):
  instance transforms plus each solid's bounds from its edges (arcs and splines included).
- `build_field_model.py` makes one axis-aligned box per part instance (the field-frame bounds
  of the part's own CAD bounding box, so angled parts come out slightly oversized). Fasteners
  and game elements are skipped, since game elements move during a match
  (`--include-game-elements` adds one box per element, flagged `dynamic`). Tape and
  alliance-station outlines are split into one thin box per straight strip, and diagonal
  strips into short pieces. Each match loader is one box whose bottom is raised to the
  bottom edge of its field-facing wall, leaving the gap elements are pulled out through.
- `render_field.py` is a z-buffered software renderer (numpy + Pillow).

## Frame and format

Inches. Origin at field centre on the top of the foam tiles, +Z up, +X towards the blue
alliance station, +Y = STEP -Z. `step_to_field` in the JSON maps STEP mm to this frame.

Each box has `category`, `part`, `assembly`, `solid` (a range sensor could hit it), `dynamic`
(game element, moves during a match), `color`, and its corners `min` = [x, y, z] and
`max` = [x, y, z]. Every box is axis-aligned; there is no rotation.
