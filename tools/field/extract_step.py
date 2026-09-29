"""Walk the field STEP assembly and dump every solid instance with its world
transform, colour and local-frame bounding box.

Used as a module by build_field_model.py; run standalone to dump the raw
records to JSON. Units: millimetres, in the STEP file's world frame.

    python extract_step.py <field.STEP> <out.json>
"""

import json
import sys
from collections import defaultdict

import numpy as np

from step_parse import Step


def ref(v):
    return v[1] if isinstance(v, tuple) and v[0] == "ref" else None


def extract(step_path, s=None):
    """Return the list of solid-instance records for the STEP assembly.

    Pass an already-loaded Step as `s` to avoid re-reading the file."""
    s = s or Step(step_path)

    # --- product definitions ------------------------------------------------
    pd_name = {}
    for pd in s.ids_of_type("PRODUCT_DEFINITION"):
        pdf = ref(s.get(pd)[1][2])
        prod = ref(s.get(pdf)[1][2])
        pd_name[pd] = s.get(prod)[1][1]

    # PRODUCT_DEFINITION_SHAPE -> what it describes (PD or NAUO)
    pds_def = {i: ref(s.get(i)[1][2]) for i in s.ids_of_type("PRODUCT_DEFINITION_SHAPE")}

    # PD -> its SHAPE_REPRESENTATION
    pd_rep = {}
    for sdr in s.ids_of_type("SHAPE_DEFINITION_REPRESENTATION"):
        p = s.get(sdr)[1]
        d = pds_def[ref(p[0])]
        if d in pd_name:
            pd_rep[d] = ref(p[1])
    rep_pd = {v: k for k, v in pd_rep.items()}

    # SHAPE_REPRESENTATION -> geometry reps (brep / surface model), untransformed
    rep_links = defaultdict(list)
    for srr in s.ids_of_type("SHAPE_REPRESENTATION_RELATIONSHIP"):
        p = s.get(srr)[1]
        a, b = ref(p[2]), ref(p[3])
        rep_links[a].append(b)
        rep_links[b].append(a)

    # --- assembly occurrences ---------------------------------------------------
    nauo_info = {}
    for n in s.ids_of_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE"):
        p = s.get(n)[1]
        nauo_info[n] = (ref(p[3]), ref(p[4]), p[0])  # parent pd, child pd, id

    children = defaultdict(list)  # parent pd -> [(child pd, 4x4 child->parent, occ id)]
    for cdsr in s.ids_of_type("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION"):
        p = s.get(cdsr)[1]
        rrwt = ref(p[0])
        nauo = pds_def[ref(p[1])]
        parent_pd, child_pd, occ = nauo_info[nauo]
        _, sub = s.get(rrwt)
        rr = sub["REPRESENTATION_RELATIONSHIP"]
        rep1, rep2 = ref(rr[2]), ref(rr[3])
        idt = ref(sub["REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION"][0])
        ip = s.get(idt)[1]
        m1, m2 = s.axis2(ref(ip[2])), s.axis2(ref(ip[3]))
        t = m2 @ np.linalg.inv(m1)
        if rep_pd.get(rep1) == parent_pd and rep_pd.get(rep2) == child_pd:
            t = np.linalg.inv(t)
        else:
            assert rep_pd.get(rep1) == child_pd and rep_pd.get(rep2) == parent_pd
        children[parent_pd].append((child_pd, t, occ))

    # --- colours --------------------------------------------------------------
    def find_rgb(eid, depth=0):
        typ, p = s.get(eid)
        if typ == "COLOUR_RGB":
            return [float(x) for x in p[1:4]]
        if depth > 10:
            return None
        stack = [p]
        while stack:
            v = stack.pop()
            if isinstance(v, list):
                stack.extend(v)
            elif ref(v) is not None:
                c = find_rgb(ref(v), depth + 1)
                if c:
                    return c
        return None

    item_color = {}
    for si in s.ids_of_type("STYLED_ITEM"):
        p = s.get(si)[1]
        item_color[ref(p[2])] = find_rgb(ref(p[1][0]))

    # --- geometry per shape representation ---------------------------------------
    solid_cache = {}

    def solids_of_rep(rep):
        """[(solid entity id, kind, colour, Nx3 points)] in rep's frame."""
        if rep in solid_cache:
            return solid_cache[rep]
        out = []
        for g in [rep] + rep_links.get(rep, []):
            gtype, gp = s.get(g)
            if gtype not in ("ADVANCED_BREP_SHAPE_REPRESENTATION",
                             "MANIFOLD_SURFACE_SHAPE_REPRESENTATION"):
                continue
            for it in gp[1]:
                iid = ref(it)
                ityp, ip = s.get(iid)
                if ityp == "MANIFOLD_SOLID_BREP":
                    pts = np.array(s.shell_points(ref(ip[1])))
                    rec = (iid, "solid", item_color.get(iid), pts)
                    ext = pts.max(0) - pts.min(0)
                    thin = int(np.argmin(ext))
                    if ext[thin] < 3.0:
                        # thin sheet (tape, floor marking): also keep a strip decomposition
                        for sign in (1.0, -1.0):
                            up = np.eye(3)[thin] * sign
                            strips = s.flat_strips(ref(ip[1]), up, float(ext[thin]))
                            if strips:
                                rec = rec + (strips,)
                                break
                    out.append(rec)
                elif ityp == "SHELL_BASED_SURFACE_MODEL":
                    pts = []
                    for sh in ip[1]:
                        pts.extend(s.shell_points(ref(sh)))
                    out.append((iid, "surface", item_color.get(iid), np.array(pts)))
        solid_cache[rep] = out
        return out

    # --- walk ---------------------------------------------------------------------
    all_children = {c for lst in children.values() for c, _, _ in lst}
    roots = [pd for pd in pd_name if pd not in all_children and pd in children]
    assert len(roots) == 1, roots
    records = []

    def walk(pd, xf, path, occs, parent_xf):
        path = path + [pd_name[pd]]
        rep = pd_rep.get(pd)
        if rep is not None:
            for sid, kind, color, pts, *strips in solids_of_rep(rep):
                lo, hi = pts.min(0), pts.max(0)
                rec = {
                    "path": path,
                    "occ": occs,  # assembly occurrence ids, root -> this part instance
                    "solid_id": sid,
                    "kind": kind,
                    "color": color,
                    "xform": xf.tolist(),
                    "parent_xform": parent_xf.tolist(),  # enclosing assembly instance
                    "local_min": lo.tolist(),
                    "local_max": hi.tolist(),
                }
                if strips:
                    rec["strips"] = [{"center": c.tolist(), "axes": a.tolist(), "size": z.tolist()}
                                     for c, a, z in strips[0]]
                records.append(rec)
        for child, t, occ in children.get(pd, []):
            walk(child, xf @ t, path, occs + [occ], xf)

    walk(roots[0], np.eye(4), [], [], np.eye(4))
    return records


if __name__ == "__main__":
    recs = extract(sys.argv[1])
    with open(sys.argv[2], "w") as f:
        json.dump({"units": "mm", "solids": recs}, f)
    print(f"{len(recs)} solid instances written to {sys.argv[2]}")
