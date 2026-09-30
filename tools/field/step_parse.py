"""Minimal STEP (ISO 10303-21) reader for extracting assembly placement and
per-solid bounding geometry from the VEX field CAD.

Only the entity types needed to walk the assembly tree and bound each solid
are interpreted; everything else is kept as raw text and parsed lazily.
"""

import math
import re

import numpy as np

_REF = re.compile(r"#(\d+)")
_TYPE = re.compile(r"\s*([A-Z0-9_]+)\s*\(")


def load_entities(path):
    """Return {id: raw_text} for every entity in the DATA section."""
    ents = {}
    buf = []
    in_data = False
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if not in_data:
                if line.startswith("DATA;"):
                    in_data = True
                continue
            if line.startswith("ENDSEC;"):
                break
            buf.append(line.rstrip("\n"))
            if line.rstrip().endswith(";"):
                rec = " ".join(buf).strip()
                buf = []
                if not rec.startswith("#"):
                    continue
                eq = rec.index("=")
                eid = int(rec[1:eq])
                ents[eid] = rec[eq + 1 :].strip().rstrip(";").strip()
    return ents


def _tokenize(s):
    i, n = 0, len(s)
    while i < n:
        c = s[i]
        if c.isspace():
            i += 1
        elif c in "(),":
            yield c
            i += 1
        elif c == "'":
            j = i + 1
            while True:
                j = s.index("'", j)
                if j + 1 < n and s[j + 1] == "'":
                    j += 2
                    continue
                break
            yield ("str", s[i + 1 : j].replace("''", "'"))
            i = j + 1
        elif c == "#":
            j = i + 1
            while j < n and s[j].isdigit():
                j += 1
            yield ("ref", int(s[i + 1 : j]))
            i = j
        elif c == ".":
            j = s.index(".", i + 1)
            yield ("enum", s[i + 1 : j])
            i = j + 1
        elif c in "$*":
            yield ("null", None)
            i += 1
        elif c.isalpha() or c == "_":
            j = i
            while j < n and (s[j].isalnum() or s[j] == "_"):
                j += 1
            yield ("kw", s[i:j])
            i = j
        else:
            j = i
            while j < n and s[j] not in "(), \t":
                j += 1
            yield ("num", float(s[i:j]))
            i = j


def parse_params(s):
    """Parse a parenthesised STEP parameter list into nested Python lists.

    Refs become ('ref', id); typed values like LENGTH_MEASURE(1.) become
    ('typed', name, [args]).
    """
    toks = list(_tokenize(s))
    pos = 0

    def parse_list():
        nonlocal pos
        assert toks[pos] == "("
        pos += 1
        out = []
        while toks[pos] != ")":
            if toks[pos] == ",":
                pos += 1
                continue
            out.append(parse_value())
        pos += 1
        return out

    def parse_value():
        nonlocal pos
        t = toks[pos]
        if t == "(":
            return parse_list()
        pos += 1
        kind, val = t
        if kind == "ref":
            return ("ref", val)
        if kind == "kw":
            if pos < len(toks) and toks[pos] == "(":
                return ("typed", val, parse_list())
            return val
        return val

    return parse_list()


class Step:
    def __init__(self, path):
        self.raw = load_entities(path)
        self._cache = {}

    def type_of(self, eid):
        m = _TYPE.match(self.raw[eid])
        return m.group(1) if m else "COMPLEX"

    def get(self, eid):
        """Return (type, params) for a simple entity, or
        ('COMPLEX', {subtype: params}) for a complex one."""
        if eid in self._cache:
            return self._cache[eid]
        txt = self.raw[eid]
        m = _TYPE.match(txt)
        if m:
            res = (m.group(1), parse_params(txt[m.end() - 1 :]))
        else:
            parts = parse_params(txt)  # list of ('typed', name, args)
            res = ("COMPLEX", {p[1]: p[2] for p in parts if isinstance(p, tuple)})
        self._cache[eid] = res
        return res

    def ids_of_type(self, name):
        pat = re.compile(r"\s*" + name + r"\s*\(")
        return [i for i, t in self.raw.items() if pat.match(t)]

    def complex_ids_with(self, name):
        return [i for i, t in self.raw.items() if t.startswith("(") and name in t]

    # ---- geometry helpers -------------------------------------------------

    def point(self, eid):
        _, p = self.get(eid)
        return np.array(p[1], dtype=float)

    def direction(self, eid):
        _, p = self.get(eid)
        v = np.array(p[1], dtype=float)
        return v / np.linalg.norm(v)

    def axis2(self, eid):
        """AXIS2_PLACEMENT_3D -> 4x4 homogeneous matrix (local -> parent)."""
        _, p = self.get(eid)
        o = self.point(p[1][1])
        z = self.direction(p[2][1]) if p[2] != "$" and p[2] is not None else np.array([0, 0, 1.0])
        if p[3] is not None and p[3] != "$":
            x = self.direction(p[3][1])
        else:
            x = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
        x = x - z * np.dot(x, z)
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        m = np.eye(4)
        m[:3, 0], m[:3, 1], m[:3, 2], m[:3, 3] = x, y, z, o
        return m

    def edge_points(self, edge_id, arc_samples=24):
        """Points bounding one EDGE_CURVE (vertices + curve bulge)."""
        _, p = self.get(edge_id)
        v1 = self.point(self.get(p[1][1])[1][1][1])
        v2 = self.point(self.get(p[2][1])[1][1][1])
        curve_id = p[3][1]
        same_sense = p[4] == "T"
        ctype, cp = self.get(curve_id)
        pts = [v1, v2]
        if ctype == "CIRCLE" or ctype == "ELLIPSE":
            m = self.axis2(cp[1][1])
            c, x, y = m[:3, 3], m[:3, 0], m[:3, 1]
            if ctype == "CIRCLE":
                ra = rb = float(cp[2])
            else:
                ra, rb = float(cp[2]), float(cp[3])

            def ang(v):
                d = v - c
                return math.atan2(np.dot(d, y) / rb, np.dot(d, x) / ra)

            a1, a2 = ang(v1), ang(v2)
            if not same_sense:
                a1, a2 = a2, a1
            span = (a2 - a1) % (2 * math.pi)
            if span < 1e-9:
                span = 2 * math.pi  # closed curve (single vertex)
            for k in range(arc_samples + 1):
                a = a1 + span * k / arc_samples
                pts.append(c + ra * math.cos(a) * x + rb * math.sin(a) * y)
        elif ctype == "COMPLEX" or ctype.startswith("B_SPLINE"):
            # control points bound the curve (convex hull property)
            if ctype == "COMPLEX":
                bs = cp.get("B_SPLINE_CURVE")
                ctrl = bs[1] if bs else []
            else:
                ctrl = cp[2]
            pts.extend(self.point(r[1]) for r in ctrl)
        return pts

    def shell_points(self, shell_id):
        """All bounding points of a CLOSED_SHELL / OPEN_SHELL."""
        _, sp = self.get(shell_id)
        pts = []
        seen = set()
        for face_ref in sp[1]:
            _, fp = self.get(face_ref[1])
            for bound_ref in fp[1]:
                _, bp = self.get(bound_ref[1])
                _, lp = self.get(bp[1][1])  # EDGE_LOOP / VERTEX_LOOP
                ltype = self.type_of(bp[1][1])
                if ltype == "VERTEX_LOOP":
                    pts.append(self.point(self.get(lp[1][1])[1][1][1]))
                    continue
                for oe in lp[1]:
                    _, op = self.get(oe[1])
                    eid = op[3][1]
                    if eid in seen:
                        continue
                    seen.add(eid)
                    pts.extend(self.edge_points(eid))
        return pts
