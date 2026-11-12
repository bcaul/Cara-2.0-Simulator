"""Lightweight STEP (AP203/AP214/AP242) assembly-structure parser.

Extracts every part *instance* in an assembly together with its world
transform, without needing a CAD kernel. Only the assembly graph is read:

    NEXT_ASSEMBLY_USAGE_OCCURRENCE (parent PD -> child PD)
    PRODUCT_DEFINITION_SHAPE(definition = NAUO)
    CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(rel, pds)
    REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(ITEM_DEFINED_TRANSFORMATION)
    ITEM_DEFINED_TRANSFORMATION(item1, item2)  -> AXIS2_PLACEMENT_3D pair

Geometry (B-reps) is ignored. Lengths are returned in the file's length
unit (Onshape exports metres).
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

_REF = re.compile(r"#(\d+)")


@dataclass
class Instance:
    path: tuple[str, ...]          # product names from root to this instance
    product: str                   # product name of the instance
    T: np.ndarray                  # 4x4 world transform (file length unit)
    is_leaf: bool = True
    children: list = field(default_factory=list)

    @property
    def name(self) -> str:
        return self.product


def _split_args(s: str) -> list[str]:
    """Split a STEP argument list at top-level commas."""
    out, depth, cur, in_str = [], 0, [], False
    for ch in s:
        if ch == "'" :
            in_str = not in_str
        if not in_str:
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
            elif ch == "," and depth == 0:
                out.append("".join(cur).strip())
                cur = []
                continue
        cur.append(ch)
    if cur:
        out.append("".join(cur).strip())
    return out


class StepFile:
    def __init__(self, path: str | Path):
        text = Path(path).read_text(encoding="latin-1")
        data = text.split("DATA;", 1)[1]
        self.ents: dict[int, tuple[str, str]] = {}
        # Records end with ';' at line end; strings may contain ';' rarely - acceptable here.
        for rec in re.split(r";\s*\n", data):
            rec = rec.strip()
            m = re.match(r"#(\d+)\s*=\s*(.*)", rec, re.S)
            if not m:
                continue
            eid, body = int(m.group(1)), m.group(2).replace("\n", "").replace("\r", "")
            if body.startswith("("):  # complex entity: keep raw
                self.ents[eid] = ("__COMPLEX__", body)
            else:
                km = re.match(r"([A-Z0-9_]+)\s*\((.*)\)\s*$", body, re.S)
                if km:
                    self.ents[eid] = (km.group(1), km.group(2))

    # -- helpers ---------------------------------------------------------
    def args(self, eid: int) -> list[str]:
        return _split_args(self.ents[eid][1])

    def ref(self, s: str) -> int:
        return int(s.lstrip("#"))

    def of_type(self, t: str):
        return [(i, b) for i, (k, b) in self.ents.items() if k == t]

    def _vec(self, eid: int) -> np.ndarray:
        kind, body = self.ents[eid]
        a = _split_args(body)
        nums = re.findall(r"[-+0-9.Ee]+", a[1] if kind == "CARTESIAN_POINT" else a[1])
        return np.array([float(x) for x in nums])

    def placement(self, eid: int) -> np.ndarray:
        """AXIS2_PLACEMENT_3D -> 4x4."""
        a = self.args(eid)
        p = self._vec(self.ref(a[1]))
        z = self._vec(self.ref(a[2])) if a[2] != "$" else np.array([0, 0, 1.0])
        x = self._vec(self.ref(a[3])) if a[3] != "$" else np.array([1.0, 0, 0])
        z = z / np.linalg.norm(z)
        x = x - z * np.dot(x, z)
        x = x / np.linalg.norm(x)
        y = np.cross(z, x)
        T = np.eye(4)
        T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, p
        return T

    def product_name(self, pd: int) -> str:
        pdf = self.ref(self.args(pd)[2])
        prod = self.ref(self.args(pdf)[2])
        return self.args(prod)[0].strip("'")

    # -- geometry point clouds -------------------------------------------
    def product_points(self, product: str) -> np.ndarray:
        """All CARTESIAN_POINTs reachable from a product's shape representation(s),
        in that product's local frame. Used for bounding boxes / centroid estimates of
        purchased parts that have no STL (battery, motors, electronics)."""
        pds = [i for i, b in self.of_type("PRODUCT_DEFINITION")
               if self.product_name(i) == product]
        if not pds:
            raise KeyError(product)
        pd = pds[0]
        pdshape = [i for i, b in self.of_type("PRODUCT_DEFINITION_SHAPE")
                   if self.ref(_split_args(b)[2]) == pd]
        reps = [self.ref(_split_args(b)[1]) for i, b in self.of_type("SHAPE_DEFINITION_REPRESENTATION")
                if self.ref(_split_args(b)[0]) in pdshape]
        # follow simple SHAPE_REPRESENTATION_RELATIONSHIPs (shape rep <-> brep rep)
        srr = self.of_type("SHAPE_REPRESENTATION_RELATIONSHIP")
        frontier = set(reps)
        for _ in range(3):
            for _i, b in srr:
                a = _split_args(b)
                r1, r2 = self.ref(a[2]), self.ref(a[3])
                if r1 in frontier or r2 in frontier:
                    frontier |= {r1, r2}
        # BFS through references collecting points (skip the context entity)
        pts, seen, stack = [], set(), []
        for r in frontier:
            kind, body = self.ents[r]
            items = _split_args(body)[1]
            stack.extend(int(x) for x in _REF.findall(items))
        while stack:
            e = stack.pop()
            if e in seen or e not in self.ents:
                continue
            seen.add(e)
            kind, body = self.ents[e]
            if kind == "CARTESIAN_POINT":
                pts.append(self._vec(e))
                continue
            if kind in ("SHAPE_REPRESENTATION", "ADVANCED_BREP_SHAPE_REPRESENTATION",
                        "MANIFOLD_SURFACE_SHAPE_REPRESENTATION"):
                continue
            stack.extend(int(x) for x in _REF.findall(body))
        return np.array([p for p in pts if p.shape == (3,)])

    # -- assembly --------------------------------------------------------
    def instances(self) -> list[Instance]:
        nauo = {}  # nauo id -> (parent_pd, child_pd)
        for i, b in self.of_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE"):
            a = _split_args(b)
            nauo[i] = (self.ref(a[3]), self.ref(a[4]))
        pds_def = {i: self.ref(_split_args(b)[2])
                   for i, b in self.of_type("PRODUCT_DEFINITION_SHAPE")}
        # complex relationship entities -> transformation id
        rel_tf = {}
        for i, (k, b) in self.ents.items():
            if k == "__COMPLEX__" and "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION" in b:
                m = re.search(r"REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\s*\(\s*#(\d+)", b)
                rel_tf[i] = int(m.group(1))
        nauo_T = {}
        for i, b in self.of_type("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION"):
            a = _split_args(b)
            rel, pds = self.ref(a[0]), self.ref(a[1])
            nid = pds_def.get(pds)
            if nid in nauo and rel in rel_tf:
                idt = self.args(rel_tf[rel])
                A1 = self.placement(self.ref(idt[2]))
                A2 = self.placement(self.ref(idt[3]))
                nauo_T[nid] = A2 @ np.linalg.inv(A1)

        children: dict[int, list[int]] = {}
        child_pds = set()
        for nid, (p, c) in nauo.items():
            children.setdefault(p, []).append(nid)
            child_pds.add(c)
        roots = [p for p in children if p not in child_pds]

        out: list[Instance] = []

        def walk(pd, T, path):
            kids = children.get(pd, [])
            for nid in kids:
                c = nauo[nid][1]
                Tc = T @ nauo_T.get(nid, np.eye(4))
                name = self.product_name(c)
                is_leaf = c not in children
                out.append(Instance(path + (name,), name, Tc, is_leaf))
                walk(c, Tc, path + (name,))

        for r in roots:
            walk(r, np.eye(4), (self.product_name(r),))
        return out


if __name__ == "__main__":
    import sys
    sf = StepFile(sys.argv[1])
    inst = sf.instances()
    leaves = [i for i in inst if i.is_leaf]
    print(len(inst), "instances,", len(leaves), "leaves")
    for i in inst:
        if len(i.path) <= 3:
            print("  " * (len(i.path) - 2), i.product, np.round(i.T[:3, 3], 4))
