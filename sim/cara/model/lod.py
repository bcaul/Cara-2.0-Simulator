"""Low-detail copy of the CARA model, used only for drawing the live 3D view.

The full visual meshes (~1M triangles per frame) are too slow to render at 30 fps on
integrated graphics. This builds cara_lo.xml + assets_lo/ with every visual mesh
decimated to <= max_faces. Bodies, joints and geoms are identical to cara.xml, so the
physics MjData can be drawn with this model. Generated once, then cached.
"""
from __future__ import annotations

import numpy as np

from .. import config as C

LO_DIR = C.MODEL_DIR / "assets_lo"
LO_XML = C.MODEL_DIR / "cara_lo.xml"


def _decimate(src, dst, max_faces):
    import trimesh
    mesh = trimesh.load(src, force="mesh")
    if len(mesh.faces) > max_faces:
        import fast_simplification
        vs, fs = fast_simplification.simplify(np.asarray(mesh.vertices), np.asarray(mesh.faces),
                                              target_reduction=1 - max_faces / len(mesh.faces))
        mesh = trimesh.Trimesh(vs, fs, process=False)
        from .build_model import _cluster_decimate
        cell = 1.0                                 # mm; QEM stalls on some parts (rope grooves, slots)
        while len(mesh.faces) > 1.5 * max_faces and cell < 12:
            mesh = _cluster_decimate(mesh, cell)
            cell *= 1.4
    mesh.export(dst)


def load_display_model(max_faces=800):
    import mujoco
    from . import load_model
    load_model()                                   # make sure cara.xml + assets exist
    stale = (not LO_XML.exists() or LO_XML.stat().st_mtime < __import__("os").path.getmtime(__file__) or LO_XML.stat().st_mtime < C.MJCF_FILE.stat().st_mtime
             or not LO_DIR.exists())
    if stale:
        print("Building low-detail display meshes (first run only)...")
        LO_DIR.mkdir(exist_ok=True)
        for src in sorted(C.ASSET_DIR.glob("*.stl")):
            _decimate(src, LO_DIR / src.name, max_faces)
        xml = C.MJCF_FILE.read_text(encoding="utf-8").replace('meshdir="assets"', 'meshdir="assets_lo"')
        LO_XML.write_text(xml, encoding="utf-8")
    return mujoco.MjModel.from_xml_path(str(LO_XML))
