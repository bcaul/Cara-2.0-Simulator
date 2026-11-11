"""CARA MJCF model: generated from the STEP assembly + STLs by build_model.py."""
from .. import config as C


def load_model():
    """Return a compiled MjModel, generating cara.xml + meshes on first use (~1 min)."""
    import mujoco
    if not C.MJCF_FILE.exists() or not C.ASSET_DIR.exists():
        from .build_model import ModelBuilder
        print("Building CARA model from STEP/STL files (first run only)...")
        b = ModelBuilder(verbose=True)
        C.MJCF_FILE.write_text(b.build())
        (C.MODEL_DIR / "mass_report.md").write_text(b.report())
    return mujoco.MjModel.from_xml_path(str(C.MJCF_FILE))
