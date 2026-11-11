"""Phase 1: open the CARA model in the MuJoCo viewer (no control - it will collapse).

    python scripts/view_model.py            # passive physics, robot falls limp
    python scripts/view_model.py --frozen   # static pose, no physics (inspect geometry)

Viewer tips: double-click a body to select it, Ctrl+right-drag to push it,
press 3 / 1 to toggle collision / visual geom groups, Tab for the side panel.
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mujoco
from mujoco import viewer as mjviewer

from cara import config as C
from cara.model import load_model


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frozen", action="store_true")
    a = ap.parse_args()
    m = load_model()
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("stand").id)
    print(f"bodies={m.nbody} joints={m.njnt} actuators={m.nu} mass={sum(m.body_mass):.3f} kg")
    with mjviewer.launch_passive(m, d) as v:
        t0 = time.perf_counter()
        while v.is_running():
            if a.frozen:
                mujoco.mj_forward(m, d)
                time.sleep(0.02)
            else:
                while d.time < time.perf_counter() - t0:
                    mujoco.mj_step(m, d)
            v.sync()


if __name__ == "__main__":
    main()
