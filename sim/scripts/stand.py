"""Phase 1: drop CARA onto the floor and hold the standing pose (zNom = 280 mm).

    python scripts/stand.py              # interactive viewer
                                         # arrow keys shove the body (trackpad-friendly)
    python scripts/stand.py --headless   # print stats only
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mujoco
import numpy as np

from cara import config as C
from cara.model import load_model
from cara.sim_backend.model_utils import chain_error_mm, foot_in_leg_frame
from cara.sim_backend.simple_pd import KD, KI, KP, TAU_MAX, JointPID


# glfw key codes -> shove direction in the world frame (x forward, y left)
SHOVE_KEYS = {265: (1, 0), 264: (-1, 0), 263: (0, 1), 262: (0, -1)}   # Up Down Left Right
SHOVE_FORCE = 40.0      # N on the body
SHOVE_TIME = 0.15       # s  (impulse 6 N*s, about 0.8 m/s on a 7.4 kg robot)


def run(seconds=10.0, viewer=False, drop=0.01, verbose=True):
    m = load_model()
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("stand").id)
    d.qpos[2] += drop
    pid = JointPID(m)
    log = []
    v = None
    pending = []            # shoves requested from the viewer thread
    shove_until, shove_dir = -1.0, (0, 0)
    base = m.body("base").id
    if viewer:
        from mujoco import viewer as mjviewer

        def on_key(key):
            if key in SHOVE_KEYS:
                pending.append(SHOVE_KEYS[key])

        v = mjviewer.launch_passive(m, d, key_callback=on_key)
        print("Arrow keys shove the robot (Up = from behind, Left/Right = sideways). "
              "Close the window to exit.")
    t_wall = time.perf_counter()
    while d.time < seconds or (v is not None and v.is_running()):
        if pending:
            shove_dir, shove_until = pending.pop(0), d.time + SHOVE_TIME
        if d.time < shove_until:
            d.xfrc_applied[base, :2] = SHOVE_FORCE * np.array(shove_dir)
        elif shove_until > 0:
            d.xfrc_applied[base, :2] = 0.0
            shove_until = -1.0
        pid(d, m.opt.timestep)
        mujoco.mj_step(m, d)
        if int(round(d.time / m.opt.timestep)) % 100 == 0:
            log.append((d.time, d.qpos[2], chain_error_mm(m, d)))
        if v is not None:
            if not v.is_running():
                break
            if int(round(d.time / m.opt.timestep)) % 33 == 0:
                v.sync()
                lag = d.time - (time.perf_counter() - t_wall)
                if lag > 0:
                    time.sleep(lag)
    log = np.array(log)
    late = log[log[:, 0] > 2.0]
    stats = dict(
        total_mass=float(sum(m.body_mass)),
        height_mean_mm=float(late[:, 1].mean() * 1000),
        height_min_mm=float(late[:, 1].min() * 1000),
        height_max_mm=float(late[:, 1].max() * 1000),
        chain_err_max_mm=float(log[:, 2].max()),
        foot_z_mm=[float(foot_in_leg_frame(m, d, i)[2]) for i in range(4)],
        upright=float(d.body("base").xmat[8]),
    )
    if verbose:
        print(f"joint-space gains from ODrive config: KP={KP:.1f} Nm/rad  KD={KD:.2f} Nm s/rad  "
              f"KI={KI:.0f} Nm/(rad s)  tau_max={TAU_MAX:.2f} Nm")
        for k, val in stats.items():
            print(f"  {k}: {val}")
    return stats


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--seconds", type=float, default=10.0)
    a = ap.parse_args()
    run(a.seconds, viewer=not a.headless)
