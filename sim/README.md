# CARA 2.0 MuJoCo simulator

A physics simulation of the CARA 2.0 quadruped, built from the real CAD and firmware.
The geometry comes from the STEP assembly and STLs. The 5-bar legs are true closed chains.
The motors are modelled on the ODrive settings from the build guide. The plan is for the
simulator to run a Python port of the Teensy firmware.

See `docs/ASSUMPTIONS.md` for every number that is an estimate.

## Setup (Windows, Python 3.12)
```
cd sim
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python -m cara.model.build_model     # generates cara/model/cara.xml + meshes (~1 min, first time only)
```
The scripts also build the model automatically if it's missing.

## Phase 1: model loads and stands
| Command | What you should see |
|---|---|
| `python scripts/view_model.py --frozen` | The robot in its standing (CAD) pose. Press `3` to show collision shapes and `1` to hide meshes. |
| `python scripts/view_model.py` | No control: the robot collapses under gravity, so you can see the loops and limits in action. |
| `python scripts/stand.py` | The robot is dropped 1 cm onto the floor and holds `zNom = 280 mm` using ODrive-equivalent gains. Ctrl+right-drag pushes it. |
| `python scripts/stand.py --headless` | Prints the mass and the stand height (expect about 300 mm: 280 mm plus the ball radius) and the loop-closure error. |
| `python -m pytest -q tests` | Model compiles; the 5-bar loop and foot positions match the firmware `getIK` on all legs; it stands for 10 s. |

## Layout
```
cara/config.py            firmware constants (same names as the .ino) + physical parameters
cara/firmware/            control code ported from the Teensy (pure Python, no MuJoCo)
  kinematics.py           getIK / getFK  (Functions.ino)
cara/model/               STEP parser, mass model, MJCF generator
cara/sim_backend/         MuJoCo-side helpers (pose utilities, Phase-1 PID)
scripts/                  runnable demos
tests/                    pytest suite
docs/ASSUMPTIONS.md       every estimate and where it came from
```

## Frames
- **Sim/world frame:** x forward, y left, z up. The body origin is on the handle axis at ABAD-axis height.
- **Firmware leg frame** (per leg, mm): x fore/aft, y outward, z down from the ABAD axis. Front legs: +x is forward. Back legs: +x is backward. These match `moveRobot` and `stepPhase`.
- **Joint angles:** `q = 0` is the CAD/standing pose. The firmware angle is `theta = THETA_REF + q / 2π` revs (`cara/sim_backend/model_utils.py`).
