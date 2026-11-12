# CARA 2.0 MuJoCo simulator

A physics simulation of the CARA 2.0 quadruped, built from the real CAD and firmware.
The geometry comes from the STEP assembly and STLs. The 5-bar legs are true closed chains.
The motors are modelled on the ODrive settings from the build guide, and the robot is
driven by a line-for-line Python port of the Teensy firmware, controlled from the
keyboard or a gamepad through the same 8 RC channels as the real transmitter.

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

## Phase 2: the firmware drives the simulated ODrives
| Command | What you should see |
|---|---|
| `python scripts/demo.py --suspended` | Held in the air by the handle: `home()` drives every joint to its end stop (about 3 s), then the legs fold to the start pose and it stays in the air. |
| `python scripts/demo.py` | Same homing, then it's set down, stands up and runs your `kinematicsDemo()`: squats, shifts, tilts and 3 jumps (about 40 s). |
| `python scripts/demo.py --headless` | The same run with no window; prints a timeline. |
| `python -m pytest -q tests` | Adds: IK/FK round trip, Ramp port, homing accuracy, stand/squat/tilt through the firmware. |

## Phase 3: drive it like the real robot
```
python scripts/run.py               # keyboard; a plugged-in gamepad also works
python scripts/run.py --stabilize   # enables the IMU getShifts() block that is commented out in loop()
```
One window opens: the 3D view on the left (drag to rotate, wheel to zoom), the transmitter on the right. The robot starts **held in the air**; a banner tells you what to press next (**hold Space 1 s** to home).
The live view uses low-detail meshes and no shadows so it keeps up on integrated graphics; `scripts/view_model.py` still shows the full-detail model.

| Key | RC channel | Does (same as the real firmware) |
|---|---|---|
| **Space** (hold 1 s) | CH6 button | 1st: `home()` (robot is held in the air, then set down crouched). 2nd: stand. 3rd: crouch. |
| **W/A/S/D** | right stick (ch1/ch2) | walk forward / left / back / right; two keys = diagonal |
| **Left / Right** | left stick X (ch4) | spin CCW / CW |
| **Up / Down** | left stick Y (ch3) | roll demo / pitch demo |
| **1 / 2 / 3** | CH5 switch fwd/mid/back | 1 = step in place, 3 = low stance (or motors IDLE when crouched); 1 again re-arms |
| **Z / X / C** | CH7 switch | Z + Space = `kinematicsDemo()`, C + Space = jump |
| **+ / -** | CH8 knob | (unused by the firmware) |
| **Backspace** | - | transmitter on/off (off prints DISCONNECTED) |
| **H** | - | pick the robot up / set it down |
| **F11** | - | fullscreen (or resize/maximise the window); Esc leaves fullscreen |

Gamepad (Xbox layout): sticks are the sticks, A = button, RB/LB = CH5, D-pad = CH7, Start = on/off, Y = pick up.

The firmware is blocking just like on the Teensy: while it walks, it only checks the
sticks between steps, and a demo or jump runs to the end before it listens again.

Tests: `test_rc_map.py` checks every stick direction against the firmware's `criteria()`.
`test_walk.py` runs a scripted session through `getReceiver()`: home, stand, walk forward
(about 3 m in 5 s), spin, low stance, crouch.

## Layout
```
cara/config.py            firmware constants (same names as the .ino) + physical parameters
cara/firmware/            control code ported from the Teensy (pure Python, no MuJoCo)
  cara_firmware.py        CaraFirmware: setup/loop and every function in Functions.ino
  kinematics.py           getIK / getFK  (Functions.ino)
  ramp.py, lowpass.py     ports of the RAMP library and the LowPass filter
  hal.py                  hardware interface the firmware talks to (sim now, real robot later)
cara/model/               STEP parser, mass model, MJCF generator
cara/sim_backend/         MuJoCo side
  sim_hal.py              Simulation + SimHAL: firmware thread in lockstep with physics, "hand"
  odrive_sim.py           12 simulated ODrives (filter, PI loops, limits, Iq, overspeed)
  imu_sim.py              BNO086 rotation vector with noise
cara/teleop/              virtual RC transmitter + keyboard/gamepad window
scripts/                  runnable demos
tests/                    pytest suite
docs/ASSUMPTIONS.md       every estimate and where it came from
```

## Frames
- **Sim/world frame:** x forward, y left, z up. The body origin is on the handle axis at ABAD-axis height.
- **Firmware leg frame** (per leg, mm): x fore/aft, y outward, z down from the ABAD axis. Front legs: +x is forward. Back legs: +x is backward. These match `moveRobot` and `stepPhase`.
- **Joint angles:** `q = 0` is the CAD/standing pose. The firmware angle is `theta = THETA_REF + q / 2π` revs (`cara/sim_backend/model_utils.py`).

## How the simulation runs the firmware
The firmware runs on its own thread with a **virtual clock**. `delay()`, `millis()`, the
Ramp grain and the LowPass timing all use simulated time. When the firmware needs time
to pass, it waits while the physics catches up (2 kHz). So the blocking
`while (ramp.isRunning()) {...; delay(5);}` loops behave as they do on the Teensy, and
the results are the same on any PC. Swapping `SimHAL` for a real serial/CAN HAL (Phase 4
stub) runs the same firmware code on the robot.
