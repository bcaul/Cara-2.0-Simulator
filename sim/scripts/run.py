"""Phase 3: drive CARA in real time with keyboard or gamepad, through the ported firmware.

    python scripts/run.py               # keyboard (+ gamepad if one is plugged in)
    python scripts/run.py --stabilize   # also enable the getShifts() block commented out in loop()
    python scripts/run.py --no-gamepad

One window: 3D view on the left, transmitter on the right. Same procedure as the real robot:

  1. The robot starts held in the air by its handle, motors idle.
  2. Hold Space 1 s  -> home() (legs find their end stops), then it is set down crouched.
  3. Hold Space 1 s  -> stand up.
  4. WASD walk, Left/Right spin, Up/Down tilt demos, 1/2/3 and Z/X/C switches.
  5. Hold Space 1 s  -> crouch again.   H picks it up / sets it down at any time.
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cara.sim_backend.sim_hal import Simulation  # noqa: E402
from cara.teleop.panel import TransmitterPanel  # noqa: E402
from cara.teleop.rc_transmitter import Transmitter  # noqa: E402

FPS = 30


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stabilize", action="store_true", help="enable IMU stabilisation (getShifts)")
    ap.add_argument("--no-gamepad", action="store_true")
    ap.add_argument("--hold-height", type=float, default=0.45, help="base height when held (m)")
    a = ap.parse_args()

    tx = Transmitter()
    sim = Simulation(held_height=a.hold_height, stabilize=a.stabilize, rc_source=tx.widths)
    panel = TransmitterPanel(tx, sim, use_gamepad=not a.no_gamepad)
    sim.start()
    print(__doc__)

    auto_set_down = True
    wall0, sim0 = time.perf_counter(), sim.time
    rtf = None
    try:
        while not panel.quit:
            frame_start = time.perf_counter()
            panel.poll()
            if panel.take_hand_toggle():
                if sim.hand.active:
                    sim.hand.lower_to_floor()
                else:
                    sim.hand.pick_up(a.hold_height)
                auto_set_down = False
            # the first time homing finishes, set the robot down like a person would
            if auto_set_down and sim.hand.active and sim.fw.state != 0:
                sim.hand.lower_to_floor()
                auto_set_down = False
            # advance physics to match the wall clock (lockstep with the firmware thread)
            target = sim0 + (frame_start - wall0)
            budget = frame_start + 0.8 / FPS
            while sim.time < target and time.perf_counter() < budget:
                sim.step()
            if sim.time < target - 0.25:            # PC too slow: drop the backlog
                wall0, sim0 = time.perf_counter(), sim.time
            if sim.hal.error:
                print("firmware crashed:", sim.hal.error)
                break
            el = time.perf_counter() - wall0
            rtf = (sim.time - sim0) / el if el > 2 else None
            panel.draw(sim, rtf)
            spare = 1 / FPS - (time.perf_counter() - frame_start)
            if spare > 0:
                time.sleep(spare)
    finally:
        sim.stop()
        panel.close()


if __name__ == "__main__":
    main()
