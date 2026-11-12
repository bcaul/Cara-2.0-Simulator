"""Phase 2: the ported firmware drives the simulated ODrives (no transmitter needed).

    python scripts/demo.py               # home in the air, set down, stand, kinematicsDemo()
    python scripts/demo.py --suspended   # stay held in the air the whole time (watch homing)
    python scripts/demo.py --headless    # no window; prints a timeline and the stand height

Sequence (same calls the firmware makes): setup() -> home() -> [set down] ->
moveFeet(...nominal...) -> kinematicsDemo() (translations, tilts, 3 jumps).
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cara.firmware.ramp import LINEAR  # noqa: E402
from cara.sim_backend.sim_hal import Simulation  # noqa: E402


def make_program(sim, suspended, events):
    def say(fw, text):
        events.append((fw.millis() / 1000, text))
        print(f"[{fw.millis() / 1000:6.2f} s] {text}")

    def program(fw):
        fw.setup()
        say(fw, "home(): driving every joint to its end stop (Iq > homingIq)")
        fw.home()
        fw.state = 1
        say(fw, "homed, legs at the start pose (z = 160 mm)")
        if not suspended:
            sim.hand.lower_to_floor()
            while sim.hand.active:
                fw.delay(10)
            say(fw, "set down on the floor")
            fw.delay(500)
        n = (fw.xNom, fw.yNom, fw.zNom)
        fw.moveFeet(*(n * 4), LINEAR, 500)
        fw.state = 3
        say(fw, "standing (zNom = 280 mm)")
        fw.delay(1500)
        say(fw, "kinematicsDemo()")
        fw.kinematicsDemo()
        say(fw, "demo finished - holding the stand")
        while True:
            fw.delay(100)
    return program


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--suspended", action="store_true")
    ap.add_argument("--headless", action="store_true")
    a = ap.parse_args()
    sim = Simulation(held_height=0.5)
    events = []
    sim.start(make_program(sim, a.suspended, events))

    if a.headless:
        w0 = time.perf_counter()
        while sim.time < 50 and sim.hal.running:
            sim.step()
        print(f"simulated {sim.time:.1f} s in {time.perf_counter() - w0:.1f} s; "
              f"height {sim.base_height_mm():.0f} mm; overspeed trips {sim.bank.overspeed_trips}")
        sim.stop()
        return

    from mujoco import viewer as mjviewer
    with mjviewer.launch_passive(sim.m, sim.d) as v:
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 1.6, -20, 135
        wall0, sim0 = time.perf_counter(), sim.time
        while v.is_running() and sim.hal.running:
            target = sim0 + (time.perf_counter() - wall0)
            budget = time.perf_counter() + 1 / 30
            while sim.time < target and time.perf_counter() < budget:
                sim.step()
            v.sync()
            time.sleep(0.001)
    sim.stop()


if __name__ == "__main__":
    main()
