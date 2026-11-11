"""Single source of truth for CARA 2.0 simulation parameters.

Section 1 copies the firmware constants VERBATIM (names and values) from
CARA_2.0_Code/CARA_2.0_Code.ino so the ported control code reads identically.
Sections 2+ are physical / simulation parameters. Every estimate is marked
ASSUMPTION and documented in sim/docs/ASSUMPTIONS.md.
"""
from pathlib import Path

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SIM_DIR = Path(__file__).resolve().parents[1]
PROJECT_DIR = SIM_DIR.parent
STEP_FILE = PROJECT_DIR / "Optional Reference Material" / "CARA 2.0 STEP Assembly.step"
STL_DIR = PROJECT_DIR / "STL Files"
MODEL_DIR = SIM_DIR / "cara" / "model"
ASSET_DIR = MODEL_DIR / "assets"
MJCF_FILE = MODEL_DIR / "cara.xml"

# ---------------------------------------------------------------------------
# 1. Firmware constants (CARA_2.0_Code.ino) - do not rename
# ---------------------------------------------------------------------------
OFF = 1000
gr = 9.7                                   # gear reduction (capstan)
homingIq = 3
incDir = [-1, 1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1]
homeDir = [1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1]
homeAbsPos = [8.183482, 8.183482, 60.0]    # deg, A, B, ABAD physical stops
c, d, e, h = 100, 150, 47.125, 43.23193    # mm
m, phi = 45, 150                           # mm, deg
w, v = 188, 359                            # mm (IKR: hip track width / wheelbase)
xStart, yStart, zStart = 0, e, 160
chnPin = [7, 6, 5, 4, 3, 2, 1, 0]
mapRange = [[-10, 10], [10, -10], [-10, 10], [-10, 10], [1, -1], [0, 1], [1, -1], [0, 100]]
constrainRange = [[-10, 10], [-10, 10], [-10, 10], [-10, 10], [-1, 1], [0, 1], [-1, 1], [0, 100]]
sensitivity = 0.9
zDefault, zLow = 280, 200

# Leg indexing used by the firmware: leg*3 + {0: A, 1: B, 2: ABAD}
LEG_NAMES = ["FL", "FR", "BL", "BR"]
JOINT_NAMES = ["A", "B", "ABAD"]

# ---------------------------------------------------------------------------
# 2. Geometry from the STEP assembly (sim frame: x forward, y left, z up, metres)
# ---------------------------------------------------------------------------
# STEP (Onshape) frame: forward = -Y, left = +X, up = +Z. Body centre (handle axis):
STEP_BODY_CENTER = (-0.039, 0.023, 0.0)
STEP_TO_SIM_ROT = ((0, -1, 0), (1, 0, 0), (0, 0, 1))
# Leg sub-assembly instance name in the STEP -> firmware leg index
STEP_LEG_INSTANCES = {"Leg <2>": 0, "Leg MIR <1>": 1, "Leg MIR <2>": 2, "Leg <1>": 3}
# Per-leg sign of the firmware leg-frame axes in the sim frame:
#   firmware x (fore/aft)  = LEG_SX * sim x   (front legs +, back legs -)
#   firmware y (outward)   = LEG_SY * sim y   (left legs +, right legs -)
#   firmware z (downward)  = -sim z
LEG_SX = [1, 1, -1, -1]
LEG_SY = [1, -1, 1, -1]
# ABAD axis lies at lateral +-w/2, height 0; the A/B pivot is h below it, +-v/2 fore/aft.

# ---------------------------------------------------------------------------
# 3. Actuators (Build Guide, step 3.3 ODrive config + motor rewind)
# ---------------------------------------------------------------------------
ODRIVE = dict(
    pos_gain=20.0,                  # (turn/s)/turn            [Build Guide]
    vel_gain=0.05,                  # Nm/(turn/s)              [Build Guide]
    vel_integrator_gain=0.333,      # Nm/turn                  [Build Guide]
    vel_limit=30.0,                 # turn/s (motor side)      [Build Guide]
    input_filter_bandwidth=30.0,    # Hz (INPUT_MODE_POS_FILTER)[Build Guide]
    current_lim=15.0,               # A                        [Build Guide]
    controller_torque_constant=0.04,  # Nm/A ODrive 0.5.x default (not set in guide) ASSUMPTION
    dc_max_current=10.0,            # A per controller bus     [Build Guide]
)
MOTOR_KV = 335.0 / 2.0              # rewound 20 -> 40 turns/slot halves KV [Build Guide 3.1.6]
MOTOR_KT = 8.27 / MOTOR_KV          # Nm/A  (~0.049)
MOTOR_ROTOR_INERTIA = 2.5e-5        # kg m^2  ASSUMPTION (TYI 5008 rotor + small drum)
CAPSTAN_EFFICIENCY = 0.95           # ASSUMPTION
JOINT_DAMPING = 0.02                # Nm s/rad  ASSUMPTION (bearings + rope)
JOINT_FRICTIONLOSS = 0.03           # Nm        ASSUMPTION

# Opposite-side joint limits (deg). Homing-side stops come from homeAbsPos.
JOINT_RANGE_DEG = {
    "A": (homeAbsPos[0], 150.0),     # upper ASSUMPTION
    "B": (homeAbsPos[1], 150.0),     # upper ASSUMPTION
    "ABAD": (-45.0, homeAbsPos[2]),  # lower ASSUMPTION
}

# ---------------------------------------------------------------------------
# 4. Mass model (Build Guide printer settings + catalogue weights)
# ---------------------------------------------------------------------------
PLA_DENSITY = 1240.0                # kg/m^3
PRINT_SHELL_THICKNESS = 0.0008      # m  (2 walls x 0.4 mm nozzle; top/bottom assumed similar)
PRINT_INFILL = 0.20                 # 20 % gyroid
# Build Guide: "about 3 kg of filament to print all of the parts" (robot + tools).
# The shell+infill model over-predicts (3.68 kg robot + 0.16 kg tools), so printed-part
# masses are scaled to this target (3.0 kg x robot share, ~5 % for supports/purge).
PRINTED_MASS_TARGET = 2.75          # kg  (set None to use the raw shell+infill model)
MASS_SCALE = 1.0                    # set so total matches a real scale reading

PURCHASED_MASS = dict(              # kg, ASSUMPTIONS (catalogue / typical values)
    motor=0.130,                    # TYI 5008 incl. rewound copper
    xdrive=0.025,                   # MKS XDrive Mini
    battery=0.860,                  # Kobalt 24 V 3 Ah
    battery_adapter=0.060,          # Power wheel adapter
    electronics=0.200,              # Teensy, protoboard, IMU, CAN, regulator, switch, terminals
    wiring=0.150,                   # 16/22 AWG wiring, XT60
    squash_ball=0.024,
    fasteners=1.08,                 # from BOM counts (see ASSUMPTIONS.md)
    bearings=0.33,                  # from BOM counts
)

# ---------------------------------------------------------------------------
# 5. Contact / simulation
# ---------------------------------------------------------------------------
FOOT_RADIUS = 0.020                 # squash ball (40 mm)
FOOT_FRICTION = 1.0                 # rubber on floor ASSUMPTION
FOOT_SOLREF = (0.008, 0.6)          # softer contact ~ squash-ball compliance ASSUMPTION
PHYSICS_DT = 0.0005                 # 2 kHz
