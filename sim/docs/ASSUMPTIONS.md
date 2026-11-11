# CARA 2.0 simulator: assumptions and data sources

Everything here is set in `cara/config.py`. Change a value there and run
`python -m cara.model.build_model` to rebuild the model.

## Sources used
| Source | What was taken from it |
|---|---|
| `CARA_2.0_Code/*.ino` | Link lengths `c d e h m phi`, `w v`, `gr`, `incDir`, `homeDir`, `homeAbsPos`, nominal heights, RC mapping, leg order (0 FL, 1 FR, 2 BL, 3 BR) |
| `Coaxial_5_Bar_Linkage_Kinematics_Derivations.pdf` | Closed-chain topology: coaxial A/B pivot; lower link B is rigid with the foot extension `m` at angle `phi` |
| `CARA 2.0 STEP Assembly.step` | Placement of all 138 printed-part instances, all 12 motors, the battery and the electronics; leg-to-corner mapping; the reference pose |
| `STL Files/` | Visual meshes, plus volume and surface area for the mass model |
| `Build Guide.pdf` | ODrive gains and limits (step 3.3), motor rewind (KV halved), print settings (2 walls, 20% gyroid), roughly 3 kg of filament |
| `CARA 2.0 BOM.xlsx` | Purchased-part counts (fasteners, bearings, squash balls, battery) |

## Geometry (verified, not assumed)
- The STEP pose is exactly the firmware standing pose `getIK(0, e, 280)`, so this is the model's q = 0 configuration. The CAD knee and ankle positions match the analytic 5-bar to within 0.00 mm.
- The hips are at ±w/2 = ±94 mm (lateral) and ±v/2 = ±179.5 mm (fore/aft) around the handle axis. The A/B pivot is h = 43.2 mm below the ABAD axis.
- The 5-bar is modelled as a true loop (`<equality connect>` at the ankle). The test suite checks that the simulated foot matches `getIK` targets to better than 0.01 mm on all four legs.
- The foot is a 40 mm squash ball centred on the firmware foot point. The ABAD axis therefore stands 280 + 20 = 300 mm above the floor.

## Masses (estimates)
| Item | Value | Basis |
|---|---|---|
| Printed parts | 2.75 kg total | A shell+infill model (0.8 mm shell, 20% infill, PLA 1.24 g/cm³) predicts 3.68 kg. That is scaled down to match the guide's "about 3 kg of filament", minus about 0.16 kg for tools and about 5% for supports and purge. Each part keeps its own shape-based centre of mass and inertia. |
| TYI 5008 motor (rewound) | 130 g × 12 | Typical catalogue mass. Modelled as a cylinder (Ø57 × 25 mm) at the stator position from the STEP. |
| MKS XDrive Mini | 25 g × 12 | Mounted on the back of each motor |
| Battery + Power Wheel adapter | 0.86 + 0.06 kg | Kobalt 24 V 3 Ah. Located from the STEP geometry at (5, 11, 95) mm in the body frame. |
| Electronics | 0.20 kg | Teensy, protoboard, IMU, CAN, regulator, terminals; front e-box |
| Wiring | 0.15 kg | Spread over the body |
| Fasteners | 1.08 kg | From BOM counts × typical steel masses (for example, 49 × M3×80 at 5.2 g). Spread over bodies in proportion to printed mass. |
| Bearings | 0.33 kg | From BOM counts × catalogue masses; spread the same way |
| Squash balls | 24 g × 4 | |
| **Total** | **7.39 kg** | **If you weigh the real robot, set `MASS_SCALE` in config.py.** |

Per body: base 4.25 kg, each carriage 0.56 kg (it carries the A and B motors), upper A 0.073 kg, upper B 0.061 kg, lower A 0.028 kg, lower B with foot 0.065 kg. The ABAD motors are fixed to the body. The A and B motors ride on the carriage, which rotates with ABAD.

## Actuators
| Parameter | Value | Basis |
|---|---|---|
| Gear ratio | 9.7 | firmware `gr` |
| Motor KV / Kt | 167.5 KV / 0.049 N·m/A | 335 KV rewound from 20 to 40 turns per slot (Build Guide 3.1.6) |
| Current limit | 15 A, so 6.8 N·m at the joint | Build Guide. Capstan efficiency assumed 0.95. |
| ODrive gains | pos 20, vel 0.05, vel_int 0.333, vel_limit 30 turn/s, filter bandwidth 30 Hz | Build Guide |
| Controller torque constant | 0.04 N·m/A | ODrive 0.5.x default; the guide doesn't set it. The real motor makes about 1.23× the torque the controller thinks it commands. |
| Rotor inertia | 2.5e-5 kg·m², so armature 2.35e-3 kg·m² at the joint | Assumption |
| Joint damping / friction | 0.02 N·m·s/rad / 0.03 N·m | Assumption |
| Joint limits | A, B ≥ 8.18° (homing stop); ABAD ≤ 60° (homing stop); opposite limits A, B ≤ 150°, ABAD ≥ −45° | Homing stops from the firmware; the opposite limits are assumptions |

**Phase 1 only:** joint-space PID equivalent to the ODrive cascade: KP = 17.6 N·m/rad, KD = 0.88 N·m·s/rad, KI = 117 N·m/(rad·s). Phase 2 replaces it with the motor-side ODrive model driven by the ported firmware.

## Contact
- Foot friction 1.0, with soft contact (`solref 0.008 0.6`) to approximate the squash ball's compliance. Both are assumptions.
- The robot collides only with the floor; robot parts don't collide with each other.
- Body and carriage collision shapes are boxes; the links are capsules.

## Known limitations
- No rope stretch, backlash or capstan slip.
- No motor electrical dynamics (current is treated as instantaneous) and no DC bus limit (10 A) yet.
- No CAN latency.
