# MuJoCo models

| File | What it is |
|---|---|
| `rz42_pendulum.xml` | **Nominal model**: the CAD assembly with the CAD mass model below. Its servo is MuJoCo's `<position>` actuator. |
| `rz42_real_plant.xml` | **Hidden "real" plant** for the identification test: different masses, COMs, inertias and friction. The stepper driver's position loop is computed by the runner and applied as a torque. |
| `rz42_real_plant_mass_d1.xml`, `_mass_d2.xml` | The same plant with a 10 g mass taped to the pendulum at d1 and d2 (the added-mass experiment). |
| `make_real_plant.py` | Writes the three real-plant files and `../dataset/ground_truth.json`. |
| `run_experiments.py` | Runs every experiment of `../identification/EXPERIMENT_PROTOCOL.md` on the real plant and writes **encoder-only** logs to `../dataset/experiments/`. Motor torque is used only internally, for a feasibility check, and is never saved. |
| `run_sine.py` | Interactive or headless sine test of the nominal model. |
| `meshes/` | STL meshes of every part, exported from the STEP files. |
| `results/` | Sine-response report, frequency sweep and videos of the nominal model. |
| `_source/` | Generators of the nominal model: OpenCASCADE mass properties + meshes, MJCF writer, C sine simulator. |

Why the real plant computes the servo loop itself: MuJoCo's `<position kv=...>` servo, integrated with `implicit`/`implicitfast`, folds h·kv (here 7.7e-4 kg m², about 5 × the real arm inertia) into the arm's effective inertia. The arm would then not follow the true rigid-body dynamics. The identification does not use torque at all, but the hidden plant should still be physically right.

## Nominal model

`rz42_pendulum.xml` is the assembled v5 model (same parts and positions as the STEP files). Open it in MuJoCo's viewer, or run `run_sine.py` to drive the arm with a sine.

```
pip install -r ../requirements.txt
python run_sine.py                          # viewer, 0.3 rad at 1.0 Hz
python run_sine.py --amp 0.3 --freq 1.9     # near resonance
python run_sine.py --amp 0 --theta0 30      # free swing (damping check)
python run_sine.py --freq 1.9 --headless --time 20   # CSV + PNG, no window
python -m mujoco.viewer --mjcf rz42_pendulum.xml     # just look at the model
```

## Mass model

Printed parts: PLA 1.24 g/cm3, **15% infill** plus solid perimeters/top/bottom of
0.84 mm (mass = rho x [infill x volume + (1 - infill) x shell x surface area], with the shell
mass placed on the real surface for the inertia). Volumes and areas come from the exact CAD solids.

| body | part | mass (g) |
|---|---|---|
| arm | RZ42-003_Arm_608 (printed, 15% infill) | 16.18 |
| arm | RZ42-005_BearingHousing_608 (printed, 15% infill) | 4.50 |
| arm | 608ZZ outer ring/shields/balls | 9.00 |
| arm | AS5600 module | 1.50 |
| arm | heat insert (hub) | 0.45 |
| arm | heat insert (tab) x2 | 0.90 |
| arm | M3x12 set screw | 0.55 |
| arm | M3x8 countersunk x2 | 1.00 |
| **arm total** | COM 33.9, 0.0, 15.6 mm (body frame) | **34.08** |
| pendulum | RZ42-004_Pendulum_608 (printed, 15% infill) | 6.60 |
| pendulum | coin UK 2p (7.12 g) | 7.12 |
| pendulum | magnet D6x2.5 NdFeB | 0.53 |
| pendulum | 608ZZ inner ring | 3.00 |
| **pendulum total** | COM -1.8, -0.0, -42.8 mm (body frame) | **17.25** |

Pendulum about its pivot: I = 4.716e-05 kg m2, COM 42.8 mm below the pivot,
small-angle natural frequency 12.39 rad/s = **1.97 Hz**.
(For comparison, Ferrolho's original URDF pendulum is 14 g with its COM at 51 mm; his measured rigs: 4.46-4.56e-5 kg m2, 12.06 rad/s.)

## Joints and actuator

- `pendulum_joint`: 608 bearing friction from Henrique Ferrolho's rig 2 measurements
  ([rotary-inverted-pendulum](https://github.com/ferrolho/rotary-inverted-pendulum), `policy/sysid_params_rig2.json`):
  viscous 1.74e-5 N m s, Coulomb 3.46e-5 N m. A 30 deg free swing halves in ~3 s and stops after ~6.7 s.
  After building, identify yours with `../identification/02_parameter_identification.ipynb`.
- `arm_joint`: closed-loop stepper as a stiff position servo (kp 1000 N m/rad, critically damped),
  torque limited to 0.45 N m, rotor inertia 6.8e-6 kg m2 (typical 42x48), range +-135 deg.
- No collisions are simulated (the CAD sweep already showed no interference).

## Results in `results/`

- `sine_response_report.png`: 1.9 Hz time series, frequency sweep 0.5-3.5 Hz (0.1 and 0.3 rad), free swing.
- `demo_A0.3_f1.0.mp4`, `demo_A0.3_f1.9_resonance.mp4`: rendered runs.
- `sweep_summary.txt`: one line per sweep run.

Regenerate: `_source/zdt_mj.cpp` (OpenCASCADE mass properties + meshes) -> `_source/make_mjcf.py` -> `_source/sine_sim.c`.
