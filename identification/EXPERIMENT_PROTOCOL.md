# Identification protocol for the physical RZ42 rig

This is the procedure that `02_parameter_identification.ipynb` expects. It needs only:
- the stepper's encoder (arm angle q1);
- the AS5600 (pendulum angle q2);
- loop timestamps;
- the arm position commands you send.

**No motor torque is used.** The rig stays fully assembled for every step: nothing is disconnected, and the motor stays powered.

## 0. Before you start

**Firmware loop**, at a fixed rate of at least 200 Hz (500 Hz is used here). Each iteration:
1. read the AS5600;
2. read the stepper encoder, in the same iteration;
3. send the arm command;
4. write one log line: `t,arm_cmd,q1_meas,q2_meas`.

**Units and conventions:**
- `t` in seconds, from a hardware timer (e.g. `micros()`), not from a nominal step count.
- Angles in rad.
- `q2` is unwrapped by the firmware (count full turns) and increases in the same sense as the Kane model's q2. The raw zero offset does not matter, because it is calibrated in step 1.
- The two encoder reads should be within about 1 ms of each other.

**Stepper driver:** closed-loop position mode, with enough microstepping that the arm motion is smooth. Command profiles should be smooth (a cosine fade-in and fade-out, as in `mujoco/run_experiments.py`). An abrupt step can make a stepper lose sync.

**Bench values**, written into `dataset/bench.json`:

| Value | How |
|---|---|
| added mass Δm | about 10 g (e.g. two M6 nuts taped together), on a 0.1 g kitchen scale |
| its diameter and length | calipers |
| d1 = 25 mm, d2 = 45 mm | Mark two spots on the pendulum centre line at these distances from the pivot axis (the 608 centre). Measure them with calipers, ±0.5 mm. **Do not use about 66 mm**: there the added mass does not change the swing frequency (g/ω²). |
| m_p (optional) | Only if you weighed the pendulum with coin, magnet and 608 inner ring **before** assembly. Gives the centre of mass l_p; not needed for control. |

## 1. Calibration: `calib_rest`
1. Arm held at 0 (driver enabled), pendulum hanging.
2. Tap the pendulum lightly and let it settle completely.
3. Log 5 s.

The mean of `q2_meas` is the AS5600 zero offset. Repeat 3 times and check the means agree within about 0.1°: the 608ZZ's Coulomb friction can stop it slightly off-centre. If they disagree, use their average.

## 2. Free decays, arm held: `free_decay_010deg` … `free_decay_150deg`
1. Arm held at 0.
2. Lift the pendulum by hand to about 10°, 30°, 60°, 90° and 150°.
3. Hold still, release cleanly (no push), and log 15 s, until it stops.

Start the log at release, or trim the hand-held part before running the notebook.

These give ω² (frequency), β (viscous) and φ (Coulomb). Using several amplitudes is what separates viscous from Coulomb friction, and the large ones cover the nonlinear swing-up range.

## 3. Arm-driven excitation, pendulum starting at rest

| Name | Arm command | Purpose |
|---|---|---|
| `sine_A0.20_f0.80` | 0.20 rad, 0.8 Hz, 10 s, 1 s fade-in/out | coupling κ, off-resonance |
| `sine_A0.20_f1.50` | 0.20 rad, 1.5 Hz | κ |
| `sine_A0.15_f1.95` | 0.15 rad, 1.95 Hz (near resonance) | κ, large q2 |
| `sine_A0.20_f2.50`, `sine_A0.20_f3.00` | above resonance | κ, phase |
| `chirp_0.3-4Hz` | 0.15 rad, 0.3 → 4 Hz linear chirp, 30 s | κ over the whole band |
| `swing_A0.80_f1.20`, `swing_A1.00_f0.80` | 0.8–1.0 rad: large, fast arm motion | centrifugal ρ |
| `pump_A0.30_f1.95-1.30` | 0.30 rad, 1.95 → 1.30 Hz falling chirp, 15 s | pendulum up to about 120°: the swing-up range |

The exact command functions are in `mujoco/run_experiments.py` (`experiments()`). The same code can generate the command tables for the firmware.

**Safety:**
- Keep the arm well within ±135° so the cable has slack.
- Stop if the stepper stalls or loses steps: the encoder and the command will diverge.
- In simulation the peak motor torque of every run is under 25 mN·m. A 42×48 stepper has far more than that, but loose couplings will show up here first.

## 4. Absolute scale: added-mass decays
1. Tape Δm on the centre line at **d1**.
2. Free decays from about 20° and 60°, arm held: `massD1_decay_020deg`, `massD1_decay_060deg`.
3. Move the mass to **d2**, then one free decay from about 40°: `massD2_decay_040deg_val`. This is a validation run.
4. Remove the mass.

## 5. Independent validation runs
These are not used for fitting:
- `multisine_val`: sum of 6 sines, 0.6–3.6 Hz, 20 s;
- `swing_A0.60_f1.30_val`: 0.6 rad at 1.3 Hz;
- `free_decay_045deg_val`: release from about 45°;
- `massD2_decay_040deg_val`: from step 4.

## 6. Run the identification
1. Put the logs in `dataset/experiments/`, named as above.
2. Update `dataset/experiments.json`. The notebook reads each run's `purpose` from it: `calibration`, `train`, `scale` or `validation`.
3. Update `dataset/bench.json`.
4. Run `identification/02_parameter_identification.ipynb`.

Do not use `03_evaluate_against_truth.ipynb` on a real rig: it only scores the simulation study.

**What to check:**
- The repeatability table (section 4): the two groups should agree within a few %. If one ratio is far off, its excitation was weak: repeat those runs with larger amplitude.
- The q2 RMS error after the fit should approach the sensor resolution (about 0.1°). If it stays at several degrees, look for timestamp problems, encoder sign errors, or coupling slip in the D-shaft.
- Validation fits should be above about 95 %. The `massD2` frequency prediction should be within about 1 %; if not, re-measure d1 and Δm.

## What you get, and what you do not
**Identified:**
- κ, ρ, ω², β, φ: the complete pendulum model with the arm acceleration as input, which is what balancing and swing-up need;
- J2, G, K, J1, b_p, F_p in SI units, from the added mass;
- l_p, if m_p was weighed.

**Not identifiable without torque:**
- the arm inertia J0;
- arm friction;
- the arm's mass and centre of mass;
- m_p and l_p separately (without weighing);
- I_py and I_pz separately;
- c_p and I_pxz separately.

None of these is needed by a controller that commands the stepper's position, speed or acceleration. Use the CAD J0 (about 1.3e-4 kg·m²) with a safety factor when you check the motor's torque margin.
