# Position-command swing-up and LQR balancing of the RZ42 Furuta pendulum

**Scope.** This report covers a closed-loop controller whose only output is the stepper's commanded arm angle `q1_cmd`. It is designed from the identified pendulum model (`identification/identified_parameters.json`) and validated on the hidden MuJoCo plant (`mujoco/rz42_real_plant.xml`). It is a simulation study, built to transfer to the ZDT-driven rig.

**Hardware assumption.** The ZDT stepper driver is modelled as a position servo. Its bandwidth (15 Hz) and damping (0.8) are **placeholders, not measured values**.

## 1. Architecture: three layers that never share information

```
 layer 3  EXTERNAL CONTROLLER  control/hybrid.py (+ swingup.py, lqr.py, sensors.StateEstimator)
          in : q1_meas (14-bit stepper encoder), q2_meas (12-bit AS5600) at 200 Hz
          out: q1_cmd  (rad)                                   <- the ONLY control input
                     |  1 ms latency, zero-order hold
 layer 2  ZDT DRIVER MODEL     control/servo.py
          in : q1_cmd, its own encoder (q1), q1'
          out: motor torque, PD law tuned for an assumed load inertia, speed limit,
               torque limit falling with speed
                     |
 layer 1  MUJOCO PLANT         control/plant.py -> mujoco/rz42_real_plant.xml (hidden "real" rig)
          full nonlinear coupled mechanics at 10 kHz, true masses/inertias/friction
```

- The external controller never sees the motor torque, the true velocities, or any plant parameter.
- `control/models.py` refuses to load a file named `ground_truth*`.
- The motor torque is written to the log for diagnostics only.
- The arm is never moved by writing `qpos`/`qvel`. It moves only because the driver model applies torque.

## 2. Conventions (checked against the Kane model and MuJoCo)

| Quantity | Definition |
|---|---|
| `q1` | arm angle about the vertical motor axis (MuJoCo `arm_joint`, axis +z). 0 = the arm along −y, as in the original URDF. |
| `q2` | pendulum angle about the arm axis (`pendulum_joint`, axis = arm +x). **0 = hanging, π = upright.** |
| upright error | δ = wrap(q2 − π) ∈ [−π, π). Every multiple of 2π counts as upright, so a pendulum that went over the top several times is still caught. |
| units | rad, rad/s, N·m. The configuration file is in SI units, except `*_deg` and `*_hz` entries. |

The identified model has the same sign of κ as MuJoCo. In the validation, the identified ratios reproduce the hidden plant to better than 1 %.

## 3. Stepper servo model (layer 2)

**Design target:** $\ddot q_1=\omega_s^2(q_{1,cmd}-q_1)-2\zeta_s\omega_s\dot q_1$, with $\omega_s = 2\pi\cdot15$ rad/s and $\zeta_s = 0.8$ (placeholders).

**Implementation** (a real driver does not know the load):
$$\tau = \hat J\,[\omega_s^2(r - q_{1,enc}) - 2\zeta_s\omega_s\dot q_1], \qquad \hat J = 1.28\times10^{-4}\ \text{kg m}^2$$
$\hat J$ is the CAD estimate of the arm inertia J0. The true plant has 1.47e-4 kg·m², and the swinging pendulum adds a reaction torque on top.

| Limit | Value |
|---|---|
| reference speed `r → q1_cmd` (the driver's max-speed setting) | 30 rad/s |
| torque | $\max(0.10,\ 0.45 - 0.003|\dot q_1|)$ N·m (linear torque-speed curve of a 42×48 stepper) |
| driver encoder | 14 bit |

**Validation on the real coupled plant** (0.1 rad step, pendulum hanging; `python -m control.validate`):

| | rise 10–90 % | overshoot | settling 2 % |
|---|---|---|---|
| ideal model | 26.2 ms | 1.5 % | 40 ms |
| actual (MuJoCo) | 28.9 ms | 0.3 % | 53 ms |

The RMS difference between the two step responses is 1.2 % of the step. The real response is slightly slower, because the true inertia is above $\hat J$.

## 4. Controller-side model

From `identified_parameters.json` (encoder-only identification):

| Ratio | Value |
|---|---|
| κ | 0.9450 |
| ρ | 0.9647 |
| ω² | 148.33 s⁻² |
| β | 0.474 s⁻¹ |
| φ | 0.953 s⁻² |

These are combined with the *assumed* servo. Linearised at upright, with x = [q1, q2−π, q1', q2'] and u = q1_cmd:

$$A_{pos}=\begin{bmatrix}0&0&1&0\\0&0&0&1\\-\omega_s^2&0&-2\zeta_s\omega_s&0\\-\kappa\omega_s^2&\omega^2&-2\kappa\zeta_s\omega_s&-\beta\end{bmatrix},\qquad
B_{pos}=\begin{bmatrix}0\\0\\\omega_s^2\\\kappa\omega_s^2\end{bmatrix}$$

The sign of κ flips at upright, because cos π = −1. This is the same structure as the `A_pos` cell in `01_kane_model.ipynb`.
- **Open-loop eigenvalues:** −75.4 ± 56.5j (the servo), −12.42, and **+11.94** (the pendulum falling over).
- **Discretisation:** zero-order hold at the 200 Hz control rate.

## 5. LQR balance

**Control law:** $q_{1,cmd} = q_{1,eq} - K(x - x_{eq})$, from a discrete LQR (`control/lqr.py`, `solve_discrete_are`).

**Tuning in simulation.** Candidates were tried on the hidden plant with two pendulum taps (+2.7 and +7.3 rad/s), with an actual servo of 15 Hz and of 8 Hz. They were compared on:
- pendulum error;
- arm excursion;
- command rate;
- survival with the slower servo.

Lower R or heavier weights on the pendulum states lost the pendulum with the 8 Hz servo.

**Chosen design:**
- weights: Q = diag(100, 200, 1, 2), R = 10;
- gains: K = [−1.195, 1.182, −0.0759, 0.0965];
- closed-loop poles: s ≈ −8.7 ± 4.5j, −10.0, −1257.

**Velocity estimation.** A first-order low-pass filter on finite differences of the two encoder angles.

| Cutoff | Command chatter (RMS per tick) | Quiet RMS pendulum error |
|---|---|---|
| 40 Hz | 1.9° | 0.16° |
| **12 Hz (chosen)** | **0.7°** | **0.12°** |

At 12 Hz the disturbance rejection is unchanged, and the lower chatter is much kinder to a real stepper.

**Arm reference.** On every catch, $q_{1,eq}$ starts at the arm's current angle and walks back to 0 at 0.3 rad/s.

**Quiet balancing** on the hidden plant: pendulum RMS error 0.12°, arm motion 0.18° RMS, peak motor torque 0.022 N·m.

## 6. Swing-up

Pendulum energy per unit $J_2$, exact at any angle:

$$E = \tfrac12\dot q_2^2 + \omega^2(1-\cos q_2), \qquad E_{up} = 2\omega^2 = 296.7$$

Along the identified nonlinear model:

$$\dot E = -\kappa\,\dot q_2\cos q_2\,\ddot q_1 - (\beta\dot q_2 + \phi\tanh(\dot q_2/v_s) - \rho\sin q_2\cos q_2\,\dot q_1^2)\,\dot q_2$$

**Chosen law: position pumping** (`control/swingup.py`):
$$q_{1,cmd} = \text{sat}\big(c\,\max(0, 1-E/E_{up})\;\overline{\dot q_2\cos q_2},\ A_{max}\big)$$
- c = 0.15 s;
- $A_{max}$ = 0.6 rad;
- the overbar is a 5 Hz low-pass filter.

**Why it works.** For a swinging pendulum, $\dot q_2\cos q_2$ oscillates at the swing frequency w, so the arm follows with $\ddot q_1 \approx -w^2 q_1$. That makes $-\kappa\dot q_2\cos q_2\,\ddot q_1 \approx \kappa c w^2(1-E/E_{up})(\dot q_2\cos q_2)^2 \ge 0$: energy is added only while it is below $E_{up}$, and the arm travel is bounded by construction.

**Kick.** The law is a state feedback recomputed every tick. At exact rest it has no direction, so a latched 0.15 rad arm step (0.25 s) starts the swing.

**Design history.** Each earlier step was tested on the hidden plant and rejected for a concrete, documented reason:

| Attempt | What happened |
|---|---|
| Arm acceleration from Åström–Furuta energy control, inverted through the servo model ($q_{1,cmd} = q_1 + (a + 2\zeta\omega_s\dot q_1)/\omega_s^2$) | No energy gain. At 200 Hz the 15 Hz servo reaches each held command within a tick, so the net acceleration is close to zero. |
| The same acceleration integrated into a smooth arm reference | Energy plateaued at about 80 % of $E_{up}$. The acceleration's non-zero mean drifts the arm into its travel limit (126°). |
| Position pumping on the raw $\dot q_2$ | A ~13 Hz limit cycle through the arm–pendulum coupling, and only 20 % of $E_{up}$. Fixed by the 5 Hz low-pass. |
| Pumping above $E_{up}$ as well (negative gain) | Unreliable once the pendulum spins, and E rose to 12 × $E_{up}$. Above $E_{up}$ the pumping is now off and friction removes the excess. |

## 7. Hybrid switching (`control/hybrid.py`)

| Transition | Condition |
|---|---|
| SWING_UP → LQR_BALANCE | \|δ\| < 25°, \|q2'\| < 8 rad/s, \|q1\| < 1.4 rad and \|q1'\| < 10 rad/s |
| LQR_BALANCE → SWING_UP | \|δ\| > 45°, or \|q1\| > 1.5 rad (the arm near its usable travel) |

- **Hysteresis:** the catch window (25°) is well inside the drop window (45°), and there is a minimum dwell of 0.2 s in each mode.
- **Why the catch rate is 8 rad/s:** at exactly $E_{up}$, the pendulum passes 20° from upright at about 4.2 rad/s. With the first choice of 4 rad/s, catching depended on luck (2 of 6 noise seeds never caught).
- **Continuity:** $q_{1,eq}$ starts at the current arm angle, and every command passes a 30 rad/s slew limit and a ±1.7 rad absolute limit. The joint stop is at ±2.36 rad.

## 8. Results on the hidden plant

**Demo** (`python -m control.run_demo`, 40 s):

| Time | Event |
|---|---|
| 0 s | start hanging at rest |
| 0.5 s | kick |
| 3.46 s | caught at δ = 11.6°, q2' = 7.8 rad/s; settled to within 2° in 0.42 s |
| 3.46–30 s | balanced continuously for 26.6 s |
| 22 s | tap (3 mN·m × 50 ms, about +2.7 rad/s): rejected, pendulum ≤ 9° from upright, arm swing about 25°, no mode switch |
| 30 s | hard push (30 mN·m × 50 ms, about +27 rad/s): LQR cannot hold it, the arm reaches 1.5 rad, switch to SWING_UP (30.08 s) |
| 32.48 s | re-caught; stays balanced to the end (final error 0.09°) |

Over the whole run the peak motor torque is 0.246 N·m, with zero torque-saturated steps. The maximum arm angle is 92°, during the hard push.

**Validation matrix** (`python -m control.validate`; details in `results/validation_summary.md`). Every one of the 15 closed-loop tests passed: the 12 required ones and the 3 informative ones.

| Group | Tests | Result |
|---|---|---|
| demo | full scenario | first catch at 3.5 s; 18.4 s longest balance within 5° |
| initial conditions | hanging 30° off, arm offset, a moving pendulum, near upright on both wrap sides | all caught within 2.9 s and balanced to the end |
| servo bandwidth mismatch (controller assumes 15 Hz) | actual 8, 10, 25 Hz; and 10 Hz matched | all pass. 25 Hz takes longer to catch (11 s) and reaches 0.35 N·m peak torque. |
| timing | 3 ms latency; 100 Hz control rate | pass |
| sensor noise | 2 extra seeds | pass |
| model mismatch | 10 g added at 25 mm (ω² +16 % vs the design model) | pass |

**Outputs:**
- `results/demo_log.png`: the whole run;
- `results/demo_log_catch_zoom.png` and `results/demo_log_disturbance{1,2}_zoom.png`;
- `results/demo_swingup_lqr.mp4`: a rendered replay of the logged run.

## 9. Limitations and what to measure on the real rig

1. **Servo bandwidth and damping are placeholders.** Identify them from `arm_cmd` vs `q1_meas` logs, for example step commands with the pendulum hanging (the same test as `servo_step_test`). Then set `controller.servo_model` and rerun `control.validate`.
2. **ZDT command interface.** Check that position commands can be sent at 200 Hz, and the real latency. 3 ms passed in simulation. The stepper must not lose steps: use closed-loop mode and keep the 30 rad/s speed limit. Lower it if the driver struggles.
3. **Not modelled:**
   - microstep torque ripple;
   - missed steps;
   - backlash in the D-shaft coupling;
   - AS5600 cable drag and magnet eccentricity;
   - quantisation of the command into steps.
4. **Large swings.** Swing-up uses up to ±35° of arm travel, and the hard-push recovery used up to 92°. Make sure the encoder cable allows ±100°.
5. **Hard push.** Recovery after the hard push goes through swing-up. The arm limit (1.5 rad) is the safety boundary.
6. **Tuning.** All gains and thresholds are in `control/config.json`. Change them there, not in the code.
