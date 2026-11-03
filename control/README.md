# control/: position-command swing-up + LQR for the RZ42 Furuta pendulum

The controller's only output is **`q1_cmd`, the arm angle sent to the ZDT stepper driver**. It never commands or reads motor torque. It is designed from `identification/identified_parameters.json` and runs against the hidden MuJoCo plant `mujoco/rz42_real_plant.xml`. Design and results: [`docs/CONTROL_REPORT.md`](../docs/CONTROL_REPORT.md).

| File | Role |
|---|---|
| `config.json` | **All tunables**: servo placeholders, sensors, rates, LQR Q/R, swing-up gains, switching thresholds, demo scenario |
| `plant.py` | Layer 1: MuJoCo plant (official `mujoco` package; `tools/libmjshim.so` fallback for Linux testing) |
| `servo.py` | Layer 2: ZDT driver model (q1_cmd → torque, speed and torque limits) |
| `sensors.py` | Encoder model (plant side) and velocity estimator (controller side) |
| `models.py` | Controller-side model from the identified ratios: A_pos, B_pos, discretisation |
| `lqr.py` | Discrete LQR about upright, with angle wrapping |
| `swingup.py` | Energy-based position-pumping swing-up |
| `hybrid.py` | SWING_UP / LQR_BALANCE switching with hysteresis and command continuity |
| `sim.py` | Closed-loop runner, CSV logging, metrics |
| `run_demo.py` | Headless demo → `results/demo_log.{csv,json,png}` |
| `viewer.py` | Live MuJoCo viewer of the same closed loop; also `--replay` of a log |
| `validate.py` | Automated tests → `results/validation_summary.{md,json}` |
| `plots.py` | Plots from a log |

## Run it (Windows PowerShell)

Run these from the project folder (the one containing `control\`). They need Python 3.10–3.12; `py -3.12` is the Python launcher.

```powershell
cd $HOME\Desktop\RZ42_furuta_pendulum
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt

.\.venv\Scripts\python.exe -m control.viewer                       # live 3-D view: swing-up, LQR, tap, push, recovery
.\.venv\Scripts\python.exe -m control.run_demo                     # headless run + log + plots in results\
.\.venv\Scripts\python.exe -m control.validate                     # automated tests (about 1 min)
.\.venv\Scripts\python.exe -m control.plots results\demo_log.csv   # re-make the plots from a log
```

**Viewer:**
- Pendulum colour: **orange = SWING_UP**, **green = LQR_BALANCE**. Mode changes are also printed in the console.
- **F6** = tap, **F7** = hard push. Both are physical torques on the pendulum.
- `--speed 0.5` gives slow motion; `--replay results\demo_log.csv` replays a logged run.

**Overriding a setting without editing the file:**
```powershell
.\.venv\Scripts\python.exe -m control.run_demo --set servo.omega_s_hz=10 --set scenario.seed=3 --out results\bw10.csv
```

## Moving to the real rig
1. **Identify the servo:** fit `servo_model.omega_s_hz` and `zeta_s` from `arm_cmd` vs `q1_meas` step logs, then rerun `control.validate`.
2. **Port the controller:** port `hybrid.py`, `swingup.py`, `lqr.py` and `sensors.StateEstimator` to the microcontroller. All are plain arithmetic on two angles per tick; the gain K is printed in `results/demo_log.json`.
3. **Safety first:** keep `switching.arm_limit` and `cmd_abs_limit` conservative, and start with the pendulum held near upright before trying swing-up.
