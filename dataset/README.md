# Dataset

Encoder-only identification logs from the **hidden "real" plant** (`mujoco/rz42_real_plant*.xml`), made by `mujoco/run_experiments.py`. The experiments are exactly the ones in `identification/EXPERIMENT_PROTOCOL.md`, so the same files can be recorded on the physical rig.

| File | Content |
|---|---|
| `experiments/*.csv` | One file per experiment, logged at 500 Hz. Columns: `t` [s] loop timestamp (with jitter); `arm_cmd` [rad] command sent to the stepper; `q1_meas` [rad] stepper encoder; `q2_meas` [rad] AS5600, unwrapped, with its raw mounting offset. **No torque.** |
| `experiments.json` | Logged columns, nominal sensor specs, and for each run its purpose (`calibration` / `train` / `scale` / `validation`), setup and arm mode. |
| `bench.json` | Values measured on the **assembled** rig: added mass Δm, its size, distances d1 and d2 (calipers), r from the CAD, and optionally m_p if the pendulum was weighed before assembly. No centres of mass. |
| `ground_truth.json` | True parameters of the hidden plant and the hidden AS5600 offset. Read **only** by `identification/03_evaluate_against_truth.ipynb`. A real rig has no such file. |
| `nominal_model_sine/` | Sine response of the *nominal* CAD model, used by `01_kane_model.ipynb` to check the Kane equations against MuJoCo. |

**Sensor model.** These are the effects a real logger adds, and the notebook handles each one:

| Effect | Value |
|---|---|
| loop timestamp jitter | 50 µs rms |
| stepper encoder | 14 bit, 0.5 count noise |
| AS5600 | 12 bit, 0.6 count noise |
| AS5600 zero offset | about 3°, unknown to the identification |

**Bench measurement errors.** The bench values are deliberately slightly off from the true ones, as real measurements would be:
* Δm is 10.0 g against a true 10.03 g;
* d1 and d2 are 25.0 and 45.0 mm against a true 25.3 and 45.2 mm.

**Feasibility.** Every experiment was checked inside the simulation to need under 25 mN·m of motor torque, so it is safe for a 42×48 stepper. That torque is never written to the dataset.
