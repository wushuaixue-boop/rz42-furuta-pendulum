"""Run the identification experiments on the hidden "real" plant and write encoder-only logs.

Everything here is something you can do on the assembled rig, with the stepper powered:
  calib_rest          pendulum hanging at rest, arm held: zero offset of the AS5600
  free_decay_*        arm held, pendulum lifted by hand and released
  sine_*, chirp_*     arm follows a position command, pendulum starts hanging at rest
  swing_*, pump_*     large / resonant arm motion: large pendulum angles (swing-up range)
  massD1_* / massD2_* free decays with a known mass taped to the pendulum (absolute scale)
  *_val               independent runs kept for validation only

Logged columns (../dataset/experiments/<name>.csv), the only signals a real rig gives:
  t [s]        timestamp of the control loop iteration (with jitter)
  arm_cmd [rad] arm position command sent to the stepper driver
  q1_meas [rad] stepper (arm) encoder
  q2_meas [rad] AS5600 (pendulum) angle, unwrapped by the firmware, raw zero (mounting offset!)
No torque is logged. The simulated motor torque is only used inside this script to check
that every experiment stays within the stepper's torque limit (printed, never saved).

usage:
  python run_experiments.py                               # mujoco Python package
  python run_experiments.py --runner ../tools/mj_runner   # compiled C runner (Linux)
"""
import argparse
import json
import subprocess
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "dataset" / "experiments"
MODELS = {"plain": HERE / "rz42_real_plant.xml",
          "mass_d1": HERE / "rz42_real_plant_mass_d1.xml",
          "mass_d2": HERE / "rz42_real_plant_mass_d2.xml"}
DT_SIM = 0.0001         # model timestep (10 kHz)
DECIM = 20              # 10 kHz -> 500 Hz control/logging loop
SEED = 2026
TORQUE_LIMIT = 0.45     # N m, stepper holding torque class of a 42x48 motor (feasibility check only)

SENSORS = {
    "loop_rate_Hz": 500.0,
    "timestamp_jitter_s": 5e-5,       # rms jitter of the logged loop timestamp
    "arm_encoder_bits": 14,           # stepper closed-loop encoder, 16384 counts/rev
    "arm_encoder_noise_counts": 0.5,
    "as5600_bits": 12,                # 4096 counts/rev
    "as5600_noise_counts": 0.6,
    "as5600_zero_offset_rad": 0.0541, # magnet/sensor mounting offset (unknown to the identification)
}


def fade(t, T, ramp):
    """cosine fade-in over `ramp` s and fade-out over the last `ramp` s"""
    e = np.ones_like(t)
    a = t < ramp
    e[a] = 0.5 * (1 - np.cos(np.pi * t[a] / ramp))
    b = t > T - ramp
    e[b] = 0.5 * (1 - np.cos(np.pi * (T - t[b]) / ramp))
    return e


def sine(A, f, T):
    return lambda t: A * fade(t, T, 1.0) * np.sin(2 * np.pi * f * t)


def experiments():
    """name -> dict(T, q2_0 [rad], cmd(t), model, purpose)"""
    hold = lambda t: 0 * t
    ex = {"calib_rest": dict(T=5.0, q2_0=0.0, cmd=hold, model="plain", purpose="calibration")}
    for deg in (10, 30, 60, 90, 150):
        ex[f"free_decay_{deg:03d}deg"] = dict(T=15.0, q2_0=np.deg2rad(deg), cmd=hold, model="plain", purpose="train")
    for A, f in [(0.20, 0.8), (0.20, 1.5), (0.15, 1.95), (0.20, 2.5), (0.20, 3.0)]:
        ex[f"sine_A{A:.2f}_f{f:.2f}"] = dict(T=10.0, q2_0=0.0, cmd=sine(A, f, 10.0), model="plain", purpose="train")

    def chirp(t, T=30.0, f0=0.3, f1=4.0, A=0.15):
        k = (f1 - f0) / T
        return A * fade(t, T, 1.0) * np.sin(2 * np.pi * (f0 * t + 0.5 * k * t**2))
    ex["chirp_0.3-4Hz"] = dict(T=30.0, q2_0=0.0, cmd=chirp, model="plain", purpose="train")
    for A, f in [(0.8, 1.2), (1.0, 0.8)]:
        ex[f"swing_A{A:.2f}_f{f:.2f}"] = dict(T=12.0, q2_0=0.0, cmd=sine(A, f, 12.0), model="plain", purpose="train")
    # resonant pumping with a falling frequency (the swing slows down at large amplitude):
    # pendulum up to ~120 deg, the swing-up range
    def pump(t, T=15.0, f0=1.95, f1=1.3, A=0.30):
        k = (f1 - f0) / T
        return A * fade(t, T, 1.0) * np.sin(2 * np.pi * (f0 * t + 0.5 * k * t**2))
    ex["pump_A0.30_f1.95-1.30"] = dict(T=15.0, q2_0=0.0, cmd=pump, model="plain", purpose="train")
    # added-mass free decays (absolute scale)
    for deg in (20, 60):
        ex[f"massD1_decay_{deg:03d}deg"] = dict(T=15.0, q2_0=np.deg2rad(deg), cmd=hold, model="mass_d1", purpose="scale")
    # ---- validation: independent runs, never used in fitting ----
    rng = np.random.default_rng(7)
    freqs = np.array([0.6, 1.1, 1.7, 2.3, 2.9, 3.6])
    phases = rng.uniform(0, 2 * np.pi, freqs.size)
    amps = np.array([0.12, 0.10, 0.06, 0.06, 0.05, 0.04])
    ex["multisine_val"] = dict(T=20.0, q2_0=0.0, model="plain", purpose="validation",
                               cmd=lambda t: fade(t, 20.0, 1.0) * np.sum(
                                   amps[:, None] * np.sin(2 * np.pi * freqs[:, None] * t + phases[:, None]), axis=0))
    ex["swing_A0.60_f1.30_val"] = dict(T=12.0, q2_0=0.0, cmd=sine(0.6, 1.3, 12.0), model="plain", purpose="validation")
    ex["free_decay_045deg_val"] = dict(T=15.0, q2_0=np.deg2rad(45), cmd=hold, model="plain", purpose="validation")
    ex["massD2_decay_040deg_val"] = dict(T=15.0, q2_0=np.deg2rad(40), cmd=hold, model="mass_d2", purpose="validation")
    return ex


def simulate_python(model, cmd, q2_0):
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(model))
    d = mujoco.MjData(m)
    kp, kv, tmax = (float(m.numeric(n).data[0]) for n in ("servo_kp", "servo_kv", "servo_tmax"))
    ja, jp = m.joint("arm_joint"), m.joint("pendulum_joint")
    qa, qp, va, vp = ja.qposadr[0], jp.qposadr[0], ja.dofadr[0], jp.dofadr[0]
    d.qpos[qp] = q2_0
    mujoco.mj_forward(m, d)
    out = np.empty((cmd.size, 7))
    for k, c in enumerate(cmd):
        # stepper driver position loop, applied as a torque (same as tools/mj_runner.c)
        tau = min(max(kp * (c - d.qpos[qa]) - kv * d.qvel[va], -tmax), tmax)
        d.ctrl[0] = tau
        out[k] = (d.time, c, d.qpos[qa], d.qpos[qp], d.qvel[va], d.qvel[vp], tau)
        mujoco.mj_step(m, d)
    return out


def simulate_runner(runner, model, cmd, q2_0):
    with tempfile.TemporaryDirectory() as tmp:
        fc, fo = Path(tmp) / "cmd.csv", Path(tmp) / "out.csv"
        np.savetxt(fc, cmd, fmt="%.9f")
        subprocess.run([str(runner), str(model), str(fc), repr(float(q2_0)), str(fo)], check=True)
        return np.loadtxt(fo, delimiter=",", skiprows=1)


def sensor_model(true, rng):
    """true columns: t, cmd, q1, q2, qd1, qd2, torque  ->  t, cmd, q1_meas, q2_meas (no torque)"""
    s = SENSORS
    lsb_a = 2 * np.pi / 2 ** s["arm_encoder_bits"]
    lsb_p = 2 * np.pi / 2 ** s["as5600_bits"]
    # the loop samples at nominal 500 Hz; the logged timestamp carries the loop's timing jitter
    k = np.arange(0, len(true), DECIM)
    jitter = rng.normal(0, s["timestamp_jitter_s"], k.size)
    # sample the true signal at the jittered instant (linear interpolation of the 10 kHz truth)
    t_true = true[:, 0]
    t_s = np.clip(t_true[k] + jitter, t_true[0], t_true[-1])
    q1 = np.interp(t_s, t_true, true[:, 2])
    q2 = np.interp(t_s, t_true, true[:, 3]) + s["as5600_zero_offset_rad"]
    cmd = true[k, 1]
    q1 = np.round((q1 + rng.normal(0, s["arm_encoder_noise_counts"] * lsb_a, k.size)) / lsb_a) * lsb_a
    q2 = np.round((q2 + rng.normal(0, s["as5600_noise_counts"] * lsb_p, k.size)) / lsb_p) * lsb_p
    return np.column_stack([t_s, cmd, q1, q2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runner", help="path to the compiled C runner (tools/mj_runner); default: mujoco Python package")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    for old in OUT.glob("*.csv"):
        old.unlink()
    rng = np.random.default_rng(SEED)
    index = {}
    for name, e in experiments().items():
        t = np.arange(0.0, e["T"], DT_SIM)
        cmd = e["cmd"](t)
        model = MODELS[e["model"]]
        true = simulate_runner(args.runner, model, cmd, e["q2_0"]) if args.runner else simulate_python(model, cmd, e["q2_0"])
        meas = sensor_model(true, rng)
        np.savetxt(OUT / f"{name}.csv", meas, delimiter=",", fmt=["%.5f", "%.6f", "%.6f", "%.6f"],
                   header="t,arm_cmd,q1_meas,q2_meas", comments="")
        peak_tau = np.abs(true[:, 6]).max()          # feasibility only, never written to the dataset
        assert peak_tau < 0.8 * TORQUE_LIMIT, f"{name}: needs {peak_tau:.3f} N m, too close to the motor limit"
        index[name] = {"purpose": e["purpose"], "duration_s": e["T"],
                       "setup": {"plain": "rig as built",
                                 "mass_d1": "added mass at distance d1 (bench.json)",
                                 "mass_d2": "added mass at distance d2 (bench.json)"}[e["model"]],
                       "arm": "held at 0 (driver enabled)" if np.all(cmd == 0) else "position command (see arm_cmd)",
                       "pendulum_release_deg": float(np.rad2deg(e["q2_0"])) if e["q2_0"] else "hanging at rest"}
        print(f"{name:26s} {e['purpose']:11s} max|q2| {np.rad2deg(np.abs(true[:, 3]).max()):7.1f} deg   "
              f"(motor torque peak {peak_tau*1e3:5.1f} mN m, limit {TORQUE_LIMIT*1e3:.0f})")
    (OUT.parent / "experiments.json").write_text(json.dumps(
        {"logged_columns": ["t", "arm_cmd", "q1_meas", "q2_meas"], "sensors_nominal": {
            k: v for k, v in SENSORS.items() if k != "as5600_zero_offset_rad"},
         "experiments": index}, indent=2))
    # the hidden sensor offset is part of the truth used only for scoring
    gt_path = OUT.parent / "ground_truth.json"
    gt = json.loads(gt_path.read_text())
    gt["sensors"] = SENSORS
    gt_path.write_text(json.dumps(gt, indent=2))


if __name__ == "__main__":
    main()
