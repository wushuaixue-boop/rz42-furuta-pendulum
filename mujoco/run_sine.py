"""Drive the RZ42 v5 rotary inverted pendulum in MuJoCo with a sine arm command.

    pip install mujoco matplotlib          (once)
    python run_sine.py                     # viewer, 0.3 rad at 1.0 Hz
    python run_sine.py --amp 0.3 --freq 1.9            # near the pendulum's natural frequency
    python run_sine.py --amp 0 --theta0 30             # free swing from 30 deg (damping check)
    python run_sine.py --amp 0.3 --freq 1.9 --headless --time 20   # no window: saves CSV + plot

Arm command (rad) = amp * fade(t) * sin(2*pi*freq*t); fade() rises smoothly 0 -> 1 over --ramp seconds
so the arm does not start with a velocity jump. Angles: arm 0 = pointing along -y, pendulum 0 = hanging.
On macOS use `mjpython run_sine.py` for the viewer.
"""
import argparse
import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

HERE = Path(__file__).resolve().parent


def command(t, amp, freq, ramp):
    env = 0.5 * (1 - math.cos(math.pi * t / ramp)) if (ramp > 0 and t < ramp) else 1.0
    return amp * env * math.sin(2 * math.pi * freq * t)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--amp", type=float, default=0.3, help="arm amplitude, rad (0.3 rad = 17 deg)")
    ap.add_argument("--freq", type=float, default=1.0, help="sine frequency, Hz")
    ap.add_argument("--ramp", type=float, default=1.0, help="fade-in time, s (0 = pure sine from t=0)")
    ap.add_argument("--theta0", type=float, default=0.0, help="initial pendulum angle, deg")
    ap.add_argument("--time", type=float, default=30.0, help="duration, s")
    ap.add_argument("--headless", action="store_true", help="no viewer; write CSV and a PNG plot")
    ap.add_argument("--model", default=str(HERE / "rz42_pendulum.xml"))
    a = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(a.model)
    d = mujoco.MjData(m)
    qa = m.joint("arm_joint").qposadr[0]; qp = m.joint("pendulum_joint").qposadr[0]
    va = m.joint("arm_joint").dofadr[0]; vp = m.joint("pendulum_joint").dofadr[0]
    d.qpos[qp] = math.radians(a.theta0)
    mujoco.mj_forward(m, d)
    log = []

    def step():
        d.ctrl[0] = command(d.time, a.amp, a.freq, a.ramp)
        log.append((d.time, d.ctrl[0], d.qpos[qa], d.qvel[va], d.qpos[qp], d.qvel[vp], d.actuator_force[0]))
        mujoco.mj_step(m, d)

    if a.headless:
        while d.time < a.time:
            step()
    else:
        with mujoco.viewer.launch_passive(m, d) as v:
            v.cam.lookat[:] = [0.0, -0.045, 0.085]
            v.cam.distance, v.cam.azimuth, v.cam.elevation = 0.36, 62, -12
            t0 = time.perf_counter()
            while v.is_running() and d.time < a.time:
                for _ in range(10):                    # 10 physics steps (10 ms) per frame
                    step()
                v.sync()
                lag = d.time - (time.perf_counter() - t0)   # real-time pacing
                if lag > 0:
                    time.sleep(lag)

    L = np.array(log)
    out = HERE / f"run_amp{a.amp:g}_f{a.freq:g}.csv"
    np.savetxt(out, L[::5], delimiter=",", header="t,arm_cmd,arm,arm_vel,pendulum,pendulum_vel,motor_torque", comments="")
    after = L[:, 0] > 0.2
    print(f"amp {a.amp} rad, freq {a.freq} Hz, {L[-1,0]:.1f} s: max |pendulum| {np.degrees(np.abs(L[:,4]).max()):.1f} deg, "
          f"max |torque| (t>0.2 s) {np.abs(L[after,6]).max():.3f} N*m, max tracking error "
          f"{np.degrees(np.abs(L[after,1]-L[after,2]).max()):.2f} deg  ->  {out.name}")
    if a.headless:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        ax[0].plot(L[:, 0], np.degrees(L[:, 2]), lw=2, label="arm"); ax[0].plot(L[:, 0], np.degrees(L[:, 1]), "k--", lw=1, label="command")
        ax[0].set_ylabel("arm (deg)"); ax[0].legend(frameon=False)
        ax[1].plot(L[:, 0], np.degrees(L[:, 4]), color="#eb6834"); ax[1].set_ylabel("pendulum (deg)"); ax[1].set_xlabel("time (s)")
        fig.tight_layout(); fig.savefig(out.with_suffix(".png"), dpi=110)


if __name__ == "__main__":
    main()
