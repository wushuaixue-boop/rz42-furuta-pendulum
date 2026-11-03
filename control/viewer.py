"""Live MuJoCo viewer of the closed loop (needs the `mujoco` Python package).

  python -m control.viewer                    # swing-up -> LQR -> tap -> hard push -> recovery, real time
  python -m control.viewer --speed 0.5        # slow motion
  python -m control.viewer --replay results/demo_log.csv   # replay a logged run (visual only)

Pendulum colour shows the controller mode: orange = SWING_UP, green = LQR_BALANCE.
The mode, time and angles are also shown in the window's top-left corner (if your mujoco version
supports viewer.set_texts) and printed in the console at every mode change.
Keys: F6 = moderate tap on the pendulum, F7 = hard push (both are physical external torques).

The closed loop is exactly the one of control.sim: the viewer only displays it. The arm is never
moved by writing qpos/qvel; it is driven by the simulated stepper driver's torque.
"""
import argparse
import math
import time

import numpy as np

from .hybrid import LQR_BALANCE
from .sim import ClosedLoop, load_config

ORANGE = np.array([0.95, 0.55, 0.15, 1.0])
GREEN = np.array([0.20, 0.75, 0.35, 1.0])
KEY_F6, KEY_F7 = 295, 296          # GLFW key codes


def setup_camera(v):
    v.cam.lookat[:] = [0.0, -0.03, 0.09]
    v.cam.distance = 0.42
    v.cam.azimuth = 60.0
    v.cam.elevation = -15.0


def overlay(v, text_left, text_right):
    try:
        import mujoco
        v.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT, text_left, text_right))
    except Exception:
        pass                               # older mujoco: colour + console output only


def live(args):
    import mujoco
    import mujoco.viewer
    cfg = load_config(args.config)
    if args.duration:
        cfg["scenario"]["duration"] = args.duration
    cl = ClosedLoop(cfg, backend="mujoco")
    sc = cfg["scenario"]
    cl.reset(sc.get("q1_0", 0.0), sc.get("q2_0", 0.0), sc.get("qd1_0", 0.0), sc.get("qd2_0", 0.0))
    m, d = cl.plant.m, cl.plant.d
    g_pen = m.geom("pendulum").id
    pending = []

    def on_key(key):
        t = d.time
        if key == KEY_F6:
            pending.append({"t": t, "torque": 0.003, "duration": 0.05})
            print(f"[{t:6.2f} s] tap (F6)")
        elif key == KEY_F7:
            pending.append({"t": t, "torque": 0.03, "duration": 0.05})
            print(f"[{t:6.2f} s] hard push (F7)")

    last_mode = None
    with mujoco.viewer.launch_passive(m, d, key_callback=on_key) as v:
        setup_camera(v)
        wall0 = time.perf_counter()
        while v.is_running() and d.time < sc["duration"]:
            while pending:
                cl.dist.append(pending.pop())
            target = (time.perf_counter() - wall0) * args.speed
            n = 0
            while d.time < target and n < 2000:
                cl.step()
                n += 1
            mode = cl.ctrl.mode
            if mode != last_mode:
                print(f"[{d.time:6.2f} s] mode -> {mode}")
                last_mode = mode
            m.geom_rgba[g_pen] = GREEN if mode == LQR_BALANCE else ORANGE
            _, q1, q2, _, _ = cl.plant.state()
            delta = math.degrees(abs(((q2 - math.pi) + math.pi) % (2 * math.pi) - math.pi))
            overlay(v, "mode\ntime\nfrom upright\narm\narm command",
                    f"{mode}\n{d.time:.2f} s\n{delta:.1f} deg\n{math.degrees(q1):+.1f} deg\n{math.degrees(cl.cmd):+.1f} deg")
            v.sync()
            time.sleep(0.005)
        print("done; close the window to exit")
        while v.is_running():
            v.sync()
            time.sleep(0.05)


def replay(args):
    import mujoco
    import mujoco.viewer
    from .plant import PLANTS
    cfg = load_config(args.config)
    m = mujoco.MjModel.from_xml_path(str(PLANTS[cfg["plant"]["name"]]))
    d = mujoco.MjData(m)
    raw = np.genfromtxt(args.replay, delimiter=",", names=True)
    qa, qp = m.joint("arm_joint").qposadr[0], m.joint("pendulum_joint").qposadr[0]
    g_pen = m.geom("pendulum").id
    with mujoco.viewer.launch_passive(m, d) as v:
        setup_camera(v)
        wall0 = time.perf_counter()
        k = 0
        while v.is_running() and k < len(raw):
            tw = (time.perf_counter() - wall0) * args.speed
            while k < len(raw) - 1 and raw["t"][k] < tw:
                k += 1
            d.qpos[qa], d.qpos[qp] = raw["q1"][k], raw["q2"][k]
            mujoco.mj_forward(m, d)                     # replay of a logged run: display only
            lqr = raw["mode_lqr"][k] > 0.5
            m.geom_rgba[g_pen] = GREEN if lqr else ORANGE
            overlay(v, "REPLAY\nmode\ntime", f"{args.replay}\n{'LQR_BALANCE' if lqr else 'SWING_UP'}\n{raw['t'][k]:.2f} s")
            v.sync()
            time.sleep(1 / 60)
            if k == len(raw) - 1:
                break


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--speed", type=float, default=1.0, help="1 = real time")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--replay", default=None, help="CSV log to replay instead of simulating")
    args = ap.parse_args(argv)
    replay(args) if args.replay else live(args)


if __name__ == "__main__":
    main()
