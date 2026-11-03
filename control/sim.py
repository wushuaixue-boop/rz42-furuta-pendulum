"""Closed-loop simulation: MuJoCo plant  <-  ZDT servo  <-  q1_cmd  <-  hybrid controller  <-  encoders.

Timing (all configurable in config.json):
  - physics + driver loop at the plant timestep (10 kHz for rz42_real_plant.xml)
  - external controller at controller.rate_hz (200 Hz), zero-order hold on q1_cmd
  - measurement -> command latency controller.latency_s (UART/CAN transfer + compute)

The external controller receives only the two encoder angles. Motor torque is logged for
diagnostics after the fact; it is never passed to the controller.
"""
import copy
import json
import math
from pathlib import Path

import numpy as np

from .hybrid import HybridController, LQR_BALANCE
from .models import load_identified
from .plant import make_plant
from .sensors import EncoderSensors
from .servo import ZDTServo

HERE = Path(__file__).resolve().parent


def load_config(path=None, overrides=None):
    cfg = json.loads(Path(path or HERE / "config.json").read_text())
    for key, val in (overrides or {}).items():          # "section.key": value
        d = cfg
        parts = key.split(".")
        for p in parts[:-1]:
            d = d[p]
        d[parts[-1]] = val
    return cfg


class ClosedLoop:
    def __init__(self, cfg, backend="auto"):
        self.cfg = copy.deepcopy(cfg)
        self.ratios, _ = load_identified(cfg["controller"].get("params_file"))
        self.plant = make_plant(cfg["plant"]["name"], backend)
        self.dt = self.plant.dt
        self.servo = ZDTServo(cfg["servo"])
        self.sensors = EncoderSensors(cfg["sensors"], seed=cfg["scenario"].get("seed", 0))
        self.ctrl = HybridController(self.ratios, cfg)
        self.n_ctrl = max(1, round(self.ctrl.Ts / self.dt))
        self.n_lat = max(0, round(cfg["controller"]["latency_s"] / self.dt))

    def reset(self, q1=0.0, q2=0.0, qd1=0.0, qd2=0.0):
        self.plant.reset(q1, q2, qd1, qd2)
        self.servo.reset(q1)
        self.k = 0
        self.pending = []                   # (activation step, q1_cmd)
        self.cmd = q1
        self.info = {"mode": self.ctrl.mode, "E": 0.0, "delta": 0.0}
        self.dist = sorted(self.cfg["scenario"].get("disturbances", []), key=lambda d: d["t"])

    def disturbance(self, t):
        for d in self.dist:
            if d["t"] <= t < d["t"] + d["duration"]:
                return d["torque"]
        return 0.0

    def step(self):
        """advance one physics step; returns a log row at control ticks, else None"""
        t, q1, q2, qd1, qd2 = self.plant.state()
        row = None
        if self.k % self.n_ctrl == 0:
            q1m, q2m = self.sensors.measure(q1, q2)
            self.cmd, self.info = self.ctrl.update(t, q1m, q2m)
            self.pending.append((self.k + self.n_lat, self.cmd))
            row = [t, q1, q2, qd1, qd2, q1m, q2m, self.cmd, 0.0, 1.0 if self.info["mode"] == LQR_BALANCE else 0.0,
                   self.info["E"], self.info["delta"], 0.0, 0.0]
        while self.pending and self.pending[0][0] <= self.k:
            self.servo.set_command(self.pending.pop(0)[1])
        tau_d = self.disturbance(t)
        self.plant.set_pendulum_torque(tau_d)
        tau = self.servo.torque(q1, qd1, self.dt)
        self.plant.step(tau)
        self.k += 1
        self._tau, self._sat, self._dist = tau, self.servo.saturated, tau_d
        if row is not None:
            row[8], row[12], row[13] = tau, 1.0 if self.servo.saturated else 0.0, tau_d
        return row


COLUMNS = ["t", "q1", "q2", "qd1", "qd2", "q1_meas", "q2_meas", "q1_cmd", "motor_torque_diag",
           "mode_lqr", "energy", "delta_upright", "torque_saturated", "disturbance_torque"]


def run(cfg, backend="auto", progress=False):
    cl = ClosedLoop(cfg, backend)
    sc = cfg["scenario"]
    cl.reset(sc.get("q1_0", 0.0), sc.get("q2_0", 0.0), sc.get("qd1_0", 0.0), sc.get("qd2_0", 0.0))
    n = int(round(sc["duration"] / cl.dt))
    rows, peak_tau, sat_steps = [], 0.0, 0
    for i in range(n):
        r = cl.step()
        peak_tau = max(peak_tau, abs(cl._tau))
        sat_steps += cl._sat
        if r is not None:
            rows.append(r)
        if progress and i % (n // 10 or 1) == 0:
            print(f"  t = {i*cl.dt:5.1f} s  mode {cl.ctrl.mode}")
    log = {c: np.array([r[j] for r in rows]) for j, c in enumerate(COLUMNS)}
    return log, {"events": cl.ctrl.events, "peak_torque": peak_tau, "sat_fraction": sat_steps / n,
                 "K": cl.ctrl.lqr.K.tolist(), "cl_eig_abs": np.abs(cl.ctrl.lqr.cl_eig).tolist(),
                 "ratios": cl.ratios, "dt": cl.dt, "Ts": cl.ctrl.Ts}


def save_log(log, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(path, np.column_stack([log[c] for c in COLUMNS]), delimiter=",", fmt="%.6g",
               header=",".join(COLUMNS), comments="")


def metrics(log, meta, cfg):
    """numbers used by the report and the validation tests"""
    t, lqr = log["t"], log["mode_lqr"] > 0.5
    delta = np.abs(log["delta_upright"])
    Ts = meta["Ts"]
    m = {}
    caught = np.flatnonzero(lqr)
    m["first_catch_s"] = float(t[caught[0]]) if caught.size else None
    m["n_switches"] = len(meta["events"])
    # balanced = in LQR with |delta| < 5 deg; longest continuous balanced stretch
    bal = lqr & (delta < math.radians(5))
    best, cur = 0, 0
    for b in bal:
        cur = cur + 1 if b else 0
        best = max(best, cur)
    m["longest_balance_s"] = best * Ts
    # settling after the first catch: time until |delta| stays < 2 deg for 1 s
    m["settle_after_catch_s"] = None
    if caught.size:
        k0 = caught[0]
        win = int(1.0 / Ts)
        ok = delta < math.radians(2)
        for k in range(k0, len(t) - win):
            if ok[k:k + win].all() and lqr[k:k + win].all():
                m["settle_after_catch_s"] = float(t[k] - t[k0])
                break
    sel = lqr.copy()
    if caught.size:
        sel[:caught[0] + int(2.0 / Ts)] = False             # steady balancing: from 2 s after the catch
    if sel.any():
        m["balance_rms_delta_deg"] = float(np.degrees(np.sqrt(np.mean(log["delta_upright"][sel]**2))))
        m["balance_max_delta_deg"] = float(np.degrees(delta[sel].max()))
        m["balance_max_arm_deg"] = float(np.degrees(np.abs(log["q1"][sel]).max()))
    m["max_arm_deg"] = float(np.degrees(np.abs(log["q1"]).max()))
    m["max_cmd_rate_radps"] = float(np.abs(np.diff(log["q1_cmd"])).max() / Ts)
    m["max_arm_speed_radps"] = float(np.abs(log["qd1"]).max())
    m["peak_motor_torque_Nm"] = meta["peak_torque"]
    m["torque_saturated_fraction"] = meta["sat_fraction"]
    m["final_mode"] = "LQR_BALANCE" if lqr[-1] else "SWING_UP"
    m["final_delta_deg"] = float(np.degrees(delta[-1]))
    return m
