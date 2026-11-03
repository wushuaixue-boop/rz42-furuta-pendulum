"""Hybrid SWING_UP / LQR_BALANCE controller (layer 3 of 3: the external controller).

Inputs : measured arm and pendulum angles at the control rate (no torque, no true velocities).
Output : q1_cmd, the arm angle command for the stepper driver.

Switching (with hysteresis and a minimum dwell time):
  SWING_UP -> LQR_BALANCE  when |wrap(q2 - pi)| < catch_angle and |q2'| < catch_rate
                           and |q1| < catch_arm_max and |q1'| < catch_arm_rate
  LQR_BALANCE -> SWING_UP  when |wrap(q2 - pi)| > drop_angle or |q1| > arm_limit
Continuity: on entering LQR the arm reference q1_eq starts at the current arm angle and ramps to 0,
and every command passes a slew limiter (the driver's speed limit), so a mode change never asks for
a jump in arm position.
"""
import math

from .lqr import LQRBalance
from .models import servo_params, wrap
from .sensors import StateEstimator
from .swingup import EnergySwingUp

SWING_UP, LQR_BALANCE = "SWING_UP", "LQR_BALANCE"


class HybridController:
    def __init__(self, ratios, cfg):
        c = cfg["controller"]
        self.Ts = 1.0 / c["rate_hz"]
        ws, zeta = servo_params(c["servo_model"])
        self.est = StateEstimator(self.Ts, cfg["estimator"]["velocity_cutoff_hz"])
        self.swing = EnergySwingUp(ratios, ws, zeta, cfg["swingup"], self.Ts)
        L = cfg["lqr"]
        self.lqr = LQRBalance(ratios, ws, zeta, self.Ts, L["Q"], L["R"])
        s = cfg["switching"]
        self.catch_angle = math.radians(s["catch_angle_deg"])
        self.catch_rate = s["catch_rate"]
        self.catch_arm_max = s["catch_arm_max"]
        self.catch_arm_rate = s["catch_arm_rate"]
        self.drop_angle = math.radians(s["drop_angle_deg"])
        self.arm_limit = s["arm_limit"]
        self.min_dwell = s["min_dwell_s"]
        self.ref_rate = s["arm_return_rate"]
        self.slew = s["cmd_slew_radps"] * self.Ts
        self.cmd_limit = s["cmd_abs_limit"]
        self.mode = SWING_UP if cfg["controller"].get("start_mode", "auto") != LQR_BALANCE else LQR_BALANCE
        self.t_mode = -1e9
        self.q1_eq = 0.0
        self.last_cmd = None
        self.first = True
        self.events = []

    def _switch(self, t, mode, reason):
        self.mode, self.t_mode = mode, t
        self.events.append((t, mode, reason))

    def update(self, t, q1m, q2m):
        q1, q2, qd1, qd2 = self.est.update(q1m, q2m)
        if self.last_cmd is None:
            self.last_cmd = q1
        delta = wrap(q2 - math.pi)
        dwell_ok = (t - self.t_mode) >= self.min_dwell

        if self.mode == SWING_UP and dwell_ok and abs(delta) < self.catch_angle and abs(qd2) < self.catch_rate \
                and abs(q1) < self.catch_arm_max and abs(qd1) < self.catch_arm_rate:
            self._switch(t, LQR_BALANCE, f"caught |d|={math.degrees(abs(delta)):.1f} deg, |q2'|={abs(qd2):.2f}")
            self.q1_eq = q1
        elif self.mode == LQR_BALANCE and dwell_ok and (abs(delta) > self.drop_angle or abs(q1) > self.arm_limit):
            self._switch(t, SWING_UP, f"lost |d|={math.degrees(abs(delta)):.1f} deg, q1={q1:.2f}")
            self.swing.reset(q1, qd1)
        if self.first:
            self.swing.reset(q1, qd1)
            self.first = False

        E = self.swing.energy(q2, qd2)
        if self.mode == LQR_BALANCE:
            step = self.ref_rate * self.Ts               # walk the arm reference back to the centre
            self.q1_eq -= max(-step, min(step, self.q1_eq))
            cmd = self.lqr.command(q1, q2, qd1, qd2, self.q1_eq)
        else:
            cmd, _, _ = self.swing.command(q1, q2, qd1, qd2)

        # continuity + driver limits: slew limit and absolute travel limit
        cmd = self.last_cmd + max(-self.slew, min(self.slew, cmd - self.last_cmd))
        cmd = max(-self.cmd_limit, min(self.cmd_limit, cmd))
        self.last_cmd = cmd
        return cmd, {"mode": self.mode, "E": E, "delta": delta, "qd1_est": qd1, "qd2_est": qd2}
