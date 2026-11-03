"""ZDT X42S/Y42 closed-loop stepper in position mode (layer 2 of 3: the driver's own controller).

On the real rig you send a target angle q1_cmd over UART/CAN and the driver closes its own fast loop
on its own encoder. This class is a model of that driver. It is the ONLY thing that produces motor
torque; the external swing-up / LQR controller never sees or sets torque.

Design target (the servo approximation used for control design):
    q1'' = ws^2 (q1_cmd - q1) - 2 zeta ws q1'
Realised as a PD torque law tuned for the load inertia the driver *assumes* (J_hat):
    tau = J_hat * [ ws^2 (r - q1_enc) - 2 zeta ws q1' ]
On the true coupled plant the response differs: the arm inertia is not J_hat, and the swinging
pendulum pushes back on the arm. tests/validate.py measures the real step response.

Limits modelled:
  - speed limit of the position reference (the driver's "max speed" setting): r -> q1_cmd at <= v_max
  - torque limit that falls with speed (stepper torque-speed curve, linear approximation)
  - 14-bit driver encoder
"""
import math


class ZDTServo:
    def __init__(self, cfg):
        self.ws = 2 * math.pi * cfg["omega_s_hz"]
        self.zeta = cfg["zeta_s"]
        self.J_hat = cfg["assumed_load_inertia"]
        self.kp = self.J_hat * self.ws**2
        self.kd = self.J_hat * 2 * self.zeta * self.ws
        self.t_hold = cfg["torque_hold"]
        self.t_derate = cfg["torque_derate_per_radps"]
        self.t_min = cfg["torque_min"]
        self.v_max = cfg["v_max"]
        self.lsb = 2 * math.pi / 2 ** cfg["encoder_bits"]
        self.r = 0.0
        self.cmd = 0.0
        self.saturated = False

    def reset(self, q1):
        self.r = q1
        self.cmd = q1

    def set_command(self, q1_cmd):
        self.cmd = q1_cmd

    def torque_limit(self, qd1):
        return max(self.t_min, self.t_hold - self.t_derate * abs(qd1))

    def torque(self, q1, qd1, dt):
        # speed-limited internal reference
        step = self.v_max * dt
        e_r = self.cmd - self.r
        self.r += max(-step, min(step, e_r))
        q1_enc = round(q1 / self.lsb) * self.lsb
        tau = self.kp * (self.r - q1_enc) - self.kd * qd1
        lim = self.torque_limit(qd1)
        self.saturated = abs(tau) > lim
        return max(-lim, min(lim, tau))
