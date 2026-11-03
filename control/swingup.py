"""Energy-based swing-up through the arm position command (bounded arm travel by construction).

Pendulum energy per unit J2, zero when hanging at rest. It uses the identified omega2 and is exact
at any angle (no small-angle or linearisation assumption):
    E = 1/2 q2'^2 + omega2 (1 - cos q2),        E_up = 2 omega2  (upright, at rest)
Along the identified nonlinear pendulum model (a = q1'' is the arm acceleration):
    dE/dt = -kappa q2' cos(q2) a  - [beta q2' + phi tanh(q2'/v_s) - rho sin q2 cos q2 q1'^2] q2'
Only the first term can be shaped by the arm.

Why not command the acceleration directly (Astrom-Furuta a = -k (E_up-E) sgn(q2' cos q2))?
Its sign changes once per pendulum swing but its mean is not zero, so the arm position (the double
integral of a) drifts and runs into the arm's travel limit. Tested on the hidden plant, the
energy then plateaus at about 80 % of E_up.

Position pumping instead makes the arm angle itself follow the pendulum velocity:
    q1_cmd = sat( c (1 - E/E_up) q2' cos q2 ,  A_max )
For a swinging pendulum q2' cos q2 oscillates at the swing frequency w, so the arm follows it with
q1'' ~ -w^2 q1, and the shaped term becomes
    -kappa q2' cos q2 * q1'' ~ +kappa c w^2 (1 - E/E_up) (q2' cos q2)^2  >= 0   while E < E_up
Above E_up the pumping is switched off (gain max(0, 1 - E/E_up)): actively removing energy
through the low-passed signal is unreliable once the pendulum spins, so the bearing friction does it.
Arm travel never exceeds A_max, and the command
is a pure state feedback computed every tick from the measured/estimated pendulum state.
At exact rest (E = 0, q2' = 0) the law gives no direction; a short arm step ("kick") starts the swing.
The kick is latched for `kick_duration` s and re-armed only after the energy has stayed tiny for
`kick_rearm_s`, so noise near rest cannot make the command chatter between kick and pumping.

Pumping signal filter: q2' itself contains the fast reaction of the pendulum to arm accelerations
(q2'' = -kappa cos q2 q1'' + ...). Feeding q2' straight back into q1_cmd closes a fast loop through
that coupling (observed on the hidden plant: a ~13 Hz limit cycle and no energy gain). So the
pumping signal q2' cos q2 is low-passed at `pump_filter_hz` (a few Hz, just above the 2 Hz
swing). Only the slow swing is pumped; the small phase lag only costs a little efficiency.
"""
import math


class EnergySwingUp:
    def __init__(self, ratios, ws, zeta, cfg, Ts):
        self.w2, self.kappa = ratios["omega2"], ratios["kappa"]
        self.E_up = 2 * self.w2 * (1 + cfg["energy_margin"])
        self.c = cfg["pump_gain"]
        self.A_max = cfg["arm_amplitude_max"]
        self.kick_amp, self.kick_E = cfg["kick_amplitude"], cfg["kick_energy"]
        self.Ts = Ts
        self.alpha = 1.0 - math.exp(-2 * math.pi * cfg["pump_filter_hz"] * Ts)
        self.sf = 0.0
        self.kick_T, self.kick_rearm = cfg["kick_duration"], cfg["kick_rearm_s"]
        self.t = 0.0
        self.kick_until = -1.0
        self.low_since = 0.0

    def energy(self, q2, qd2):
        return 0.5 * qd2**2 + self.w2 * (1 - math.cos(q2))

    def reset(self, q1, qd1):
        self.sf = 0.0
        self.low_since = self.t

    def command(self, q1, q2, qd1, qd2):
        self.t += self.Ts
        E = self.energy(q2, qd2)
        self.sf += self.alpha * (qd2 * math.cos(q2) - self.sf)
        if E >= self.kick_E:
            self.low_since = self.t
        if self.t < self.kick_until:
            cmd = self.kick_amp
        elif self.t - self.low_since >= self.kick_rearm:      # at rest long enough: start a kick
            self.kick_until = self.t + self.kick_T
            self.low_since = self.t + self.kick_T
            cmd = self.kick_amp
        else:
            cmd = self.c * max(0.0, 1 - E / self.E_up) * self.sf     # pump only below E_up; above it, friction removes the excess
        cmd = max(-self.A_max, min(self.A_max, cmd))
        return cmd, None, E
