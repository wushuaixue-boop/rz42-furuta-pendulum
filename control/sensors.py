"""Sensors (plant side) and state estimator (controller side).

EncoderSensors turns the true joint angles into what the hardware reports: a 14-bit stepper encoder
for the arm and a 12-bit AS5600 for the pendulum, with a little noise. The AS5600 zero offset is
assumed calibrated (see identification/EXPERIMENT_PROTOCOL.md, step 1). No velocities are measured.

StateEstimator is part of the external controller. It only receives the two measured angles at the
control rate and estimates the velocities with a filtered finite difference ("dirty derivative").
"""
import math

import numpy as np


class EncoderSensors:
    def __init__(self, cfg, seed=0):
        self.lsb1 = 2 * math.pi / 2 ** cfg["arm_bits"]
        self.lsb2 = 2 * math.pi / 2 ** cfg["pend_bits"]
        self.n1 = cfg["arm_noise_counts"] * self.lsb1
        self.n2 = cfg["pend_noise_counts"] * self.lsb2
        self.rng = np.random.default_rng(seed)

    def measure(self, q1, q2):
        q1m = round((q1 + self.rng.normal(0, self.n1)) / self.lsb1) * self.lsb1
        q2m = round((q2 + self.rng.normal(0, self.n2)) / self.lsb2) * self.lsb2
        return q1m, q2m


class StateEstimator:
    def __init__(self, Ts, cutoff_hz):
        self.Ts = Ts
        self.alpha = 1.0 - math.exp(-2 * math.pi * cutoff_hz * Ts)
        self.prev = None
        self.v = [0.0, 0.0]

    def reset(self):
        self.prev = None
        self.v = [0.0, 0.0]

    def update(self, q1m, q2m):
        if self.prev is not None:
            for i, q in enumerate((q1m, q2m)):
                raw = (q - self.prev[i]) / self.Ts
                self.v[i] += self.alpha * (raw - self.v[i])
        self.prev = (q1m, q2m)
        return q1m, q2m, self.v[0], self.v[1]
