"""Discrete LQR about the upright equilibrium with the arm position command as the input.

    q1_cmd = q1_eq - K (x - x_eq),   x = [q1, q2, q1', q2'],  x_eq = [q1_eq, pi, 0, 0]

The pendulum error uses the wrapped angle wrap(q2 - pi), so any multiple of 2 pi counts as upright.
"""
import numpy as np
from scipy.linalg import solve_discrete_are

from .models import c2d, position_model, wrap


def dlqr(Ad, Bd, Q, R):
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    return K, P


class LQRBalance:
    def __init__(self, ratios, ws, zeta, Ts, Q, R):
        self.A, self.B = position_model(ratios, ws, zeta)
        self.Ad, self.Bd = c2d(self.A, self.B, Ts)
        self.Q, self.R = np.diag(Q), np.atleast_2d(R)
        self.K, _ = dlqr(self.Ad, self.Bd, self.Q, self.R)
        self.K = self.K.ravel()
        self.cl_eig = np.linalg.eigvals(self.Ad - self.Bd @ self.K[None, :])

    def command(self, q1, q2, qd1, qd2, q1_eq):
        e = np.array([q1 - q1_eq, wrap(q2 - np.pi), qd1, qd2])
        return q1_eq - float(self.K @ e)
