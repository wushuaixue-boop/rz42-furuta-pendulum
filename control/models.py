"""Controller-side models, built ONLY from the identified parameters (identification/identified_parameters.json).

Pendulum model identified from encoder data (q2 = 0 hanging, q2 = pi upright):
    q2'' = -kappa cos(q2) q1'' + rho sin(q2)cos(q2) q1'^2 - omega2 sin(q2) - beta q2' - phi tanh(q2'/v_s)
Assumed stepper position servo (placeholders until identified from arm_cmd vs q1_meas logs):
    q1'' = ws^2 (q1_cmd - q1) - 2 zeta ws q1'
Linearised about upright, x = [q1, q2 - pi, q1', q2'], u = q1_cmd:
    x' = A_pos x + B_pos u
"""
import json
import math
from pathlib import Path

import numpy as np
from scipy.linalg import expm

ROOT = Path(__file__).resolve().parent.parent


def load_identified(path=None):
    path = Path(path) if path else ROOT / "identification" / "identified_parameters.json"
    if "ground_truth" in path.name:
        raise ValueError("the controller must not be designed from the hidden ground truth")
    d = json.loads(path.read_text())
    return dict(d["ratios"]), d.get("v_s", 0.01)


def servo_params(cfg):
    return 2 * math.pi * cfg["omega_s_hz"], cfg["zeta_s"]


def position_model(ratios, ws, zeta):
    """continuous A_pos, B_pos about upright"""
    k, w2, b = ratios["kappa"], ratios["omega2"], ratios["beta"]
    A = np.array([[0, 0, 1, 0],
                  [0, 0, 0, 1],
                  [-ws**2, 0, -2 * zeta * ws, 0],
                  [-k * ws**2, w2, -2 * k * zeta * ws, -b]])
    # row 4: delta'' = omega2 delta - beta delta' + kappa q1''  (cos(pi) = -1 flips the kappa sign)
    B = np.array([[0.0], [0.0], [ws**2], [k * ws**2]])
    return A, B


def c2d(A, B, Ts):
    """zero-order-hold discretisation"""
    n, m = A.shape[0], B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n], M[:n, n:] = A, B
    E = expm(M * Ts)
    return E[:n, :n], E[:n, n:]


def wrap(a):
    """angle to (-pi, pi]"""
    return (a + math.pi) % (2 * math.pi) - math.pi
