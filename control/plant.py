"""MuJoCo plant: the full nonlinear mechanics of the rig (layer 1 of 3).

The plant knows the true masses, inertias and friction, because it simulates the physical rig.
It exposes only what physics needs: a motor-torque input (set by the stepper driver model in
servo.py, never by the external controller), an external disturbance torque on the pendulum,
and the true joint state, which goes to the sensor model and the driver's own encoder.

Two backends, same interface:
  MujocoPlant  - the official `mujoco` Python package (pip install mujoco). Used by default and
                 by the live viewer.
  ShimPlant    - tools/libmjshim.so, a tiny C shim over the MuJoCo C library. Only for running the
                 tests where the Python package cannot be installed (Linux).
"""
import ctypes
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
PLANTS = {
    "real": ROOT / "mujoco" / "rz42_real_plant.xml",                 # hidden "real" rig
    "real_mass_d1": ROOT / "mujoco" / "rz42_real_plant_mass_d1.xml",  # same rig + 10 g at 25 mm (model-mismatch test)
}


class MujocoPlant:
    def __init__(self, xml):
        import mujoco
        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(str(xml))
        self.d = mujoco.MjData(self.m)
        ja, jp = self.m.joint("arm_joint"), self.m.joint("pendulum_joint")
        self.qa, self.qp = int(ja.qposadr[0]), int(jp.qposadr[0])
        self.va, self.vp = int(ja.dofadr[0]), int(jp.dofadr[0])
        self.b_arm = self.m.body("arm").id
        self.b_pen = self.m.body("pendulum").id
        self.dt = float(self.m.opt.timestep)

    def reset(self, q1=0.0, q2=0.0, qd1=0.0, qd2=0.0):
        self.mj.mj_resetData(self.m, self.d)
        self.d.qpos[self.qa], self.d.qpos[self.qp] = q1, q2
        self.d.qvel[self.va], self.d.qvel[self.vp] = qd1, qd2
        self.mj.mj_forward(self.m, self.d)

    def set_pendulum_torque(self, tau):
        axis = self.d.xmat[self.b_arm].reshape(3, 3)[:, 0]          # hinge axis = arm x axis (world)
        self.d.xfrc_applied[self.b_pen, :] = 0.0
        self.d.xfrc_applied[self.b_pen, 3:6] = tau * axis

    def step(self, motor_torque):
        self.d.ctrl[0] = motor_torque
        self.mj.mj_step(self.m, self.d)

    def state(self):
        d = self
        return (float(d.d.time), float(d.d.qpos[d.qa]), float(d.d.qpos[d.qp]),
                float(d.d.qvel[d.va]), float(d.d.qvel[d.vp]))


class ShimPlant:
    def __init__(self, xml, lib=ROOT / "tools" / "libmjshim.so"):
        self.lib = ctypes.CDLL(str(lib))
        L = self.lib
        L.plant_load.argtypes = [ctypes.c_char_p]
        L.plant_timestep.restype = ctypes.c_double
        L.plant_reset.argtypes = [ctypes.c_double] * 4
        L.plant_set_pendulum_torque.argtypes = [ctypes.c_double]
        L.plant_step.argtypes = [ctypes.c_double]
        L.plant_state.argtypes = [ctypes.POINTER(ctypes.c_double)]
        if not L.plant_load(str(xml).encode()):
            raise RuntimeError(f"could not load {xml}")
        self.dt = L.plant_timestep()
        self._buf = (ctypes.c_double * 5)()
        self.m = None

    def reset(self, q1=0.0, q2=0.0, qd1=0.0, qd2=0.0):
        self.lib.plant_reset(q1, q2, qd1, qd2)

    def set_pendulum_torque(self, tau):
        self.lib.plant_set_pendulum_torque(tau)

    def step(self, motor_torque):
        self.lib.plant_step(motor_torque)

    def state(self):
        self.lib.plant_state(self._buf)
        return tuple(self._buf)


def make_plant(name="real", backend="auto"):
    xml = PLANTS.get(name, name)
    if backend in ("auto", "mujoco"):
        try:
            return MujocoPlant(xml)
        except ImportError:
            if backend == "mujoco":
                raise
    return ShimPlant(xml)
