// Tiny C shim around the MuJoCo C API, used ONLY to test control/ without the mujoco Python
// package (e.g. on a machine where pip cannot reach PyPI). control/plant.py uses the official
// `mujoco` Python package when it is installed, and falls back to this library otherwise.
// Build (Linux): gcc -O2 -shared -fPIC -o libmjshim.so mjshim.c -I<mujoco>/include -L<mujoco>/lib -lmujoco -Wl,-rpath,<mujoco>/lib
#include <mujoco/mujoco.h>
#include <string.h>
#include <stdio.h>

typedef struct { mjModel* m; mjData* d; int qa, qp, va, vp, bp, ba; } Plant;
static Plant P;

int plant_load(const char* xml) {
    char err[1000];
    P.m = mj_loadXML(xml, NULL, err, sizeof(err));
    if (!P.m) { fprintf(stderr, "mjshim load error: %s\n", err); return 0; }
    P.d = mj_makeData(P.m);
    int ja = mj_name2id(P.m, mjOBJ_JOINT, "arm_joint"), jp = mj_name2id(P.m, mjOBJ_JOINT, "pendulum_joint");
    P.qa = P.m->jnt_qposadr[ja]; P.qp = P.m->jnt_qposadr[jp];
    P.va = P.m->jnt_dofadr[ja];  P.vp = P.m->jnt_dofadr[jp];
    P.bp = mj_name2id(P.m, mjOBJ_BODY, "pendulum");
    P.ba = mj_name2id(P.m, mjOBJ_BODY, "arm");
    return 1;
}
double plant_timestep(void) { return P.m->opt.timestep; }
void plant_reset(double q1, double q2, double qd1, double qd2) {
    mj_resetData(P.m, P.d);
    P.d->qpos[P.qa] = q1; P.d->qpos[P.qp] = q2; P.d->qvel[P.va] = qd1; P.d->qvel[P.vp] = qd2;
    mj_forward(P.m, P.d);
}
// external torque on the pendulum about its hinge axis (world frame), e.g. a finger tap
void plant_set_pendulum_torque(double tau) {
    double* R = P.d->xmat + 9 * P.ba;            // arm orientation; hinge axis = arm x axis
    double ax[3] = {R[0], R[3], R[6]};
    double* f = P.d->xfrc_applied + 6 * P.bp;
    memset(f, 0, 6 * sizeof(double));
    f[3] = tau * ax[0]; f[4] = tau * ax[1]; f[5] = tau * ax[2];
}
void plant_step(double motor_torque) { P.d->ctrl[0] = motor_torque; mj_step(P.m, P.d); }
void plant_state(double* out) {      // t, q1, q2, qd1, qd2
    out[0] = P.d->time; out[1] = P.d->qpos[P.qa]; out[2] = P.d->qpos[P.qp];
    out[3] = P.d->qvel[P.va]; out[4] = P.d->qvel[P.vp];
}
