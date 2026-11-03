// Minimal headless MuJoCo runner used to generate the dataset (Linux, MuJoCo 3.3.7 C API).
// usage: mj_runner model.xml commands.csv q2_0_rad out.csv
//   commands.csv : one arm position command (rad) per line, one line per model timestep
//   out.csv      : t, arm_cmd, q1, q2, qd1, qd2, motor_torque  (true states, every timestep)
// The stepper driver's position loop  T = clip(kp*(cmd - q1) - kv*qd1, +-tmax)  is computed here,
// with kp, kv, tmax read from the model's <custom> numerics, and applied through a torque motor,
// so the logged torque is exactly the torque that acted on the arm during the step.
#include <mujoco/mujoco.h>
#include <stdio.h>
#include <stdlib.h>

static double custom(const mjModel* m, const char* name) {
    int id = mj_name2id(m, mjOBJ_NUMERIC, name);
    if (id < 0) { fprintf(stderr, "missing <numeric name=\"%s\"> in the model\n", name); exit(1); }
    return m->numeric_data[m->numeric_adr[id]];
}

int main(int argc, char** argv) {
    if (argc < 5) { fprintf(stderr, "usage: %s model.xml commands.csv q2_0 out.csv\n", argv[0]); return 1; }
    char err[1000];
    mjModel* m = mj_loadXML(argv[1], NULL, err, sizeof(err));
    if (!m) { fprintf(stderr, "load error: %s\n", err); return 1; }
    mjData* d = mj_makeData(m);
    double kp = custom(m, "servo_kp"), kv = custom(m, "servo_kv"), tmax = custom(m, "servo_tmax");
    int ja = mj_name2id(m, mjOBJ_JOINT, "arm_joint"), jp = mj_name2id(m, mjOBJ_JOINT, "pendulum_joint");
    int qa = m->jnt_qposadr[ja], qp = m->jnt_qposadr[jp], va = m->jnt_dofadr[ja], vp = m->jnt_dofadr[jp];
    d->qpos[qp] = atof(argv[3]);
    mj_forward(m, d);
    FILE* fc = fopen(argv[2], "r");
    FILE* fo = fopen(argv[4], "w");
    if (!fc || !fo) { fprintf(stderr, "file error\n"); return 1; }
    fprintf(fo, "t,arm_cmd,q1,q2,qd1,qd2,motor_torque\n");
    double cmd;
    while (fscanf(fc, "%lf", &cmd) == 1) {
        double tau = kp * (cmd - d->qpos[qa]) - kv * d->qvel[va];
        if (tau > tmax) tau = tmax;
        if (tau < -tmax) tau = -tmax;
        d->ctrl[0] = tau;
        fprintf(fo, "%.5f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n", d->time, cmd, d->qpos[qa], d->qpos[qp],
                d->qvel[va], d->qvel[vp], tau);
        mj_step(m, d);
    }
    fclose(fc); fclose(fo);
    mj_deleteData(d); mj_deleteModel(m);
    return 0;
}
