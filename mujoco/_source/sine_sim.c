// Drive the RZ42 MuJoCo model with a sine arm command, log the motion, optionally render an MP4.
// usage: sine_sim model.xml A_rad f_Hz T_s out.csv [out.mp4] [ramp_s=1] [theta0_deg=0]
// arm command = A*sin(2*pi*f*t), amplitude faded in over ramp_s with 0.5*(1-cos(pi*t/ramp_s)).
// A = 0 with theta0 != 0 gives a free-swing (damping) test.
#include <mujoco/mujoco.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// minimal EGL declarations (headers are not installed; libEGL is)
typedef void* EGLDisplay; typedef void* EGLConfig; typedef void* EGLSurface; typedef void* EGLContext;
typedef int EGLint; typedef unsigned int EGLBoolean; typedef unsigned int EGLenum;
extern EGLDisplay eglGetDisplay(void*);
extern EGLBoolean eglInitialize(EGLDisplay, EGLint*, EGLint*);
extern EGLBoolean eglChooseConfig(EGLDisplay, const EGLint*, EGLConfig*, EGLint, EGLint*);
extern EGLSurface eglCreatePbufferSurface(EGLDisplay, EGLConfig, const EGLint*);
extern EGLBoolean eglBindAPI(EGLenum);
extern EGLContext eglCreateContext(EGLDisplay, EGLConfig, EGLContext, const EGLint*);
extern EGLBoolean eglMakeCurrent(EGLDisplay, EGLSurface, EGLSurface, EGLContext);
extern EGLint eglGetError(void);

static int initGL(int W, int H) {
    EGLDisplay dpy = eglGetDisplay((void*)0);
    EGLint maj, min;
    if (!eglInitialize(dpy, &maj, &min)) { fprintf(stderr, "eglInitialize failed 0x%x\n", eglGetError()); return 0; }
    const EGLint cfg[] = {0x3033, 0x0001, 0x3040, 0x0008, 0x3024, 8, 0x3023, 8, 0x3022, 8, 0x3021, 8, 0x3025, 24, 0x3038};
    EGLConfig conf; EGLint n;
    if (!eglChooseConfig(dpy, cfg, &conf, 1, &n) || n < 1) { fprintf(stderr, "eglChooseConfig failed\n"); return 0; }
    const EGLint pb[] = {0x3057, W, 0x3056, H, 0x3038};
    EGLSurface surf = eglCreatePbufferSurface(dpy, conf, pb);
    eglBindAPI(0x30A2);
    EGLContext ctx = eglCreateContext(dpy, conf, (void*)0, NULL);
    if (!ctx || !eglMakeCurrent(dpy, surf, surf, ctx)) { fprintf(stderr, "EGL context failed 0x%x\n", eglGetError()); return 0; }
    return 1;
}

int main(int argc, char** argv) {
    if (argc < 6) { fprintf(stderr, "usage: %s model.xml A_rad f_Hz T_s out.csv [out.mp4]\n", argv[0]); return 1; }
    char err[1000];
    mjModel* m = mj_loadXML(argv[1], NULL, err, sizeof(err));
    if (!m) { fprintf(stderr, "load error: %s\n", err); return 1; }
    mjData* d = mj_makeData(m);
    double A = atof(argv[2]), f = atof(argv[3]), T = atof(argv[4]);
    const char* mp4 = (argc > 6 && strcmp(argv[6], "-") != 0) ? argv[6] : NULL;
    double ramp = argc > 7 ? atof(argv[7]) : 1.0, th0 = argc > 8 ? atof(argv[8]) * M_PI / 180 : 0.0;
    int ja = mj_name2id(m, mjOBJ_JOINT, "arm_joint"), jp = mj_name2id(m, mjOBJ_JOINT, "pendulum_joint");
    int qa = m->jnt_qposadr[ja], qp = m->jnt_qposadr[jp], va = m->jnt_dofadr[ja], vp = m->jnt_dofadr[jp];
    d->qpos[qp] = th0; mj_forward(m, d);

    const int W = 1280, H = 720, FPS = 30;
    FILE* ff = NULL; unsigned char* rgb = NULL; unsigned char* row = NULL;
    mjvScene scn; mjrContext con; mjvCamera cam; mjvOption opt;
    if (mp4) {
        if (!initGL(W, H)) return 2;
        mjv_defaultScene(&scn); mjr_defaultContext(&con); mjv_defaultCamera(&cam); mjv_defaultOption(&opt);
        mjv_makeScene(m, &scn, 2000); mjr_makeContext(m, &con, mjFONTSCALE_150);
        mjr_setBuffer(mjFB_OFFSCREEN, &con);
        cam.type = mjCAMERA_FREE; cam.lookat[0] = 0.0; cam.lookat[1] = -0.045; cam.lookat[2] = 0.085;
        cam.distance = 0.36; cam.azimuth = 62; cam.elevation = -12;
        char cmd[1024];
        snprintf(cmd, sizeof cmd, "ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgb24 -s %dx%d -r %d -i - "
                 "-c:v libx264 -preset veryfast -pix_fmt yuv420p -crf 22 \"%s\"", W, H, FPS, mp4);
        ff = popen(cmd, "w");
        rgb = (unsigned char*)malloc(3 * W * H); row = (unsigned char*)malloc(3 * W);
    }
    FILE* csv = fopen(argv[5], "w");
    fprintf(csv, "t,arm_cmd,arm,arm_vel,pendulum,pendulum_vel,motor_torque\n");
    int nsteps = (int)(T / m->opt.timestep + 0.5), renderEvery = (int)(1.0 / (FPS * m->opt.timestep) + 0.5);
    double maxPen = 0, maxTorque = 0, maxTrack = 0;
    for (int i = 0; i <= nsteps; i++) {
        double t = d->time, env = (ramp > 0 && t < ramp) ? 0.5 * (1 - cos(M_PI * t / ramp)) : 1.0;
        double cmd = A * env * sin(2 * M_PI * f * t);
        d->ctrl[0] = cmd;
        if (i % 5 == 0)
            fprintf(csv, "%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", t, cmd, d->qpos[qa], d->qvel[va], d->qpos[qp], d->qvel[vp], d->actuator_force[0]);
        if (fabs(d->qpos[qp]) > maxPen) maxPen = fabs(d->qpos[qp]);
        if (t > 0.2 && fabs(d->actuator_force[0]) > maxTorque) maxTorque = fabs(d->actuator_force[0]);
        if (t > 0.5 && fabs(cmd - d->qpos[qa]) > maxTrack) maxTrack = fabs(cmd - d->qpos[qa]);
        if (ff && i % renderEvery == 0) {
            mjrRect vp = {0, 0, W, H};
            mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
            mjr_render(vp, &scn, &con);
            char t1[200], t2[200];
            snprintf(t1, sizeof t1, "time\narm command\narm angle\npendulum angle\nmotor torque");
            snprintf(t2, sizeof t2, "%.2f s\n%+.1f deg\n%+.1f deg\n%+.1f deg\n%+.3f N*m", t, cmd * 180 / M_PI,
                     d->qpos[qa] * 180 / M_PI, d->qpos[qp] * 180 / M_PI, d->actuator_force[0]);
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, vp, t1, t2, &con);
            char t3[200]; snprintf(t3, sizeof t3, "input: %.2f rad x sin(2 pi %.2f Hz t), %.1f s fade-in", A, f, ramp);
            mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, vp, t3, NULL, &con);
            mjr_readPixels(rgb, NULL, vp, &con);
            for (int y = 0; y < H / 2; y++) {   // flip vertically
                memcpy(row, rgb + 3 * W * y, 3 * W);
                memcpy(rgb + 3 * W * y, rgb + 3 * W * (H - 1 - y), 3 * W);
                memcpy(rgb + 3 * W * (H - 1 - y), row, 3 * W);
            }
            fwrite(rgb, 1, 3 * W * H, ff);
        }
        mj_step(m, d);
    }
    fclose(csv);
    if (ff) { pclose(ff); mjr_freeContext(&con); mjv_freeScene(&scn); }
    printf("A=%.3f rad f=%.3f Hz T=%.1f s: max |pendulum| %.1f deg, max |motor torque| (t>0.2 s) %.3f N*m, max tracking error %.2f deg\n",
           A, f, T, maxPen * 180 / M_PI, maxTorque, maxTrack * 180 / M_PI);
    mj_deleteData(d); mj_deleteModel(m);
    return 0;
}
