// Render a logged closed-loop run (control/run_demo.py CSV) to MP4 with MuJoCo's offscreen renderer.
// Replay for visualisation only: joint angles come from the log; no physics is stepped here.
// usage: render_replay model.xml log.csv out.mp4 [fps=30]
// build: gcc -O2 -o render_replay render_replay.c -I<mujoco>/include -L<mujoco>/lib -lmujoco -lEGL -lm -Wl,-rpath,<mujoco>/lib
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
    if (argc < 4) { fprintf(stderr, "usage: %s model.xml log.csv out.mp4 [fps]\n", argv[0]); return 1; }
    char err[1000];
    mjModel* m = mj_loadXML(argv[1], NULL, err, sizeof(err));
    if (!m) { fprintf(stderr, "%s\n", err); return 1; }
    mjData* d = mj_makeData(m);
    int FPS = argc > 4 ? atoi(argv[4]) : 30, W = 1280, H = 720;
    int qa = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "arm_joint")];
    int qp = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "pendulum_joint")];
    int gp = mj_name2id(m, mjOBJ_GEOM, "pendulum");
    // read log: t,q1,q2,qd1,qd2,q1_meas,q2_meas,q1_cmd,motor_torque_diag,mode_lqr,energy,delta,sat,dist
    FILE* f = fopen(argv[2], "r");
    char line[2048]; fgets(line, sizeof line, f);
    int cap = 1 << 16, n = 0;
    double (*L)[14] = malloc(sizeof(double[14]) * cap);
    while (fgets(line, sizeof line, f)) {
        if (n == cap) { cap *= 2; L = realloc(L, sizeof(double[14]) * cap); }
        char* p = line;
        for (int j = 0; j < 14; j++) { L[n][j] = strtod(p, &p); if (*p == ',') p++; }
        n++;
    }
    fclose(f);
    if (!initGL(W, H)) return 2;
    mjvScene scn; mjrContext con; mjvCamera cam; mjvOption opt;
    mjv_defaultScene(&scn); mjr_defaultContext(&con); mjv_defaultCamera(&cam); mjv_defaultOption(&opt);
    mjv_makeScene(m, &scn, 2000); mjr_makeContext(m, &con, mjFONTSCALE_150);
    mjr_setBuffer(mjFB_OFFSCREEN, &con);
    cam.type = mjCAMERA_FREE; cam.lookat[0] = 0.0; cam.lookat[1] = -0.03; cam.lookat[2] = 0.09;
    cam.distance = 0.42; cam.azimuth = 60; cam.elevation = -15;
    char cmd[1024];
    snprintf(cmd, sizeof cmd, "ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgb24 -s %dx%d -r %d -i - "
             "-c:v libx264 -preset veryfast -pix_fmt yuv420p -crf 24 \"%s\"", W, H, FPS, argv[3]);
    FILE* ff = popen(cmd, "w");
    unsigned char* rgb = malloc(3 * W * H); unsigned char* row = malloc(3 * W);
    double T = L[n - 1][0];
    int k = 0;
    for (int fr = 0; fr * (1.0 / FPS) <= T; fr++) {
        double t = fr * (1.0 / FPS);
        while (k < n - 1 && L[k + 1][0] <= t) k++;
        d->qpos[qa] = L[k][1]; d->qpos[qp] = L[k][2];
        mj_forward(m, d);
        int lqr = L[k][9] > 0.5;
        float c[4] = {lqr ? 0.20f : 0.95f, lqr ? 0.75f : 0.55f, lqr ? 0.35f : 0.15f, 1.0f};
        memcpy(m->geom_rgba + 4 * gp, c, sizeof c);
        mjrRect vp = {0, 0, W, H};
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(vp, &scn, &con);
        double dl = fabs(L[k][11]) * 180 / M_PI;
        char t1[300], t2[300];
        snprintf(t1, sizeof t1, "mode\ntime\nfrom upright\narm angle\narm command (q1_cmd)");
        snprintf(t2, sizeof t2, "%s\n%.2f s\n%.1f deg\n%+.1f deg\n%+.1f deg", lqr ? "LQR_BALANCE" : "SWING_UP", t, dl,
                 L[k][1] * 180 / M_PI, L[k][7] * 180 / M_PI);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, vp, t1, t2, &con);
        const char* note = "";
        for (int j = k; j < n && L[j][0] < t + 0.6; j++) if (L[j][13] != 0) { note = L[j][13] > 0.01 ? "HARD PUSH" : "TAP"; break; }
        for (int j = k; j >= 0 && L[j][0] > t - 0.8; j--) if (L[j][13] != 0) { note = L[j][13] > 0.01 ? "HARD PUSH" : "TAP"; break; }
        char t3[300];
        snprintf(t3, sizeof t3, "position-command swing-up + LQR on the hidden MuJoCo plant (replay of the logged run)  %s", note);
        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, vp, t3, NULL, &con);
        mjr_readPixels(rgb, NULL, vp, &con);
        for (int y = 0; y < H / 2; y++) {
            memcpy(row, rgb + 3 * W * y, 3 * W);
            memcpy(rgb + 3 * W * y, rgb + 3 * W * (H - 1 - y), 3 * W);
            memcpy(rgb + 3 * W * (H - 1 - y), row, 3 * W);
        }
        fwrite(rgb, 1, 3 * W * H, ff);
    }
    pclose(ff);
    printf("wrote %s (%d frames)\n", argv[3], (int)(T * FPS));
    return 0;
}
