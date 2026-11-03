"""Plots of a closed-loop log (results/*.csv written by control.run_demo).

  python -m control.plots results/demo_log.csv            # -> results/demo_log.png and *_zoom.png
"""
import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

C_ARM, C_CMD, C_PEN, C_TAU, C_E = "#2a6fdb", "#e09f3e", "#d1495b", "#555555", "#3a9d5d"


def load(path):
    raw = np.genfromtxt(path, delimiter=",", names=True)
    return {k: raw[k] for k in raw.dtype.names}


def shade_modes(ax, t, lqr):
    edges = np.flatnonzero(np.diff(lqr.astype(int))) + 1
    starts = np.r_[0, edges]
    ends = np.r_[edges, len(t) - 1]
    for s, e in zip(starts, ends):
        if lqr[s]:
            ax.axvspan(t[s], t[e], color="#2a6fdb", alpha=0.08, lw=0)


def plot(log, out, title="", window=None, limits=None):
    t = log["t"]
    sel = np.ones_like(t, bool) if window is None else (t >= window[0]) & (t <= window[1])
    t = t[sel]
    L = {k: v[sel] for k, v in log.items()}
    lqr = L["mode_lqr"] > 0.5
    fig, ax = plt.subplots(5, 1, figsize=(11, 12), sharex=True)
    for a in ax:
        shade_modes(a, t, lqr)
        a.grid(True, alpha=0.3)
        for td in t[np.flatnonzero(np.diff((L["disturbance_torque"] != 0).astype(int)) == 1) + 1]:
            a.axvline(td, color="#999999", ls=":", lw=1)
        for ts in t[np.flatnonzero(np.diff(lqr.astype(int))) + 1]:
            a.axvline(ts, color="#333333", ls="--", lw=0.8)
    ax[0].plot(t, np.degrees(np.abs(L["delta_upright"])), color=C_PEN, lw=1)
    ax[0].set_ylabel("pendulum angle from\nupright |wrap(q2 - π)| [deg]")
    ax[0].set_ylim(-5, 185)
    ax[0].set_yticks([0, 45, 90, 135, 180])
    ax[0].text(0.005, 0.93, "180 = hanging, 0 = upright", transform=ax[0].transAxes, fontsize=8, color="#555555")
    ax[1].plot(t, np.degrees(L["q1_cmd"]), color=C_CMD, lw=1, label="q1_cmd (controller output)")
    ax[1].plot(t, np.degrees(L["q1"]), color=C_ARM, lw=1.1, label="q1 (arm)")
    ax[1].set_ylabel("arm [deg]"); ax[1].legend(loc="upper right", fontsize=8)
    ax[2].plot(t, L["qd1"], color=C_ARM, lw=1, label="q1' arm")
    ax[2].plot(t, L["qd2"], color=C_PEN, lw=1, label="q2' pendulum")
    ax[2].set_ylabel("velocity [rad/s]"); ax[2].legend(loc="upper right", fontsize=8)
    ax[3].plot(t, L["energy"], color=C_E, lw=1)
    if limits and "E_up" in limits:
        ax[3].axhline(limits["E_up"], color=C_E, ls="--", lw=0.8, label="E_up = 2 ω²")
        ax[3].legend(loc="upper right", fontsize=8)
    ax[3].set_ylabel("pendulum energy\nE / J2 [rad²/s²]")
    ax[4].plot(t, 1e3 * L["motor_torque_diag"], color=C_TAU, lw=0.8, label="motor torque (driver, diagnostics only)")
    if limits and "torque_hold" in limits:
        ax[4].axhline(1e3 * limits["torque_hold"], color="#c0392b", ls="--", lw=0.8, label="torque limit (standstill)")
        ax[4].axhline(-1e3 * limits["torque_hold"], color="#c0392b", ls="--", lw=0.8)
    sat = L["torque_saturated"] > 0.5
    if sat.any():
        ax[4].plot(t[sat], 1e3 * L["motor_torque_diag"][sat], ".", color="#c0392b", ms=2, label="saturated")
    ax[4].set_ylabel("torque [mN m]"); ax[4].set_xlabel("time [s]"); ax[4].legend(loc="upper right", fontsize=8)
    ax[0].set_title(title + "\nblue band = LQR_BALANCE, white = SWING_UP, dashed = mode switch, dotted = disturbance")
    plt.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--meta", default=None, help="json written next to the log (for limits)")
    args = ap.parse_args(argv)
    log = load(args.log)
    meta_path = Path(args.meta) if args.meta else Path(args.log).with_suffix(".json")
    limits = {}
    if meta_path.exists():
        meta = json.loads(meta_path.read_text())
        limits = {"E_up": 2 * meta["ratios"]["omega2"], "torque_hold": meta["torque_hold"]}
    out = Path(args.log).with_suffix(".png")
    plot(log, out, title=Path(args.log).stem, limits=limits)
    lqr = log["mode_lqr"] > 0.5
    k = np.flatnonzero(np.diff(lqr.astype(int)) == 1)
    if k.size:
        t0 = log["t"][k[0]]
        plot(log, out.with_name(out.stem + "_catch_zoom.png"), title=Path(args.log).stem + " (first catch)",
             window=(t0 - 1.5, t0 + 2.5), limits=limits)
    on = np.flatnonzero(np.diff((log["disturbance_torque"] != 0).astype(int)) == 1) + 1
    for i, k in enumerate(on):
        t0 = log["t"][k]
        plot(log, out.with_name(out.stem + f"_disturbance{i + 1}_zoom.png"),
             title=Path(args.log).stem + f" (disturbance {i + 1}: {1e3 * log['disturbance_torque'][k]:.0f} mN m for 50 ms)",
             window=(t0 - 0.5, t0 + 4.0), limits=limits)
    print("wrote", out)


if __name__ == "__main__":
    main()
