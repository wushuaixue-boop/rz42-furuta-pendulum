"""Headless closed-loop demonstration: swing-up -> LQR -> >=10 s balance -> tap -> hard push -> recovery.

  python -m control.run_demo                       # writes results/demo_log.csv, .json, .png
  python -m control.run_demo --duration 60 --out results/my_run.csv
  python -m control.run_demo --set servo.omega_s_hz=10 --set scenario.seed=3

The live 3-D view of the same closed loop is control.viewer.
"""
import argparse
import json
from pathlib import Path

from . import plots
from .sim import load_config, metrics, run, save_log

ROOT = Path(__file__).resolve().parent.parent


def parse_sets(items):
    ov = {}
    for s in items or []:
        k, v = s.split("=", 1)
        try:
            v = json.loads(v)
        except json.JSONDecodeError:
            pass
        ov[k] = v
    return ov


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--backend", default="auto", choices=["auto", "mujoco", "shim"])
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--out", default=str(ROOT / "results" / "demo_log.csv"))
    ap.add_argument("--set", action="append", help="override a config value, e.g. --set lqr.R=5")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args(argv)
    ov = parse_sets(args.set)
    if args.duration:
        ov["scenario.duration"] = args.duration
    cfg = load_config(args.config, ov)
    print(f"plant: {cfg['plant']['name']}   duration {cfg['scenario']['duration']} s")
    log, meta = run(cfg, args.backend, progress=True)
    m = metrics(log, meta, cfg)
    out = Path(args.out)
    save_log(log, out)
    meta_out = {"metrics": m, "events": meta["events"], "K": meta["K"], "ratios": meta["ratios"],
                "torque_hold": cfg["servo"]["torque_hold"], "config": cfg}
    out.with_suffix(".json").write_text(json.dumps(meta_out, indent=2, default=str))
    print("\nmode switches:")
    for t, mode, why in meta["events"]:
        print(f"  t = {t:6.2f} s  -> {mode:12s} ({why})")
    print("\nmetrics:")
    for k, v in m.items():
        print(f"  {k:28s} {v:.4g}" if isinstance(v, float) else f"  {k:28s} {v}")
    print(f"\nlog: {out}")
    if not args.no_plot:
        plots.main([str(out)])


if __name__ == "__main__":
    main()
