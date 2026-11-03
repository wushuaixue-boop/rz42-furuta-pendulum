"""Automated validation of the closed loop on the hidden MuJoCo plant.

  python -m control.validate                 # all tests, writes results/validation_summary.{md,json}
  python -m control.validate --quick         # a smaller set

Tests
  servo_*      the ZDT servo model on the real coupled plant vs the ideal 2nd-order approximation
  demo         the full demonstration: swing-up, catch, >=10 s balance, moderate tap, hard push, recovery
  init_*       several initial conditions
  servo_bw_*   actual servo bandwidth different from the 15 Hz the controller assumes
  latency / rate / noise seeds / added-mass plant (model mismatch)
Exit code 1 if any required test fails.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from .plant import make_plant
from .servo import ZDTServo
from .sim import load_config, metrics, run

ROOT = Path(__file__).resolve().parent.parent
RES = ROOT / "results"


def servo_step_test(cfg, backend, step=0.1, T=0.6):
    """command step on the real coupled plant (pendulum hanging) vs. the ideal servo model"""
    plant = make_plant(cfg["plant"]["name"], backend)
    servo = ZDTServo(cfg["servo"])
    plant.reset()
    servo.reset(0.0)
    servo.set_command(step)
    dt, n = plant.dt, int(T / plant.dt)
    q = np.empty(n)
    for k in range(n):
        _, q1, _, qd1, _ = plant.state()
        plant.step(servo.torque(q1, qd1, dt))
        q[k] = q1
    ws, z = servo.ws, servo.zeta
    x, v, qi = 0.0, 0.0, np.empty(n)
    for k in range(n):                            # ideal  q'' = ws^2 (cmd - q) - 2 z ws q'
        a = ws**2 * (step - x) - 2 * z * ws * v
        v += a * dt
        x += v * dt
        qi[k] = x
    t = np.arange(n) * dt

    def char(y):
        i10, i90 = np.argmax(y >= 0.1 * step), np.argmax(y >= 0.9 * step)
        band = np.abs(y - step) > 0.02 * step
        ts = t[np.flatnonzero(band)[-1]] if band.any() else 0.0
        return {"rise_10_90_ms": 1e3 * (t[i90] - t[i10]), "overshoot_pct": 100 * (y.max() - step) / step,
                "settle_2pct_ms": 1e3 * ts}
    return {"actual": char(q), "ideal_model": char(qi),
            "rms_deviation_pct_of_step": 100 * float(np.sqrt(np.mean((q - qi)**2))) / step}


def check(name, cfg, backend, require, expect):
    log, meta = run(cfg, backend)
    m = metrics(log, meta, cfg)
    ok, why = True, []
    for key, (op, val) in expect.items():
        got = m.get(key)
        good = got is not None and ((got <= val) if op == "<=" else (got >= val) if op == ">=" else (got == val))
        if not good:
            ok = False
            why.append(f"{key}={got} (need {op} {val})")
    return {"name": name, "required": require, "pass": ok, "why": why, "metrics": m,
            "events": [(round(e[0], 2), e[1], e[2]) for e in meta["events"]]}


def demo_expect():
    return {"first_catch_s": ("<=", 15.0), "longest_balance_s": (">=", 10.0), "final_mode": ("==", "LQR_BALANCE"),
            "final_delta_deg": ("<=", 5.0), "torque_saturated_fraction": ("<=", 0.01), "max_arm_deg": ("<=", 130.0)}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", default="auto", choices=["auto", "mujoco", "shim"])
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--config", default=None)
    args = ap.parse_args(argv)
    base = load_config(args.config)
    RES.mkdir(exist_ok=True)
    results = []

    print("servo step test (0.1 rad command step, real coupled plant):")
    sv = servo_step_test(base, args.backend)
    print("  ", json.dumps(sv))

    def cfg_with(**ov):
        return load_config(args.config, overrides=ov)

    # 1. the full demonstration, plus a check that the moderate tap did not cause a mode switch
    r = check("demo", base, args.backend, True, demo_expect())
    taps = [e for e in r["events"] if 22.0 <= e[0] < 30.0]
    if taps:
        r["pass"], r["why"] = False, r["why"] + [f"moderate tap caused a mode switch: {taps}"]
    after = [e for e in r["events"] if e[0] >= 30.0 and e[1] == "LQR_BALANCE"]
    r["metrics"]["recovered_after_hard_push"] = bool(after)
    results.append(r)

    quiet = {"scenario.disturbances": [], "scenario.duration": 30.0}
    bal_expect = {"first_catch_s": ("<=", 18.0), "longest_balance_s": (">=", 10.0), "final_mode": ("==", "LQR_BALANCE"),
                  "final_delta_deg": ("<=", 5.0)}
    inits = [("init_hanging_offset_30deg", {"scenario.q2_0": math.radians(30)}),
             ("init_hanging_arm_offset", {"scenario.q1_0": 0.5}),
             ("init_moving_pendulum", {"scenario.q2_0": math.radians(-60), "scenario.qd2_0": 3.0}),
             ("init_near_upright", {"scenario.q2_0": math.pi - 0.15}),
             ("init_near_upright_other_side_wrapped", {"scenario.q2_0": -math.pi + 0.2})]
    if args.quick:
        inits = inits[:2]
    for name, ov in inits:
        results.append(check(name, cfg_with(**quiet, **ov), args.backend, True, bal_expect))

    # 2. actual servo bandwidth differs from the controller's assumption (15 Hz)
    bws = [8.0, 10.0, 25.0] if not args.quick else [10.0]
    for hz in bws:
        results.append(check(f"servo_bw_actual_{hz:.0f}Hz_assumed_15Hz", cfg_with(**{"servo.omega_s_hz": hz}),
                             args.backend, hz >= 10.0, demo_expect()))
    if not args.quick:
        results.append(check("servo_bw_10Hz_matched", cfg_with(**{"servo.omega_s_hz": 10.0,
                             "controller.servo_model.omega_s_hz": 10.0}), args.backend, True, demo_expect()))
        # 3. timing, noise, model mismatch
        results.append(check("latency_3ms", cfg_with(**{"controller.latency_s": 0.003}), args.backend, True, demo_expect()))
        results.append(check("control_rate_100Hz", cfg_with(**{"controller.rate_hz": 100.0}), args.backend, False, demo_expect()))
        for seed in (1, 2):
            results.append(check(f"sensor_noise_seed_{seed}", cfg_with(**{"scenario.seed": seed}), args.backend, True, demo_expect()))
        results.append(check("plant_mismatch_added_mass_10g", cfg_with(**{"plant.name": "real_mass_d1", **quiet}),
                             args.backend, False, bal_expect))

    print(f"\n{'test':45s} {'req':4s} {'result':6s} catch[s] bal[s] rms[deg] arm[deg] tau[Nm]")
    for r in results:
        m = r["metrics"]
        print(f"{r['name']:45s} {'yes' if r['required'] else 'no':4s} {'PASS' if r['pass'] else 'FAIL':6s} "
              f"{m['first_catch_s'] if m['first_catch_s'] is None else round(m['first_catch_s'], 2)!s:>8} "
              f"{m['longest_balance_s']:6.1f} {m.get('balance_rms_delta_deg', float('nan')):8.2f} "
              f"{m['max_arm_deg']:8.1f} {m['peak_motor_torque_Nm']:7.3f}  {'; '.join(r['why'])}")
    required_ok = all(r["pass"] for r in results if r["required"])
    out = {"servo_step_test": sv, "tests": results, "all_required_passed": required_ok}
    (RES / "validation_summary.json").write_text(json.dumps(out, indent=2, default=str))
    lines = ["# Validation summary", "", f"All required tests passed: **{required_ok}**", "",
             "## Servo model on the real coupled plant (0.1 rad step)", "",
             "| | rise 10-90 % [ms] | overshoot [%] | settling 2 % [ms] |", "|---|---|---|---|"]
    for k in ("ideal_model", "actual"):
        c = sv[k]
        lines.append(f"| {k} | {c['rise_10_90_ms']:.1f} | {c['overshoot_pct']:.1f} | {c['settle_2pct_ms']:.0f} |")
    lines += [f"", f"RMS difference actual vs. ideal: {sv['rms_deviation_pct_of_step']:.1f} % of the step.", "",
              "## Closed-loop tests", "",
              "| test | required | result | first catch [s] | longest balance [s] | balance RMS [deg] | max arm [deg] | peak torque [N m] | switches | notes |",
              "|---|---|---|---|---|---|---|---|---|---|"]
    for r in results:
        m = r["metrics"]
        lines.append(f"| {r['name']} | {'yes' if r['required'] else 'no'} | {'PASS' if r['pass'] else 'FAIL'} | "
                     f"{'-' if m['first_catch_s'] is None else round(m['first_catch_s'], 2)} | {m['longest_balance_s']:.1f} | "
                     f"{m.get('balance_rms_delta_deg', float('nan')):.2f} | {m['max_arm_deg']:.1f} | {m['peak_motor_torque_Nm']:.3f} | "
                     f"{m['n_switches']} | {'; '.join(r['why'])} |")
    (RES / "validation_summary.md").write_text("\n".join(lines) + "\n")
    print(f"\nall required tests passed: {required_ok}   -> results/validation_summary.md")
    return 0 if required_ok else 1


if __name__ == "__main__":
    sys.exit(main())
