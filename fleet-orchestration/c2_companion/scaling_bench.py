"""Layer D — Asymptotic Scaling Study (5 to 500 Aircraft) & Component Profiler.

Empirically answers the hackathon PDF research question:
  "Scaling study: how far can your planner go before it breaks (30 to 100+ aircraft)?"

Features:
  - Physically consistent scenario scaling extending `fleetkit.generator` from N=5 to N=500
    (without mutating `fleetkit` or `handin`).
  - Non-invasive stage-level profiling of certified V10 (`FleetSolver` subclass).
  - Side-by-side comparison against official `greedy_plan` and `fleetkit.scorer.evaluate`.
  - Log-log power-law exponent fitting T(N) ~ c * N^alpha per subsystem.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import json
import math
import random
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Tuple

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "fleet-kit"))
sys.path.insert(0, str(ROOT / "fleet-kit" / "handin"))

from fleetkit.baseline import greedy_plan  # noqa: E402
from fleetkit.generator import LEVELS, _generate_once, generate  # noqa: E402
from fleetkit.model import Scenario, save_json  # noqa: E402
from fleetkit.scorer import evaluate  # noqa: E402
from solver import FleetSolver  # noqa: E402

OUT_DIR = Path("/home/nisseya/.gemini/antigravity-acp/brain/d6074ffc-cefc-488c-9fb3-57a9bcb06b28/scratch/c2_exp_plans")


def make_scaled_level_spec(n_aircraft: int, regime: str = "fixed_theater") -> Tuple[str, dict]:
    """Builds a physically consistent generator level spec for arbitrary fleet size N."""
    official_map = {5: "L1", 12: "L2", 30: "L3", 100: "L4"}
    if n_aircraft in official_map and regime == "fixed_theater":
        lvl = official_map[n_aircraft]
        return lvl, dict(LEVELS[lvl])

    scouts = max(2, round(0.15 * n_aircraft))
    light = max(2, round(0.50 * n_aircraft))
    heavy = max(1, n_aircraft - scouts - light)
    hostile = max(3, round(0.50 * n_aircraft))
    unknown = max(1, round(0.20 * n_aircraft))
    friendly = max(1, round(0.12 * n_aircraft))
    civilian = max(1, round(0.08 * n_aircraft))

    scale = n_aircraft / 100.0
    if regime == "fixed_theater":
        area = 7000.0 if n_aircraft >= 60 else 4500.0 + 2500.0 * (n_aircraft - 30) / 30.0
        zones = 8 if n_aircraft >= 100 else max(4, round(8 * scale ** 0.5))
    elif regime == "scaled_area":
        area = min(9000.0, 7000.0 * (max(scale, 0.3) ** 0.25))
        zones = max(4, round(8 * (max(scale, 0.3) ** 0.5)))
    else:
        raise ValueError(f"Unknown regime {regime!r}")

    c_lo = max(2, round(4 * max(scale, 0.25) ** 0.5))
    c_hi = max(c_lo + 1, round(6 * max(scale, 0.25) ** 0.5))
    spec = dict(
        scouts=scouts,
        light=light,
        heavy=heavy,
        hostile=hostile,
        unknown=unknown,
        friendly=friendly,
        civilian=civilian,
        zones=zones,
        area=round(area, 1),
        horizon=900.0,
        clusters=(c_lo, c_hi),
        sigma=0.07,
        window=(0.2, 0.4),
    )
    return f"S{n_aircraft}", spec


def generate_scaled_scenario(n_aircraft: int, seed: int, regime: str = "fixed_theater") -> dict:
    """Generates a deterministic scenario of size `n_aircraft` without mutating official LEVELS."""
    lvl_name, spec = make_scaled_level_spec(n_aircraft, regime=regime)
    if lvl_name in LEVELS and regime == "fixed_theater":
        return generate(lvl_name, seed)
    # Temporarily register synthetic level key, generate once, and clean up
    LEVELS[lvl_name] = spec
    try:
        rng = random.Random(f"fleetkit-scale-{regime}-{n_aircraft}-{seed}")
        data = _generate_once(lvl_name, seed, rng)
    finally:
        LEVELS.pop(lvl_name, None)
    return data


class InstrumentedFleetSolver(FleetSolver):
    """Zero-behavior-change profiling wrapper around certified V10 FleetSolver."""

    def __init__(self, scenario: Scenario, use_direct_z0: bool = True):
        self.prof: Dict[str, Any] = {
            "t_init": 0.0,
            "t_knapsack": 0.0, "n_knapsack": 0,
            "t_eval_subset": 0.0, "n_eval_subset": 0,
            "t_swap_ls": 0.0, "n_swap_ls": 0,
            "t_liberation": 0.0, "n_liberation": 0,
            "t_deconflict": 0.0, "n_deconflict": 0,
            "t_fallback": 0.0, "n_fallback": 0,
        }
        t0 = time.perf_counter()
        super().__init__(scenario, use_direct_z0=use_direct_z0)
        self.prof["t_init"] = time.perf_counter() - t0

    def _knapsack_candidates(self, *args, **kwargs):
        t0 = time.perf_counter()
        out = super()._knapsack_candidates(*args, **kwargs)
        self.prof["t_knapsack"] += time.perf_counter() - t0
        self.prof["n_knapsack"] += 1
        return out

    def evaluate_subset(self, *args, **kwargs):
        t0 = time.perf_counter()
        out = super().evaluate_subset(*args, **kwargs)
        self.prof["t_eval_subset"] += time.perf_counter() - t0
        self.prof["n_eval_subset"] += 1
        return out

    def _hostile_swap_local_search(self, *args, **kwargs):
        t0 = time.perf_counter()
        out = super()._hostile_swap_local_search(*args, **kwargs)
        self.prof["t_swap_ls"] += time.perf_counter() - t0
        self.prof["n_swap_ls"] += 1
        return out

    def _super_scout_liberation(self, *args, **kwargs):
        t0 = time.perf_counter()
        out = super()._super_scout_liberation(*args, **kwargs)
        self.prof["t_liberation"] += time.perf_counter() - t0
        self.prof["n_liberation"] += 1
        return out

    def deconflict_and_build_plan(self, *args, **kwargs):
        t0 = time.perf_counter()
        out = super().deconflict_and_build_plan(*args, **kwargs)
        self.prof["t_deconflict"] += time.perf_counter() - t0
        self.prof["n_deconflict"] += 1
        return out

    def _run_fallback_trio(self, *args, **kwargs):
        t0 = time.perf_counter()
        out = super()._run_fallback_trio(*args, **kwargs)
        self.prof["t_fallback"] += time.perf_counter() - t0
        self.prof["n_fallback"] += 1
        return out


def run_single_scale_case(args: Tuple[int, int, str, float, bool]) -> Dict[str, Any]:
    """Runs one (N, seed, regime) case: V10 instrumented + validator + optional greedy baseline."""
    n_ac, seed, regime, time_budget_s, run_baseline = args
    sc_dict = generate_scaled_scenario(n_ac, seed, regime=regime)
    sc = Scenario.from_dict(sc_dict)
    n_contacts = len(sc.contacts)
    n_hostile = sum(1 for c in sc.contacts if c.cls == "hostile")

    t0 = time.perf_counter()
    solver = InstrumentedFleetSolver(sc, use_direct_z0=True)
    plan = solver.solve(time_budget_s=time_budget_s)
    t_total = time.perf_counter() - t0

    tv0 = time.perf_counter()
    res = evaluate(sc, plan)
    t_val = time.perf_counter() - tv0

    base_score = None
    base_valid = None
    t_base = None
    if run_baseline:
        tb0 = time.perf_counter()
        bplan = greedy_plan(sc)
        t_base = time.perf_counter() - tb0
        bres = evaluate(sc, bplan)
        base_score = round(bres.score, 2)
        base_valid = bres.valid

    return {
        "n_aircraft": n_ac,
        "seed": seed,
        "regime": regime,
        "n_contacts": n_contacts,
        "n_hostile": n_hostile,
        "n_zones": len(sc.zones),
        "area_m": round(max(c.pos[0] for c in sc.contacts) + 300.0, 0),
        "valid": res.valid,
        "score": round(res.score, 2),
        "gross_points": round(res.points, 1),
        "served_count": len(res.served),
        "active_routes": len(plan.get("routes", [])),
        "within_60s": t_total <= 60.0,
        "t_total_s": round(t_total, 3),
        "t_validate_s": round(t_val, 4),
        "prof": {k: (round(v, 4) if isinstance(v, float) else v) for k, v in solver.prof.items()},
        "base_score": base_score,
        "base_valid": base_valid,
        "t_base_s": None if t_base is None else round(t_base, 3),
        "ratio_vs_base": round(res.score / base_score, 3) if (base_score and base_score > 0) else None,
    }


def fit_power_law(xs: List[float], ys: List[float]) -> Tuple[float, float, float]:
    """Fits y ~ c * x^alpha via log-log OLS; returns (alpha, c, r2)."""
    pts = [(math.log(x), math.log(y)) for x, y in zip(xs, ys) if x > 0 and y > 1e-6]
    if len(pts) < 2:
        return 0.0, 0.0, 0.0
    mx = sum(p[0] for p in pts) / len(pts)
    my = sum(p[1] for p in pts) / len(pts)
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    alpha = sxy / max(sxx, 1e-12)
    b = my - alpha * mx
    ss_tot = sum((p[1] - my) ** 2 for p in pts)
    ss_res = sum((p[1] - (alpha * p[0] + b)) ** 2 for p in pts)
    r2 = 1.0 - ss_res / max(ss_tot, 1e-12)
    return alpha, math.exp(b), r2


def main() -> int:
    ap = argparse.ArgumentParser(description="Asymptotic Scaling Study (5 to 500 aircraft)")
    ap.add_argument("--scales", default="5,12,30,60,100,150,200,300")
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--regime", default="fixed_theater", choices=["fixed_theater", "scaled_area"])
    ap.add_argument("--budget", type=float, default=45.0)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--out-name", default="exp3_scaling.json")
    args = ap.parse_args()

    scales = [int(x) for x in args.scales.split(",")]
    tasks = []
    for n_ac in scales:
        run_base = n_ac <= 200  # greedy_plan is super slow at >=300; test separately if needed
        for s in range(args.seeds):
            tasks.append((n_ac, 2000 + s, args.regime, args.budget, run_base))

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    t_wall = time.perf_counter()
    rows: List[dict] = []
    with concurrent.futures.ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = {ex.submit(run_single_scale_case, t): t for t in tasks}
        for fut in concurrent.futures.as_completed(futs):
            t = futs[fut]
            try:
                r = fut.result(timeout=180)
                rows.append(r)
                print(
                    f"  N={r['n_aircraft']:3d} seed={r['seed']} | valid={r['valid']} <=60s={r['within_60s']} "
                    f"t={r['t_total_s']:6.2f}s (init={r['prof']['t_init']:5.2f}s, knap={r['prof']['t_knapsack']:5.2f}s/{r['prof']['n_knapsack']}, "
                    f"eval={r['prof']['t_eval_subset']:5.2f}s/{r['prof']['n_eval_subset']}, deconf={r['prof']['t_deconflict']:5.2f}s/{r['prof']['n_deconflict']}, "
                    f"val={r['t_validate_s']:5.3f}s) | score={r['score']:7.1f} (base={r['base_score']}, x={r['ratio_vs_base']}) "
                    f"served={r['served_count']}/{r['n_hostile']}H routes={r['active_routes']}",
                    flush=True,
                )
            except Exception as exc:
                print(f"  N={t[0]:3d} seed={t[1]} | FAILED: {exc}", flush=True)
                rows.append({"n_aircraft": t[0], "seed": t[1], "regime": t[2], "error": str(exc), "valid": False})

    rows.sort(key=lambda r: (r["n_aircraft"], r["seed"]))
    print(f"\n=== EXP 3 SUMMARY ({args.regime}, {len(rows)} runs in {time.perf_counter()-t_wall:.1f}s) ===")
    agg = {}
    for n_ac in scales:
        sub = [r for r in rows if r["n_aircraft"] == n_ac and "error" not in r]
        if not sub:
            continue
        avg = lambda fn: sum(fn(r) for r in sub) / len(sub)
        n_ok = sum(1 for r in sub if r["valid"])
        n_60 = sum(1 for r in sub if r["within_60s"])
        knap_per = avg(lambda r: r["prof"]["t_knapsack"] / max(1, r["prof"]["n_knapsack"]))
        deconf_per = avg(lambda r: r["prof"]["t_deconflict"] / max(1, r["prof"]["n_deconflict"]))
        eval_per = avg(lambda r: r["prof"]["t_eval_subset"] / max(1, r["prof"]["n_eval_subset"]))
        base_vals = [r["ratio_vs_base"] for r in sub if r["ratio_vs_base"] is not None]
        ratio_avg = sum(base_vals) / len(base_vals) if base_vals else None
        agg[n_ac] = {
            "n": len(sub),
            "valid": n_ok,
            "within_60s": n_60,
            "t_total": round(avg(lambda r: r["t_total_s"]), 2),
            "t_init": round(avg(lambda r: r["prof"]["t_init"]), 3),
            "t_knap_per_call": round(knap_per, 4),
            "t_eval_per_call": round(eval_per, 4),
            "t_deconf_per_call": round(deconf_per, 3),
            "n_eval_calls": round(avg(lambda r: r["prof"]["n_eval_subset"]), 1),
            "n_deconf_calls": round(avg(lambda r: r["prof"]["n_deconflict"]), 1),
            "t_validate": round(avg(lambda r: r["t_validate_s"]), 4),
            "score": round(avg(lambda r: r["score"]), 1),
            "served": round(avg(lambda r: r["served_count"]), 1),
            "routes": round(avg(lambda r: r["active_routes"]), 1),
            "ratio_vs_base": None if ratio_avg is None else round(ratio_avg, 3),
        }
        a = agg[n_ac]
        print(
            f"  N={n_ac:3d}: valid={a['valid']}/{a['n']} <=60s={a['within_60s']}/{a['n']} "
            f"t_tot={a['t_total']:6.2f}s | init={a['t_init']:5.2f}s | knap/c={a['t_knap_per_call']:6.4f}s | "
            f"eval/c={a['t_eval_per_call']:6.3f}s (x{a['n_eval_calls']:4.1f}) | "
            f"deconf/c={a['t_deconf_per_call']:5.2f}s (x{a['n_deconf_calls']:3.1f}) | "
            f"val={a['t_validate']:6.4f}s | score={a['score']:7.1f} (xBase={a['ratio_vs_base']})"
        )

    # Power-law fits on N >= 30
    fit_ns = [float(n) for n in sorted(agg.keys()) if n >= 30]
    fits = {}
    for metric_name, key in [
        ("t_init", "t_init"),
        ("t_knap_per_call", "t_knap_per_call"),
        ("t_eval_per_call", "t_eval_per_call"),
        ("t_deconf_per_call", "t_deconf_per_call"),
        ("t_validate", "t_validate"),
    ]:
        ys = [agg[int(n)][key] for n in fit_ns]
        alpha, c, r2 = fit_power_law(fit_ns, ys)
        fits[metric_name] = {"alpha": round(alpha, 3), "c": c, "r2": round(r2, 4)}
        print(f"  Power-law {metric_name:18s}: T(N) ~ {c:.2e} * N^{alpha:.2f}  (R^2={r2:.4f})")

    save_json({"rows": rows, "agg": agg, "fits": fits}, OUT_DIR / args.out_name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
