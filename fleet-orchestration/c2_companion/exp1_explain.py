"""Experiment 1: Layer B Explainability Audit across 28 scenarios (8 Val + 20 Unseen)."""
from __future__ import annotations

import concurrent.futures
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "fleet-kit"))
sys.path.insert(0, str(ROOT / "fleet-kit" / "handin"))
sys.path.insert(0, str(ROOT))

from fleetkit.baseline import greedy_plan  # noqa: E402
from fleetkit.generator import generate  # noqa: E402
from fleetkit.model import Scenario, load_json, save_json  # noqa: E402
from fleetkit.validation import VALIDATION_SET, scenario_id  # noqa: E402
from c2_companion.explain import explain_plan, format_markdown_briefing  # noqa: E402
from solver import solve_scenario  # noqa: E402

OUT_DIR = Path("/home/nisseya/.gemini/antigravity-acp/brain/d6074ffc-cefc-488c-9fb3-57a9bcb06b28/scratch/c2_exp_plans")


def _solve_one(args):
    lvl, seed = args
    name = f"{lvl}-{seed}"
    sc_dict = generate(lvl, seed)
    sc = Scenario.from_dict(sc_dict)
    t0 = time.perf_counter()
    plan = solve_scenario(sc, time_budget_s=45.0)
    dt = time.perf_counter() - t0
    save_json(sc_dict, OUT_DIR / f"{name}.scenario.json")
    save_json(plan, OUT_DIR / f"{name}.plan.json")
    rep = explain_plan(sc, plan)
    return name, dt, rep


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    val_rows = []
    for lvl, seed in VALIDATION_SET:
        name = scenario_id(lvl, seed)
        sc_dict = generate(lvl, seed)
        sc = Scenario.from_dict(sc_dict)
        plan = load_json(str(ROOT / "fleet-kit" / "plans" / f"{name}.json"))
        save_json(sc_dict, OUT_DIR / f"{name}.scenario.json")
        save_json(plan, OUT_DIR / f"{name}.plan.json")
        rep_v10 = explain_plan(sc, plan)
        rep_base = explain_plan(sc, greedy_plan(sc))
        val_rows.append((name, rep_v10, rep_base))

    tasks = [(lvl, s) for lvl in ("L1", "L2", "L3", "L4") for s in range(5)]
    t_par = time.perf_counter()
    with concurrent.futures.ProcessPoolExecutor(max_workers=8) as ex:
        unseen_rows = list(ex.map(_solve_one, tasks))
    print(f"Solved 20 unseen scenarios in {time.perf_counter() - t_par:.1f}s wall-clock")

    all_reps = [(n, r) for n, r, _ in val_rows] + [(n, r) for n, _, r in unseen_rows]
    by_level = {}
    max_cost_err = 0.0
    tot_rej_counts = {}
    tot_rej_pts = {}
    tot_tiers = {}

    for name, r in all_reps:
        cc = r["cost_decomposition_check"]
        err = abs(cc["total_cost_exact"] - (cc["attributed_to_served"] + cc["unattributed_scout_cost"]))
        max_cost_err = max(max_cost_err, err)
        lvl = r["level"]
        bl = by_level.setdefault(lvl, {
            "n": 0, "served": 0, "unserved": 0, "legal_pts": 0.0, "gross_pts": 0.0,
            "min_r8": 1e9, "min_scout_res": 1e9, "min_eff_res": 1e9, "min_r4": 1e9,
            "max_r10": 0.0, "explain_ms": [], "rej_counts": {}, "rej_pts": {}, "tiers": {},
        })
        bl["n"] += 1
        bl["served"] += r["target_coverage"]["served_count"]
        bl["unserved"] += r["target_coverage"]["unserved_count"]
        bl["legal_pts"] += r["target_coverage"]["legal_actionable_points"]
        bl["gross_pts"] += r["gross_points"]
        bl["explain_ms"].append(r["explain_runtime_ms"])
        se = r["safety_envelope"]
        if se["r8_closest_approach"]:
            bl["min_r8"] = min(bl["min_r8"], se["r8_closest_approach"]["distance_m"])
        if se["min_scout_battery_reserve_pct"] is not None:
            bl["min_scout_res"] = min(bl["min_scout_res"], se["min_scout_battery_reserve_pct"])
        if se["min_effector_battery_reserve_pct"] is not None:
            bl["min_eff_res"] = min(bl["min_eff_res"], se["min_effector_battery_reserve_pct"])
        if se["min_r4_confirm_margin_s"] is not None:
            bl["min_r4"] = min(bl["min_r4"], se["min_r4_confirm_margin_s"])
        if se["max_r10_sync_spread_s"] is not None:
            bl["max_r10"] = max(bl["max_r10"], se["max_r10_sync_spread_s"])
        for k, v in r["target_coverage"]["rejection_counts"].items():
            bl["rej_counts"][k] = bl["rej_counts"].get(k, 0) + v
            tot_rej_counts[k] = tot_rej_counts.get(k, 0) + v
        for k, v in r["target_coverage"]["rejection_points"].items():
            bl["rej_pts"][k] = bl["rej_pts"].get(k, 0.0) + v
            tot_rej_pts[k] = tot_rej_pts.get(k, 0.0) + v
        for k, v in r["target_coverage"]["roe_tier_breakdown"].items():
            bl["tiers"][k] = bl["tiers"].get(k, 0) + v
            tot_tiers[k] = tot_tiers.get(k, 0) + v

    n_valid = sum(1 for _, r in all_reps if r["valid"])
    print(f"\n=== EXP 1: EXPLAINABILITY AUDIT (28 scenarios: 8 Val + 20 Unseen) ===")
    print(f"Max cost conservation error: {max_cost_err:.4f} pts | Valid: {n_valid}/{len(all_reps)}")
    print("\nBy Level:")
    for lvl in ("L1", "L2", "L3", "L4"):
        b = by_level[lvl]
        cap = 100.0 * b["gross_pts"] / max(b["legal_pts"], 1e-6)
        avg_ms = sum(b["explain_ms"]) / len(b["explain_ms"])
        max_ms = max(b["explain_ms"])
        print(
            f"  {lvl} (n={b['n']}): served={b['served']}, unserved={b['unserved']}, "
            f"legal_capture={cap:.1f}%, min_R8={b['min_r8']:.2f}m, min_R4=+{b['min_r4']:.2f}s, "
            f"max_R10={b['max_r10']:.2f}s, min_batt(S/E)={b['min_scout_res']:.1f}%/{b['min_eff_res']:.1f}%, "
            f"explain_ms={avg_ms:.1f}ms (max {max_ms:.1f}ms)"
        )
        print(f"     rejections: {b['rej_counts']}")

    print("\nGlobal Unserved Breakdown (28 scenarios):")
    tot_u_cnt = sum(tot_rej_counts.values())
    tot_u_pts = sum(tot_rej_pts.values())
    for k, cnt in sorted(tot_rej_counts.items(), key=lambda x: -tot_rej_pts[x[0]]):
        print(
            f"  {k:28s}: {cnt:4d} contacts ({100 * cnt / tot_u_cnt:5.1f}%) | "
            f"{tot_rej_pts[k]:8.1f} pts ({100 * tot_rej_pts[k] / tot_u_pts:5.1f}%)"
        )

    print("\nGlobal Served ROE Tiers (28 scenarios):")
    tot_s_cnt = sum(tot_tiers.values())
    for k, cnt in sorted(tot_tiers.items(), key=lambda x: -x[1]):
        print(f"  {k:28s}: {cnt:4d} contacts ({100 * cnt / tot_s_cnt:5.1f}%)")

    print("\nValidation V10 vs Greedy Baseline:")
    for name, rv, rb in val_rows:
        print(
            f"  {name:7s}: V10={rv['score']:7.1f} ({rv['target_coverage']['gross_capture_of_legal_pct']:5.1f}% legal, "
            f"{rv['target_coverage']['served_count']:2d} served) "
            f"vs Base={rb['score']:7.1f} ({rb['target_coverage']['gross_capture_of_legal_pct']:5.1f}% legal, "
            f"{rb['target_coverage']['served_count']:2d} served) "
            f"| V10 rej={rv['target_coverage']['rejection_counts']}"
        )

    save_json({
        "max_cost_err": max_cost_err,
        "tot_rej_counts": tot_rej_counts,
        "tot_rej_pts": tot_rej_pts,
        "tot_tiers": tot_tiers,
        "by_level": by_level,
    }, OUT_DIR / "exp1_summary.json")

    for name, rv, _ in val_rows:
        if name == "L4-401":
            (OUT_DIR / "L4-401.briefing.md").write_text(format_markdown_briefing(rv), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
