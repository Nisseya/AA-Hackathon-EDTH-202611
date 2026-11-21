"""Experiment 2: Layer C Fewer Approvals & Safe-Veto Certification + C2 Demo Viewer HTMLs."""
from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "fleet-kit"))
sys.path.insert(0, str(ROOT))

from fleetkit.model import Scenario, load_json, save_json  # noqa: E402
from fleetkit.validation import VALIDATION_SET, scenario_id  # noqa: E402
from c2_companion.explain import explain_plan  # noqa: E402
from c2_companion.packages import build_strike_packages, verify_all_safe_vetoes  # noqa: E402
from c2_companion.demo_viewer import render_c2_demo  # noqa: E402

PLAN_DIR = Path("/home/nisseya/.gemini/antigravity-acp/brain/d6074ffc-cefc-488c-9fb3-57a9bcb06b28/scratch/c2_exp_plans")
DEMO_DIR = ROOT / "demos"


def main() -> int:
    DEMO_DIR.mkdir(parents=True, exist_ok=True)
    names = [scenario_id(l, s) for l, s in VALIDATION_SET] + [
        f"{l}-{s}" for l in ("L1", "L2", "L3", "L4") for s in range(5)
    ]

    policies = [
        ("ATOMIC_CC", 999),
        ("ROE_WAVE_K", 8),
        ("ROE_WAVE_K", 6),
        ("ROE_WAVE_K", 4),
    ]

    results = {}
    t0 = time.perf_counter()
    for pol, kmax in policies:
        key = pol if pol == "ATOMIC_CC" else f"{pol}_{kmax}"
        by_lvl = {}
        tot_single_ok = tot_single = tot_multi_ok = tot_multi = 0
        for name in names:
            sc = Scenario.from_dict(load_json(str(PLAN_DIR / f"{name}.scenario.json")))
            plan = load_json(str(PLAN_DIR / f"{name}.plan.json"))
            exp = explain_plan(sc, plan)
            rep = build_strike_packages(sc, plan, policy=pol, k_max=kmax, explanation=exp)
            sv = verify_all_safe_vetoes(sc, plan, rep, n_random_subsets=8, seed=sc.seed)
            tot_single_ok += sv["single_vetoes_valid"]
            tot_single += sv["single_vetoes_total"]
            tot_multi_ok += sv["multi_vetoes_valid"]
            tot_multi += sv["multi_vetoes_total"]

            bl = by_lvl.setdefault(sc.level, {
                "n": 0, "routes": [], "targets": [], "ccs": [], "pkgs": [],
                "red_routes": [], "red_targets": [], "max_var": [],
                "all_safe": 0,
            })
            bl["n"] += 1
            bl["routes"].append(rep["n_active_routes"])
            bl["targets"].append(rep["n_served_contacts"])
            bl["ccs"].append(rep["n_atomic_components"])
            bl["pkgs"].append(rep["n_packages"])
            bl["red_routes"].append(rep["approval_reduction_vs_routes_pct"])
            bl["red_targets"].append(rep["approval_reduction_vs_targets_pct"])
            bl["max_var"].append(rep["max_single_package_value_share_pct"])
            bl["all_safe"] += int(sv["all_safe"])

        results[key] = {
            "by_level": by_lvl,
            "single_veto_ok": tot_single_ok,
            "single_veto_total": tot_single,
            "multi_veto_ok": tot_multi_ok,
            "multi_veto_total": tot_multi,
        }

    print(f"=== EXP 2: FEWER APPROVALS & SAFE-VETO CERTIFICATION (28 scenarios, {time.perf_counter()-t0:.1f}s) ===")
    for key, data in results.items():
        print(
            f"\nPolicy {key}: Single-Veto Valid = {data['single_veto_ok']}/{data['single_veto_total']} "
            f"| Multi-Veto Valid = {data['multi_veto_ok']}/{data['multi_veto_total']}"
        )
        for lvl in ("L1", "L2", "L3", "L4"):
            b = data["by_level"][lvl]
            avg = lambda xs: sum(xs) / len(xs)
            print(
                f"  {lvl} (n={b['n']}): routes={avg(b['routes']):5.1f} -> targets={avg(b['targets']):4.1f} "
                f"-> CCs={avg(b['ccs']):4.1f} -> pkgs={avg(b['pkgs']):4.1f} "
                f"(red vs routes: -{avg(b['red_routes']):4.1f}%, vs targets: -{avg(b['red_targets']):4.1f}%, "
                f"max_pkg_VaR={avg(b['max_var']):4.1f}%, safe={b['all_safe']}/{b['n']})"
            )

    # Render showcase offline HTML C2 demos for L3-301 and L4-401
    print("\n=== Rendering Showcase C2 Demo Viewers ===")
    for demo_id in ("L3-301", "L4-401"):
        res = render_c2_demo(
            str(PLAN_DIR / f"{demo_id}.scenario.json"),
            str(PLAN_DIR / f"{demo_id}.plan.json"),
            str(DEMO_DIR / f"c2_demo_{demo_id}.html"),
            k_max=6,
        )
        pr = res["packages"]
        print(f"  {demo_id}: {res['html']} ({pr['n_active_routes']} routes -> {pr['n_packages']} packages, -{pr['approval_reduction_vs_routes_pct']}%)")
        for p in pr["packages"]:
            print("    " + p["summary"])

    save_json(results, PLAN_DIR / "exp2_summary.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
