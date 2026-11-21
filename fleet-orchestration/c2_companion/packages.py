"""Layer C — Fewer Approvals Engine: Atomic ROE-Valid Strike Packages & Safe-Veto Certification.

Groups individual drone trajectories into minimal, human-approvable tactical packages
while mathematically guaranteeing Zero-Cascade Safe Rejection under Rules R1-R10:
  - Theorem: If every package P_k is a disjoint union of dependency connected components
    (complete scout-target-effector subgraphs), then vetoing ANY arbitrary subset of
    packages by deleting their aircraft routes ALWAYS produces a 100% R1-R10 valid plan.

Implements & compares 3 policies:
  1. BASELINE_ROUTES: per-aircraft approval (K = N_active_routes)
  2. ATOMIC_CC: exact dependency connected components
  3. ROE_WAVE_K: ROE-risk-stratified agglomerative spatio-temporal clustering (default K_max=6)

Pure Python stdlib + `fleetkit`. Does not modify `handin/`.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

KIT_DIR = Path(__file__).resolve().parent.parent / "fleet-kit"
if str(KIT_DIR) not in sys.path:
    sys.path.insert(0, str(KIT_DIR))

from fleetkit.model import (  # noqa: E402
    EFFECTOR,
    HOSTILE,
    SCOUT,
    Scenario,
    load_json,
    parse_plan,
)
from fleetkit.scorer import evaluate  # noqa: E402

try:
    from .explain import explain_plan
except ImportError:
    from explain import explain_plan


def _convex_hull_2d(pts: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Andrew's monotone chain 2D convex hull."""
    uniq = sorted({(round(x, 1), round(y, 1)) for x, y in pts})
    if len(uniq) <= 2:
        return uniq

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in uniq:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(uniq):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _sector_label(cx: float, cy: float, area: float) -> str:
    """Compass sector relative to theater center."""
    mx, my = area * 0.5, area * 0.5
    dx, dy = cx - mx, cy - my
    if math.hypot(dx, dy) < area * 0.12:
        return "CENTER"
    ns = "N" if dy >= 0 else "S"
    ew = "E" if dx >= 0 else "W"
    return ns + ew


def _build_atomic_components(scenario: Scenario, plan: dict, exp: Dict[str, Any]) -> List[dict]:
    """Extracts exact dependency connected components over (active aircraft, served contacts)."""
    routes, _ = parse_plan(plan)
    if not routes:
        return []

    served_by_id = {j["contact_id"]: j for j in exp["served_contacts"]}
    # Adjacency over tagged nodes ("A:S01", "C:C04")
    adj: Dict[str, Set[str]] = {}
    for aid in routes:
        adj.setdefault(f"A:{aid}", set())
    for cid, j in served_by_id.items():
        cn = f"C:{cid}"
        adj.setdefault(cn, set())
        for sid in j["confirming_scouts"]:
            an = f"A:{sid}"
            adj.setdefault(an, set()).add(cn)
            adj[cn].add(an)
        for eid in j["effectors"]:
            an = f"A:{eid}"
            adj.setdefault(an, set()).add(cn)
            adj[cn].add(an)

    visited: Set[str] = set()
    components: List[dict] = []
    for node in sorted(adj.keys()):
        if node in visited:
            continue
        stack = [node]
        visited.add(node)
        comp_ac: List[str] = []
        comp_c: List[str] = []
        while stack:
            u = stack.pop()
            if u.startswith("A:"):
                comp_ac.append(u[2:])
            else:
                comp_c.append(u[2:])
            for v in adj[u]:
                if v not in visited:
                    visited.add(v)
                    stack.append(v)

        scouts = sorted(a for a in comp_ac if exp["route_stats"][a]["type"] == SCOUT)
        effectors = sorted(a for a in comp_ac if exp["route_stats"][a]["type"] == EFFECTOR)
        c_objs = [served_by_id[c] for c in sorted(comp_c)]
        if not c_objs:
            stratum = "RECON_ONLY"
            cx = sum(scenario.aircraft_by_id()[a].home[0] for a in comp_ac) / len(comp_ac)
            cy = sum(scenario.aircraft_by_id()[a].home[1] for a in comp_ac) / len(comp_ac)
            t_lo = min(exp["route_stats"][a]["t_launch_s"] for a in comp_ac)
            t_hi = max(exp["route_stats"][a]["t_end_s"] for a in comp_ac)
            gross = 0.0
            net = -sum(scenario.rules.lambda_e * exp["route_stats"][a]["energy_wh"] for a in comp_ac)
            min_collat = None
        else:
            has_hostile = any(c["class"] == HOSTILE for c in c_objs)
            has_high_scrutiny = any(c["roe_tier"] == "HIGH_SCRUTINY_COLLATERAL" for c in c_objs)
            if not has_hostile:
                stratum = "RECON_ONLY"
            elif has_high_scrutiny:
                stratum = "HIGH_SCRUTINY_STRIKE"
            else:
                stratum = "STANDARD_STRIKE"
            cx = sum(c["pos"][0] for c in c_objs) / len(c_objs)
            cy = sum(c["pos"][1] for c in c_objs) / len(c_objs)
            t_lo = min(c["t_obs_start_s"] for c in c_objs)
            t_hi = max((c["t_last_engage_s"] if c["t_last_engage_s"] is not None else c["t_obs_end_s"])
                       for c in c_objs)
            gross = sum(c["points"] for c in c_objs)
            net = sum(c["net_value"] for c in c_objs)
            # R7 collateral clearance applies strictly to lethal HOSTILE engagements
            collat_vals = [
                c["r7_collateral_clearance_m"]
                for c in c_objs
                if c["class"] == HOSTILE and c["r7_collateral_clearance_m"] is not None
            ]
            min_collat = min(collat_vals) if collat_vals else None

        components.append({
            "aircraft": sorted(comp_ac),
            "scouts": scouts,
            "effectors": effectors,
            "contacts": sorted(comp_c),
            "stratum": stratum,
            "centroid": (cx, cy),
            "t_span": (t_lo, t_hi),
            "gross_points": gross,
            "net_value": net,
            "min_collateral_m": min_collat,
        })
    return components


def _merge_two(a: dict, b: dict) -> dict:
    na = max(1, len(a["contacts"]))
    nb = max(1, len(b["contacts"]))
    cx = (a["centroid"][0] * na + b["centroid"][0] * nb) / (na + nb)
    cy = (a["centroid"][1] * na + b["centroid"][1] * nb) / (na + nb)
    collat_vals = [x for x in (a["min_collateral_m"], b["min_collateral_m"]) if x is not None]
    order = {"RECON_ONLY": 0, "STANDARD_STRIKE": 1, "HIGH_SCRUTINY_STRIKE": 2}
    stratum = a["stratum"] if order[a["stratum"]] >= order[b["stratum"]] else b["stratum"]
    return {
        "aircraft": sorted(set(a["aircraft"]) | set(b["aircraft"])),
        "scouts": sorted(set(a["scouts"]) | set(b["scouts"])),
        "effectors": sorted(set(a["effectors"]) | set(b["effectors"])),
        "contacts": sorted(set(a["contacts"]) | set(b["contacts"])),
        "stratum": stratum,
        "centroid": (cx, cy),
        "t_span": (min(a["t_span"][0], b["t_span"][0]), max(a["t_span"][1], b["t_span"][1])),
        "gross_points": a["gross_points"] + b["gross_points"],
        "net_value": a["net_value"] + b["net_value"],
        "min_collateral_m": min(collat_vals) if collat_vals else None,
    }


def _cluster_within_stratum(comps: List[dict], target_k: int, area: float, horizon: float,
                            total_net: float) -> List[dict]:
    """Agglomerative spatio-temporal clustering with value-at-risk balance regularization."""
    clusters = list(comps)
    if target_k <= 0 or len(clusters) <= target_k:
        return clusters

    space_scale = max(area / 3.0, 500.0)
    time_scale = max(horizon / 4.0, 100.0)
    max_val_share = max(0.38, 1.35 / max(1, target_k))

    while len(clusters) > target_k:
        best_pair = None
        best_dist = float("inf")
        for i in range(len(clusters)):
            ci = clusters[i]
            ti = 0.5 * (ci["t_span"][0] + ci["t_span"][1])
            for j in range(i + 1, len(clusters)):
                cj = clusters[j]
                tj = 0.5 * (cj["t_span"][0] + cj["t_span"][1])
                ds = math.hypot(ci["centroid"][0] - cj["centroid"][0],
                                ci["centroid"][1] - cj["centroid"][1]) / space_scale
                dt = abs(ti - tj) / time_scale
                d = math.hypot(ds, dt)
                if total_net > 0:
                    merged_share = (ci["net_value"] + cj["net_value"]) / total_net
                    if merged_share > max_val_share:
                        d += 2.0 * (merged_share - max_val_share)
                if d < best_dist:
                    best_dist = d
                    best_pair = (i, j)
        i, j = best_pair
        merged = _merge_two(clusters[i], clusters[j])
        clusters = [c for idx, c in enumerate(clusters) if idx not in (i, j)]
        clusters.append(merged)
    return clusters


def build_strike_packages(scenario: Scenario, plan: dict, *,
                          policy: str = "ROE_WAVE_K",
                          k_max: int = 6,
                          explanation: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Partitions a valid plan into human-approvable Strike Packages."""
    exp = explanation if explanation is not None else explain_plan(scenario, plan)
    comps = _build_atomic_components(scenario, plan, exp)
    n_routes = exp["fleet_utilization"]["active_aircraft"]
    n_served = exp["target_coverage"]["served_count"]

    all_xy = [c.pos[:2] for c in scenario.contacts] + [a.home[:2] for a in scenario.aircraft]
    area = max(max(p[0] for p in all_xy), max(p[1] for p in all_xy), 3000.0)
    total_net = max(exp["score"], 1.0)

    if policy == "ATOMIC_CC" or len(comps) <= k_max:
        final_groups = comps
    elif policy == "ROE_WAVE_K":
        recon = [c for c in comps if c["stratum"] == "RECON_ONLY"]
        std = [c for c in comps if c["stratum"] == "STANDARD_STRIKE"]
        high = [c for c in comps if c["stratum"] == "HIGH_SCRUTINY_STRIKE"]

        k_recon = 1 if recon else 0
        rem = max(1, k_max - k_recon)
        if high and std:
            k_high = max(1, min(len(high), round(rem * len(high) / (len(high) + len(std)))))
            k_std = max(1, rem - k_high)
            if k_high + k_std > rem:
                k_high = max(1, rem - k_std)
        elif high:
            k_high, k_std = min(len(high), rem), 0
        else:
            k_high, k_std = 0, min(len(std), rem)

        final_groups = (
            _cluster_within_stratum(recon, k_recon, area, scenario.horizon_s, total_net)
            + _cluster_within_stratum(std, k_std, area, scenario.horizon_s, total_net)
            + _cluster_within_stratum(high, k_high, area, scenario.horizon_s, total_net)
        )
    else:
        raise ValueError(f"Unknown policy {policy!r}")

    final_groups.sort(key=lambda g: (g["t_span"][0], g["stratum"]))

    served_by_id = {j["contact_id"]: j for j in exp["served_contacts"]}
    packages_out: List[dict] = []
    for idx, g in enumerate(final_groups, start=1):
        pid = f"PKG-{idx:02d}"
        sector = _sector_label(g["centroid"][0], g["centroid"][1], area)
        pts_xy = [(served_by_id[c]["pos"][0], served_by_id[c]["pos"][1]) for c in g["contacts"] if c in served_by_id]
        hull = _convex_hull_2d(pts_xy)
        t0, t1 = g["t_span"]
        share_pct = round(100.0 * g["net_value"] / max(exp["score"], 1e-6), 1)

        if g["stratum"] == "RECON_ONLY":
            title = f"ISR Recon Sweep ({sector}, {t0:.0f}-{t1:.0f}s)"
        elif g["stratum"] == "HIGH_SCRUTINY_STRIKE":
            c_txt = f"{g['min_collateral_m']:.0f}m" if g["min_collateral_m"] is not None else "n/a"
            title = f"High-Scrutiny Strike ({sector}, min R7 {c_txt}, {t0:.0f}-{t1:.0f}s)"
        else:
            title = f"Strike Wave {sector} ({t0:.0f}-{t1:.0f}s)"

        summary = (
            f"{pid} [{g['stratum']}] — {title}: "
            f"{len(g['contacts'])} targets ({len(g['scouts'])}S + {len(g['effectors'])}E = {len(g['aircraft'])} drones), "
            f"net {g['net_value']:+.1f} pts ({share_pct:.1f}% of mission value)."
        )
        packages_out.append({
            "package_id": pid,
            "title": title,
            "stratum": g["stratum"],
            "sector": sector,
            "t_start_s": round(t0, 1),
            "t_end_s": round(t1, 1),
            "centroid": [round(g["centroid"][0], 1), round(g["centroid"][1], 1)],
            "hull": [[round(p[0], 1), round(p[1], 1)] for p in hull],
            "aircraft": g["aircraft"],
            "scouts": g["scouts"],
            "effectors": g["effectors"],
            "contacts": g["contacts"],
            "gross_points": round(g["gross_points"], 1),
            "net_value": round(g["net_value"], 2),
            "value_share_pct": share_pct,
            "min_collateral_m": None if g["min_collateral_m"] is None else round(g["min_collateral_m"], 1),
            "summary": summary,
        })

    n_pkg = len(packages_out)
    route_reduction_pct = round(100.0 * (1.0 - n_pkg / max(n_routes, 1)), 1)
    target_reduction_pct = round(100.0 * (1.0 - n_pkg / max(n_served, 1)), 1)
    max_var_pct = max((p["value_share_pct"] for p in packages_out), default=0.0)

    return {
        "scenario_id": exp["scenario_id"],
        "policy": policy,
        "k_max": k_max,
        "n_atomic_components": len(comps),
        "n_packages": n_pkg,
        "n_active_routes": n_routes,
        "n_served_contacts": n_served,
        "approval_reduction_vs_routes_pct": route_reduction_pct,
        "approval_reduction_vs_targets_pct": target_reduction_pct,
        "max_single_package_value_share_pct": max_var_pct,
        "packages": packages_out,
    }


def apply_package_veto(scenario: Scenario, plan: dict, pkg_report: Dict[str, Any],
                       vetoed_ids: Set[str]) -> Dict[str, Any]:
    """Removes all aircraft in `vetoed_ids` and validates the remaining plan under R1-R10."""
    drop_ac: Set[str] = set()
    for p in pkg_report["packages"]:
        if p["package_id"] in vetoed_ids:
            drop_ac.update(p["aircraft"])
    reduced = {
        "version": plan.get("version", "1"),
        "seed": plan.get("seed", scenario.seed),
        "routes": [r for r in plan.get("routes", []) if r["aircraft"] not in drop_ac],
    }
    res = evaluate(scenario, reduced)
    orig = evaluate(scenario, plan)
    return {
        "vetoed_packages": sorted(vetoed_ids),
        "dropped_aircraft": len(drop_ac),
        "valid": res.valid,
        "violations": [str(v) for v in res.violations],
        "score_before": round(orig.score, 2),
        "score_after": round(res.score, 2),
        "retained_pct": round(100.0 * res.score / max(orig.score, 1e-6), 1),
    }


def verify_all_safe_vetoes(scenario: Scenario, plan: dict, pkg_report: Dict[str, Any],
                           n_random_subsets: int = 10, seed: int = 0) -> Dict[str, Any]:
    """Empirically certifies Zero-Cascade Safe Rejection for all single vetoes + random subsets."""
    pids = [p["package_id"] for p in pkg_report["packages"]]
    single_ok = 0
    for pid in pids:
        out = apply_package_veto(scenario, plan, pkg_report, {pid})
        if out["valid"]:
            single_ok += 1

    rng = random.Random(seed)
    multi_ok = 0
    multi_total = 0
    if len(pids) >= 2:
        for _ in range(n_random_subsets):
            k = rng.randint(1, len(pids) - 1)
            sub = set(rng.sample(pids, k))
            out = apply_package_veto(scenario, plan, pkg_report, sub)
            multi_total += 1
            if out["valid"]:
                multi_ok += 1

    return {
        "single_vetoes_valid": single_ok,
        "single_vetoes_total": len(pids),
        "multi_vetoes_valid": multi_ok,
        "multi_vetoes_total": multi_total,
        "all_safe": (single_ok == len(pids) and multi_ok == multi_total),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Build ROE-Valid Strike Packages for a fleetkit plan")
    ap.add_argument("scenario")
    ap.add_argument("plan")
    ap.add_argument("--policy", default="ROE_WAVE_K", choices=["ATOMIC_CC", "ROE_WAVE_K"])
    ap.add_argument("--k-max", type=int, default=6)
    ap.add_argument("--verify-vetoes", action="store_true")
    ap.add_argument("--json-out")
    args = ap.parse_args()

    sc = Scenario.from_dict(load_json(args.scenario))
    plan = load_json(args.plan)
    rep = build_strike_packages(sc, plan, policy=args.policy, k_max=args.k_max)
    if args.verify_vetoes:
        rep["safe_veto_certification"] = verify_all_safe_vetoes(sc, plan, rep)
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(rep, indent=2) + "\n", encoding="utf-8")
    else:
        print(f"=== {rep['scenario_id']} ({rep['policy']}, K_max={rep['k_max']}) ===")
        print(
            f"Packages: {rep['n_packages']} (from {rep['n_atomic_components']} atomic CCs, "
            f"{rep['n_active_routes']} routes -> -{rep['approval_reduction_vs_routes_pct']}%, "
            f"{rep['n_served_contacts']} targets -> -{rep['approval_reduction_vs_targets_pct']}%)"
        )
        for p in rep["packages"]:
            print("  " + p["summary"])
        if "safe_veto_certification" in rep:
            sv = rep["safe_veto_certification"]
            print(
                f"Safe-Veto Cert: single {sv['single_vetoes_valid']}/{sv['single_vetoes_total']}, "
                f"multi {sv['multi_vetoes_valid']}/{sv['multi_vetoes_total']}, all_safe={sv['all_safe']}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
