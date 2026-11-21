"""Layer A+B+C — Interactive Offline C2 Demo Viewer with Custom Tactical Layers.

Enriches the official `fleetkit.viewer.Viewer` with operator-grade overlays:
  - Per-Strike-Package convex hulls, sector labels, and ROE stratum color-coding
  - Timed Kill-Chain R4 links (confirming scout -> target during [t_obs_start, t_last_engage])
  - Counterfactual Rejected Contacts overlay (exact R7 collateral veto / exhausted / low ROI)
  - Global R8 Closest-Approach safety envelope highlight

Produces a single self-contained offline HTML file + matching JSON layers & Markdown briefing.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

KIT_DIR = Path(__file__).resolve().parent.parent / "fleet-kit"
if str(KIT_DIR) not in sys.path:
    sys.path.insert(0, str(KIT_DIR))

from fleetkit.model import Scenario, load_json  # noqa: E402
from fleetkit.viewer import Viewer  # noqa: E402

try:
    from .explain import explain_plan, format_markdown_briefing
    from .packages import build_strike_packages
except ImportError:
    from explain import explain_plan, format_markdown_briefing
    from packages import build_strike_packages

STRATUM_COLORS = {
    "RECON_ONLY": "#7ee787",
    "STANDARD_STRIKE": "#79c0ff",
    "HIGH_SCRUTINY_STRIKE": "#ffa657",
}
PKG_PALETTE = ["#79c0ff", "#d2a8ff", "#7ee787", "#ffa657", "#f778ba", "#ff7b72", "#56d4dd", "#e3b341"]


def build_c2_viewer_layers(scenario: Scenario, plan: dict,
                           k_max: int = 6) -> tuple[Dict[str, Any], Dict[str, Any], List[dict]]:
    """Computes explainability report, Strike Packages, and Viewer custom layers."""
    exp = explain_plan(scenario, plan)
    pkg_rep = build_strike_packages(scenario, plan, policy="ROE_WAVE_K", k_max=k_max, explanation=exp)
    layers: List[dict] = []

    # 1. One layer per Strike Package (hull + centroid badge + active window)
    for idx, p in enumerate(pkg_rep["packages"]):
        color = STRATUM_COLORS.get(p["stratum"], PKG_PALETTE[idx % len(PKG_PALETTE)])
        hull = p["hull"]
        lines = []
        circles = []
        if len(hull) >= 3:
            lines.append(hull + [hull[0]])
        elif len(hull) == 2:
            lines.append(hull)
        elif len(hull) == 1:
            circles.append((hull[0][0], hull[0][1], 180.0))
        cx, cy = p["centroid"]
        badge = f"{p['package_id']} ({p['stratum'][:4]} · {len(p['aircraft'])}AC · {p['net_value']:+.0f})"
        layers.append({
            "name": f"{p['package_id']}: {p['title']}",
            "color": color,
            "visible": True,
            "lines": lines,
            "circles": circles,
            "labels": [(cx, cy, badge)],
        })

    # 2. Timed R4 Kill-Chain Handoffs (scout obs position -> contact, visible during kill-chain)
    routes_by_id = {r["aircraft"]: r["waypoints"] for r in plan.get("routes", [])}
    kc_lines = []
    kc_labels = []
    for j in exp["served_contacts"]:
        if j["class"] != "hostile":
            continue
        sid = j["primary_scout"]
        wps = routes_by_id.get(sid, [])
        obs_wp = next((w for w in wps if w.get("action") == "observe" and w.get("target") == j["contact_id"]), None)
        if obs_wp is not None:
            t0 = j["t_obs_start_s"]
            t1 = j["t_last_engage_s"] if j["t_last_engage_s"] is not None else j["t_obs_end_s"]
            kc_lines.append({
                "pts": [[obs_wp["x"], obs_wp["y"]], [j["pos"][0], j["pos"][1]]],
                "t0": t0,
                "t1": t1,
                "width": 2.0,
            })
            kc_labels.append({
                "x": j["pos"][0],
                "y": j["pos"][1] + 55.0,
                "label": f"{j['contact_id']} +{j['net_value']:.0f} (R4+{j['r4_margin_s']:.0f}s)",
                "t0": t0,
                "t1": t1 + 15.0,
            })
    if kc_lines:
        layers.append({
            "name": "Kill-Chain R4 Handoffs (timed)",
            "color": "#f778ba",
            "visible": True,
            "lines": kc_lines,
            "labels": kc_labels,
        })

    # 3. Counterfactual Rejected Contacts (why unserved?)
    rej_pts = []
    rej_circles = []
    short_code = {
        "R7_COLLATERAL_VETO": "R7 VETO",
        "NEGATIVE_ISOLATED_ROI": "LOW ROI",
        "ENERGY_RANGE_INFEASIBLE": "R3 RANGE",
        "WINDOW_R9_INFEASIBLE": "R9 WIN",
        "FLEET_EFFECTOR_SATURATED": "EFF FULL",
        "SCOUT_OR_R8_BOTTLENECK": "SCOUT/R8",
    }
    for u in exp["unserved_contacts"]:
        tag = short_code.get(u["reason_code"], u["reason_code"])
        rej_pts.append((u["pos"][0], u["pos"][1], f"{u['contact_id']} [{tag}]"))
        if u["reason_code"] == "R7_COLLATERAL_VETO":
            rej_circles.append((u["pos"][0], u["pos"][1], scenario.rules.collateral_radius_m))
    if rej_pts:
        layers.append({
            "name": "Rejected Contacts (Counterfactual Audit)",
            "color": "#ff7b72",
            "visible": True,
            "points": rej_pts,
            "circles": rej_circles,
        })

    # 4. R8 Closest Approach Safety Marker
    r8 = exp["safety_envelope"]["r8_closest_approach"]
    if r8:
        mx = 0.5 * (r8["pos_a"][0] + r8["pos_b"][0])
        my = 0.5 * (r8["pos_a"][1] + r8["pos_b"][1])
        layers.append({
            "name": f"R8 Min Separation ({r8['distance_m']:.1f}m @ t={r8['t_s']:.0f}s)",
            "color": "#e3b341",
            "visible": False,
            "lines": [{"pts": [[r8["pos_a"][0], r8["pos_a"][1]], [r8["pos_b"][0], r8["pos_b"][1]]],
                       "t0": max(0.0, r8["t_s"] - 15.0), "t1": r8["t_s"] + 15.0}],
            "circles": [{"x": mx, "y": my, "r": scenario.rules.d_sep_m,
                         "t0": max(0.0, r8["t_s"] - 15.0), "t1": r8["t_s"] + 15.0}],
            "labels": [{"x": mx, "y": my + 50.0,
                        "label": f"Min R8: {r8['distance_m']:.1f}m ({r8['aircraft_a']}-{r8['aircraft_b']})",
                        "t0": max(0.0, r8["t_s"] - 15.0), "t1": r8["t_s"] + 15.0}],
        })

    return exp, pkg_rep, layers


def render_c2_demo(scenario_path: str, plan_path: str, out_html: str, *,
                   k_max: int = 6, title: str | None = None) -> Dict[str, Any]:
    sc = Scenario.from_dict(load_json(scenario_path))
    plan = load_json(plan_path)
    exp, pkg_rep, layers = build_c2_viewer_layers(sc, plan, k_max=k_max)
    v = Viewer(
        sc, plan,
        title=title or f"C2 Demo — {exp['scenario_id']} | Score {exp['score']:.1f} | {pkg_rep['n_packages']} Packages",
    )
    v.add_layers(layers)
    v.save(out_html)
    out_p = Path(out_html)
    out_p.with_suffix(".layers.json").write_text(json.dumps(layers, indent=1) + "\n", encoding="utf-8")
    out_p.with_suffix(".briefing.md").write_text(format_markdown_briefing(exp), encoding="utf-8")
    return {"html": str(out_p), "exp": exp, "packages": pkg_rep}


def main() -> int:
    ap = argparse.ArgumentParser(description="Render enriched offline C2 HTML Demo Viewer")
    ap.add_argument("scenario")
    ap.add_argument("plan")
    ap.add_argument("-o", "--output", default="c2_demo.html")
    ap.add_argument("--k-max", type=int, default=6)
    ap.add_argument("--title")
    args = ap.parse_args()
    res = render_c2_demo(args.scenario, args.plan, args.output, k_max=args.k_max, title=args.title)
    print(
        f"Wrote {res['html']} (score={res['exp']['score']:.1f}, "
        f"{res['packages']['n_packages']} strike packages, "
        f"-{res['packages']['approval_reduction_vs_routes_pct']}% approvals)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
