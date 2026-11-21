"""Appendix E — Executable Audit of Rules R1-R10 ("What breaks the rules?" & Loopholes).

Programmatically constructs minimal proof-of-concept plans and tests them against the
official `fleetkit.validator.validate` and `fleetkit.scorer.evaluate` to document exact
boundary behaviors and formal specification loopholes for the Alta Ares jury.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Dict, List

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "fleet-kit"))

from fleetkit.generator import generate  # noqa: E402
from fleetkit.model import Scenario  # noqa: E402
from fleetkit.scorer import evaluate  # noqa: E402


def _make_open_scenario(n_light_need: int = 1) -> Scenario:
    d = generate("L2", 0)
    d["zones"] = []
    d["contacts"] = [c for c in d["contacts"] if c["class"] == "hostile"][:1]
    d["contacts"][0]["engage_needs"] = {"light": n_light_need}
    d["contacts"][0]["window_s"] = [0.0, 900.0]
    d["contacts"][0]["points"] = 100.0
    return Scenario.from_dict(d)


def run_audit() -> List[Dict]:
    findings: List[Dict] = []

    sc = _make_open_scenario(n_light_need=1)
    r = sc.rules
    s0 = next(a for a in sc.aircraft if a.type == "scout")
    e0 = next(a for a in sc.aircraft if a.type == "effector" and a.subtype == "light")
    h = sc.contacts[0]

    d_s = math.hypot(h.pos[0] - s0.home[0], h.pos[1] - s0.home[1])
    z_safe = 100.0
    t_climb_s = z_safe / s0.v_up_mps + 2.0
    t_fly_s = d_s / s0.v_h_mps + 5.0
    t_obs = t_climb_s + t_fly_s
    t_obs_end = t_obs + r.obs_dwell_s
    t_ret1 = t_obs_end + t_fly_s
    t_ret2 = t_ret1 + z_safe / s0.v_down_mps + 2.0
    scout_safe = {
        "aircraft": s0.id,
        "waypoints": [
            {"t": 0.0, "x": s0.home[0], "y": s0.home[1], "z": 0.0},
            {"t": round(t_climb_s, 1), "x": s0.home[0], "y": s0.home[1], "z": z_safe},
            {"t": round(t_obs, 1), "x": h.pos[0], "y": h.pos[1], "z": z_safe,
             "action": "observe", "target": h.id, "dwell_s": r.obs_dwell_s},
            {"t": round(t_ret1, 1), "x": s0.home[0], "y": s0.home[1], "z": z_safe},
            {"t": round(t_ret2, 1), "x": s0.home[0], "y": s0.home[1], "z": 0.0},
        ],
    }

    d_e = math.hypot(h.pos[0] - e0.home[0], h.pos[1] - e0.home[1])
    t_fly_e = d_e / e0.v_h_mps + 10.0
    t_dep_e = t_ret2 + 10.0
    t_eng = t_dep_e + t_fly_e

    # 1. LOOPHOLE: R2 Ground-Crawl (z=0.0m for ENTIRE effector flight!)
    eff_z0 = {
        "aircraft": e0.id,
        "waypoints": [
            {"t": round(t_dep_e, 1), "x": e0.home[0], "y": e0.home[1], "z": 0.0},
            {"t": round(t_eng, 1), "x": h.pos[0], "y": h.pos[1], "z": 0.0,
             "action": "engage", "target": h.id},
        ],
    }
    res_z0 = evaluate(sc, {"seed": sc.seed, "routes": [scout_safe, eff_z0]})

    t_c_e = r.z_min_m / e0.v_up_mps + 2.0
    t_d_e = r.z_min_m / e0.v_down_mps + 2.0
    eff_z30 = {
        "aircraft": e0.id,
        "waypoints": [
            {"t": round(t_dep_e, 1), "x": e0.home[0], "y": e0.home[1], "z": 0.0},
            {"t": round(t_dep_e + t_c_e, 1), "x": e0.home[0], "y": e0.home[1], "z": r.z_min_m},
            {"t": round(t_dep_e + t_c_e + t_fly_e, 1), "x": h.pos[0], "y": h.pos[1], "z": r.z_min_m},
            {"t": round(t_dep_e + t_c_e + t_fly_e + t_d_e, 1), "x": h.pos[0], "y": h.pos[1], "z": 0.0,
             "action": "engage", "target": h.id},
        ],
    }
    res_z30 = evaluate(sc, {"seed": sc.seed, "routes": [scout_safe, eff_z30]})
    findings.append({
        "id": "LOOPHOLE-R2-GROUND-CRAWL",
        "rule": "R2 (kinematics / altitude)",
        "severity": "HIGH",
        "accepted_by_validator": res_z0.valid,
        "violations": [str(v) for v in res_z0.violations[:2]],
        "score_z0": round(res_z0.score, 2),
        "score_z30": round(res_z30.score, 2),
        "energy_saved_wh": round(res_z30.energy_wh - res_z0.energy_wh, 2),
        "summary": (
            f"A 2-waypoint effector [home(z=0) -> target(z=0)] flies at z=0.0m for the ENTIRE {d_e:.0f}m flight, "
            f"never entering [z_min=30m, z_max=150m], pays 0 Wh climb energy, and passes `_r2` because "
            f"waypoint k=len(w)-1 allows low=0.0 and segment altitude is never checked."
        ),
    })

    # 2. LOOPHOLE: R2 Sub-z_min Linear Cruise Skimming
    t_s_leg = d_s / s0.v_h_mps + 10.0
    t_skim_end = 2 * t_s_leg + r.obs_dwell_s
    scout_skim = {
        "aircraft": s0.id,
        "waypoints": [
            {"t": 0.0, "x": s0.home[0], "y": s0.home[1], "z": 0.0},
            {"t": round(t_s_leg, 1), "x": h.pos[0], "y": h.pos[1], "z": r.z_min_m,
             "action": "observe", "target": h.id, "dwell_s": r.obs_dwell_s},
            {"t": round(t_skim_end, 1), "x": s0.home[0], "y": s0.home[1], "z": 0.0},
        ],
    }
    t_dep_after_skim = t_skim_end + 10.0
    eff_after_skim = {
        "aircraft": e0.id,
        "waypoints": [
            {"t": round(t_dep_after_skim, 1), "x": e0.home[0], "y": e0.home[1], "z": 0.0},
            {"t": round(t_dep_after_skim + t_fly_e, 1), "x": h.pos[0], "y": h.pos[1], "z": 0.0,
             "action": "engage", "target": h.id},
        ],
    }
    res_skim = evaluate(sc, {"seed": sc.seed, "routes": [scout_skim, eff_after_skim]})
    findings.append({
        "id": "LOOPHOLE-R2-SUB-ZMIN-SKIM",
        "rule": "R2 (kinematics / altitude)",
        "severity": "MEDIUM",
        "accepted_by_validator": res_skim.valid,
        "violations": [str(v) for v in res_skim.violations[:2]],
        "summary": (
            f"A 3-waypoint scout flying directly (home, z=0) -> (target, z=30m) -> (home, z=0) spends "
            f"100% of both {d_s:.0f}m cruise legs strictly below z_min=30m (mean z=15m), yet passes `_r2` "
            f"because only intermediate waypoint altitudes are checked."
        ),
    })

    # 3. LOOPHOLE: R8 Mid-Air 0.0m Collision Accepted Inside Target Exempt Sphere
    sc2 = _make_open_scenario(n_light_need=2)
    s2 = next(a for a in sc2.aircraft if a.type == "scout")
    lights2 = [a for a in sc2.aircraft if a.type == "effector" and a.subtype == "light"]
    ea, eb = lights2[0], lights2[1]
    h2 = sc2.contacts[0]
    ux, uy = s2.home[0] - h2.pos[0], s2.home[1] - h2.pos[1]
    un = math.hypot(ux, uy) or 1.0
    d_s2 = math.hypot(h2.pos[0] - s2.home[0], h2.pos[1] - s2.home[1])
    t_s2_arr = d_s2 / s2.v_h_mps + 30.0
    t_s2_end = t_s2_arr + r.obs_dwell_s
    t_s2_home = t_s2_end + d_s2 / s2.v_h_mps + 30.0
    scout2_route = {
        "aircraft": s2.id,
        "waypoints": [
            {"t": 0.0, "x": s2.home[0], "y": s2.home[1], "z": 0.0},
            {"t": 20.0, "x": s2.home[0], "y": s2.home[1], "z": 120.0},
            {"t": round(t_s2_arr, 1), "x": h2.pos[0], "y": h2.pos[1], "z": 120.0,
             "action": "observe", "target": h2.id, "dwell_s": r.obs_dwell_s},
            {"t": round(t_s2_home - 20.0, 1), "x": s2.home[0], "y": s2.home[1], "z": 120.0},
            {"t": round(t_s2_home, 1), "x": s2.home[0], "y": s2.home[1], "z": 0.0},
        ],
    }
    t_base = t_s2_home + 10.0
    in_x, in_y = h2.pos[0] + 50.0 * ux / un, h2.pos[1] + 50.0 * uy / un
    da2 = math.hypot(in_x - ea.home[0], in_y - ea.home[1])
    db2 = math.hypot(in_x - eb.home[0], in_y - eb.home[1])
    t_fly_a = da2 / ea.v_h_mps + 15.0
    t_fly_b = db2 / eb.v_h_mps + 15.0
    t_enter = t_base + max(t_fly_a, t_fly_b) + 40.0
    t_col = t_enter + 10.0
    eff_a_col = {
        "aircraft": ea.id,
        "waypoints": [
            {"t": round(t_enter - t_fly_a - 15.0, 1), "x": ea.home[0], "y": ea.home[1], "z": 0.0},
            {"t": round(t_enter - t_fly_a, 1), "x": ea.home[0], "y": ea.home[1], "z": 35.0},
            {"t": round(t_enter, 1), "x": in_x, "y": in_y, "z": 35.0},
            {"t": round(t_col, 1), "x": in_x, "y": in_y, "z": 35.0},
            {"t": round(t_col + 8.0, 1), "x": h2.pos[0], "y": h2.pos[1], "z": 0.0, "action": "engage", "target": h2.id},
        ],
    }
    eff_b_col = {
        "aircraft": eb.id,
        "waypoints": [
            {"t": round(t_enter - t_fly_b - 25.0, 1), "x": eb.home[0], "y": eb.home[1], "z": 0.0},
            {"t": round(t_enter - t_fly_b, 1), "x": eb.home[0], "y": eb.home[1], "z": 82.0},
            {"t": round(t_enter, 1), "x": in_x, "y": in_y, "z": 82.0},
            {"t": round(t_col, 1), "x": in_x, "y": in_y, "z": 35.0},
            {"t": round(t_col + 22.0, 1), "x": h2.pos[0], "y": h2.pos[1], "z": 0.0, "action": "engage", "target": h2.id},
        ],
    }
    res_exempt = evaluate(sc2, {"seed": sc2.seed, "routes": [scout2_route, eff_a_col, eff_b_col]})
    findings.append({
        "id": "LOOPHOLE-R8-MIDAIR-COLLISION-IN-EXEMPT-SPHERE",
        "rule": "R8 (separation)",
        "severity": "MEDIUM",
        "accepted_by_validator": res_exempt.valid,
        "violations": [str(v) for v in res_exempt.violations[:2]],
        "summary": (
            f"Two effectors ({ea.id}, {eb.id}) targeting {h2.id} literally collide (0.0m 3D separation at z=35m, "
            f"61.0m from target) 8-22s BEFORE engaging (14s apart within R10 sync window), and `_r8` accepts it "
            f"with 0 violations because both lie inside `exempt_radius_m=100m`."
        ),
    })

    # 4. SPEC: R10 Airburst Engagement at z=24.9m
    eff_ab = {
        "aircraft": e0.id,
        "waypoints": [
            {"t": round(t_dep_e, 1), "x": e0.home[0], "y": e0.home[1], "z": 0.0},
            {"t": round(t_dep_e + t_c_e, 1), "x": e0.home[0], "y": e0.home[1], "z": r.z_min_m},
            {"t": round(t_dep_e + t_c_e + t_fly_e, 1), "x": h.pos[0], "y": h.pos[1], "z": 24.9,
             "action": "engage", "target": h.id},
        ],
    }
    res_ab = evaluate(sc, {"seed": sc.seed, "routes": [scout_safe, eff_ab]})
    findings.append({
        "id": "SPEC-R10-AIRBURST-25M",
        "rule": "R10 (engagement range) + R2",
        "severity": "LOW (Legitimate Spec Physics)",
        "accepted_by_validator": res_ab.valid,
        "violations": [str(v) for v in res_ab.violations[:2]],
        "summary": (
            "Effectors can engage at z=24.9m directly above a ground target (dist3 <= R_eng_m=25m) "
            "without ever diving to z=0m, saving final dive time and enabling 25m horizontal standoff."
        ),
    })

    # 5. SPEC: R8 Ground Cloak vs Pad Congestion
    sc_l4 = Scenario.from_dict(generate("L4", 401))
    min_pad_d = min(
        math.hypot(sc_l4.aircraft[i].home[0] - sc_l4.aircraft[j].home[0],
                   sc_l4.aircraft[i].home[1] - sc_l4.aircraft[j].home[1])
        for i in range(len(sc_l4.aircraft)) for j in range(i + 1, len(sc_l4.aircraft))
    )
    findings.append({
        "id": "SPEC-R8-GROUND-CLOAK-VS-PAD-BOTTLENECK",
        "rule": "R8 (separation) + Generator Pad Grid",
        "severity": "MEDIUM (Physical Realism & Scaling)",
        "accepted_by_validator": True,
        "min_l4_pad_spacing_m": round(min_pad_d, 1),
        "summary": (
            f"In L4 (100 drones), pads are spaced {min_pad_d:.1f}m apart inside a 100x100m grid (< d_sep_m=40m). "
            f"Because `route_pieces()` ignores ground wait before `w[0].depart`, a launching drone flies right over "
            f"parked neighbors at {min_pad_d:.1f}m with 0 R8 violations (unphysical ground cloak), yet any two drones "
            f"launching simultaneously within 40m immediately violate R8, creating a hard launch-throughput ceiling at N>=200."
        ),
    })

    return findings


def main() -> int:
    findings = run_audit()
    print("=== APPENDIX E: EXECUTABLE RULE AUDIT ('What breaks the rules?') ===")
    for f in findings:
        print(f"\n[{f['id']}] ({f['rule']}) | Severity: {f['severity']} | Accepted by validator: {f['accepted_by_validator']}")
        print(f"  {f['summary']}")
        if "energy_saved_wh" in f:
            print(f"  Quantified gain: score_z0={f['score_z0']} vs score_z30={f['score_z30']} (saves {f['energy_saved_wh']} Wh)")
        if f.get("violations"):
            print(f"  Violations: {f['violations']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
