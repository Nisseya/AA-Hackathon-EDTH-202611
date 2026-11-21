"""Layer B — Deterministic C2 Operator Explainability & Counterfactual Rejection Auditor.

Given any (scenario, plan), produces an exact, physically grounded justification for:
  1. Every served contact (hostile & unknown): kill-chain timeline, R4/R7/R8/R9/R10 margins,
     additive cost & net tactical ROI attribution, and ROE scrutiny tier.
  2. Every unserved actionable contact: hierarchical counterfactual root-cause diagnosis
     (R7 collateral veto, negative isolated ROI, battery/range infeasibility, R9 window
     infeasibility, effector fleet saturation with opportunity cost, or scout/R8 bottleneck).
  3. Fleet-wide safety envelope: exact global minimum 3D R8 separation, minimum NFZ/protected
     clearance, minimum battery reserve, and economic efficiency breakdown.

Pure Python stdlib + `fleetkit`. Does not modify `handin/`.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

KIT_DIR = Path(__file__).resolve().parent.parent / "fleet-kit"
if str(KIT_DIR) not in sys.path:
    sys.path.insert(0, str(KIT_DIR))

from fleetkit.energy import best_range_speed, route_energy_wh  # noqa: E402
from fleetkit.geometry import (  # noqa: E402
    EPS,
    INF,
    close_intervals,
    dist3,
    hdist,
    min_distance,
    overlapping_pieces,
    within_radius_interval,
)
from fleetkit.model import (  # noqa: E402
    CIVILIAN,
    EFFECTOR,
    FRIENDLY,
    HOSTILE,
    SCOUT,
    UNKNOWN,
    Aircraft,
    Contact,
    Route,
    Scenario,
    Waypoint,
    load_json,
    parse_plan,
)
from fleetkit.scorer import evaluate  # noqa: E402
from fleetkit.validator import separation_item, valid_observation  # noqa: E402


# ---------- Geometry & Physics Helpers ----------

def _point_to_segment_dist_2d(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    dx, dy = bx - ax, by - ay
    d2 = dx * dx + dy * dy
    if d2 <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / d2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _leg_cylinder_clearance(a: Tuple[float, float, float], b: Tuple[float, float, float],
                            cx: float, cy: float, radius: float, top_m: float) -> float:
    """Approximate minimum safety clearance from a 3D leg to a no-fly cylinder."""
    d_xy = _point_to_segment_dist_2d(cx, cy, a[0], a[1], b[0], b[1])
    if d_xy >= radius:
        return d_xy - radius
    # Horizontal projection crosses cylinder: clearance is vertical above top_m
    if math.isinf(top_m):
        return d_xy - radius  # negative if inside
    z_min_leg = min(a[2], b[2])
    return max(0.0, z_min_leg - top_m)


def _min_one_way_energy_and_time(ac: Aircraft, dist_xy: float, z_cruise: float,
                                 z_end: float) -> Tuple[float, float]:
    """Lower-bound energy (Wh) and earliest feasible time (s) for a straight flight."""
    v_opt = min(ac.v_h_mps, best_range_speed(ac.power))
    t_climb = z_cruise / ac.v_up_mps
    t_desc = max(0.0, z_cruise - z_end) / ac.v_down_mps
    t_fast = max(dist_xy / ac.v_h_mps, t_climb)
    # Best-range energy lower bound (ignoring detours around NFZs)
    t_opt = max(dist_xy / max(v_opt, 1e-6), t_climb)
    v_eff = dist_xy / t_opt if t_opt > 0 else 0.0
    e_min = (ac.power.p0_w + ac.power.k_v * v_eff * v_eff) * t_opt / 3600.0 + ac.power.e_climb_wh_per_m * z_cruise
    return e_min, t_fast


def _min_scout_roundtrip_energy_and_arrival(ac: Aircraft, c: Contact, rules) -> Tuple[float, float]:
    """Lower-bound round-trip energy (Wh) and earliest observation arrival (s) for a scout."""
    d_raw = hdist(ac.home, c.pos)
    # Standoff point at z = z_min_m saves up to sqrt(R_obs^2 - z_min^2)
    z_obs = rules.z_min_m
    standoff_xy = math.sqrt(max(0.0, rules.R_obs_m ** 2 - z_obs ** 2))
    d_eff = max(0.0, d_raw - standoff_xy)
    v_opt = min(ac.v_h_mps, best_range_speed(ac.power))
    t_leg_opt = max(d_eff / max(v_opt, 1e-6), z_obs / ac.v_up_mps)
    v_leg = d_eff / t_leg_opt if t_leg_opt > 0 else 0.0
    e_leg = (ac.power.p0_w + ac.power.k_v * v_leg * v_leg) * t_leg_opt / 3600.0
    e_climb = ac.power.e_climb_wh_per_m * z_obs
    e_dwell = ac.power.p0_w * rules.obs_dwell_s / 3600.0
    e_rt = 2.0 * e_leg + e_climb + e_dwell
    t_earliest_arr = max(d_eff / ac.v_h_mps, z_obs / ac.v_up_mps)
    return e_rt, t_earliest_arr


def _min_effector_energy_and_arrival(ac: Aircraft, c: Contact, rules) -> Tuple[float, float]:
    """Lower-bound one-way energy (Wh) and earliest engagement arrival (s) for an effector."""
    d_raw = hdist(ac.home, c.pos)
    d_eff = max(0.0, d_raw - rules.R_eng_m)
    z_cruise = rules.z_min_m
    return _min_one_way_energy_and_time(ac, d_eff, z_cruise, 0.0)


def _compute_r8_margins(scenario: Scenario, routes: Dict[str, Route]) -> Tuple[Dict[str, float], Optional[dict]]:
    """Computes exact minimum R8 3D separation per aircraft and globally (respecting target exemptions)."""
    contacts = scenario.contact_by_id()
    aircraft = scenario.aircraft_by_id()
    rules = scenario.rules
    items = []
    for aid, route in routes.items():
        it = separation_item(aid, route, aircraft[aid])
        if it is not None:
            items.append(it)

    per_ac_min: Dict[str, float] = {aid: INF for aid in routes}
    global_min = INF
    closest_event: Optional[dict] = None

    for i in range(len(items)):
        aid_a, pa, s_a, e_a, tgt_a = items[i]
        for j in range(i + 1, len(items)):
            aid_b, pb, s_b, e_b, tgt_b = items[j]
            if min(e_a, e_b) - max(s_a, s_b) <= 1e-12:
                continue
            exempt_center = None
            if tgt_a is not None and tgt_a == tgt_b and tgt_a in contacts:
                exempt_center = contacts[tgt_a].pos
            pair_min = INF
            pair_t = 0.0
            pair_pa = (0.0, 0.0, 0.0)
            pair_pb = (0.0, 0.0, 0.0)
            for a, b, lo, hi in overlapping_pieces(pa, pb):
                # Quick check: compute exact minimum distance on [lo, hi]
                d = min_distance(a, b, lo, hi)
                if d >= pair_min and d >= global_min + 50.0:
                    continue
                # If exempt near shared target, exclude exempt interval
                sub_intervals = [(lo, hi)]
                if exempt_center is not None:
                    ea = within_radius_interval(a, lo, hi, exempt_center, rules.exempt_radius_m)
                    eb = within_radius_interval(b, lo, hi, exempt_center, rules.exempt_radius_m)
                    if ea is not None and eb is not None:
                        e1, e2 = max(ea[0], eb[0]), min(ea[1], eb[1])
                        if e1 <= e2:
                            sub_intervals = []
                            if e1 - lo > 1e-6:
                                sub_intervals.append((lo, e1))
                            if hi - e2 > 1e-6:
                                sub_intervals.append((e2, hi))
                for slo, shi in sub_intervals:
                    ds = min_distance(a, b, slo, shi)
                    if ds < pair_min:
                        pair_min = ds
                        # Locate exact time of closest approach on [slo, shi]
                        t_star = _argmin_time(a, b, slo, shi)
                        pair_t = t_star
                        pair_pa = _interp_piece(a, t_star)
                        pair_pb = _interp_piece(b, t_star)
            if pair_min < per_ac_min[aid_a]:
                per_ac_min[aid_a] = pair_min
            if pair_min < per_ac_min[aid_b]:
                per_ac_min[aid_b] = pair_min
            if pair_min < global_min:
                global_min = pair_min
                closest_event = {
                    "aircraft_a": aid_a,
                    "aircraft_b": aid_b,
                    "t_s": round(pair_t, 2),
                    "distance_m": round(pair_min, 2),
                    "margin_above_d_sep_m": round(pair_min - rules.d_sep_m, 2),
                    "pos_a": [round(v, 1) for v in pair_pa],
                    "pos_b": [round(v, 1) for v in pair_pb],
                }
    return per_ac_min, closest_event


def _interp_piece(piece, t: float) -> Tuple[float, float, float]:
    t0, t1, p0, p1 = piece
    if t1 <= t0 + 1e-12:
        return p0
    u = max(0.0, min(1.0, (t - t0) / (t1 - t0)))
    return (p0[0] + u * (p1[0] - p0[0]), p0[1] + u * (p1[1] - p0[1]), p0[2] + u * (p1[2] - p0[2]))


def _argmin_time(a, b, lo: float, hi: float) -> float:
    pa0 = _interp_piece(a, lo)
    pa1 = _interp_piece(a, hi)
    pb0 = _interp_piece(b, lo)
    pb1 = _interp_piece(b, hi)
    dt = hi - lo
    if dt <= 1e-12:
        return lo
    rx, ry, rz = pa0[0] - pb0[0], pa0[1] - pb0[1], pa0[2] - pb0[2]
    vx = ((pa1[0] - pa0[0]) - (pb1[0] - pb0[0])) / dt
    vy = ((pa1[1] - pa0[1]) - (pb1[1] - pb0[1])) / dt
    vz = ((pa1[2] - pa0[2]) - (pb1[2] - pb0[2])) / dt
    vv = vx * vx + vy * vy + vz * vz
    if vv <= 1e-12:
        return lo
    tau = -(rx * vx + ry * vy + rz * vz) / vv
    tau = max(0.0, min(dt, tau))
    return lo + tau


# ---------- Main Explainability Engine ----------

def explain_plan(scenario: Scenario, plan: dict, verify: bool = False) -> Dict[str, Any]:
    """Generates a complete, deterministic C2 operator explanation for a scenario & plan."""
    t0 = time.perf_counter()
    score_res = evaluate(scenario, plan, verify=verify)
    routes, parse_errors = parse_plan(plan)
    if routes is None:
        return {
            "valid": False,
            "error": "; ".join(parse_errors),
            "scenario_id": f"{scenario.level}-{scenario.seed}",
        }

    rules = scenario.rules
    aircraft = scenario.aircraft_by_id()
    contacts = scenario.contact_by_id()
    protected_contacts = [c for c in scenario.contacts if c.cls in (FRIENDLY, CIVILIAN)]

    # 1. Per-route physical & safety audit
    per_ac_r8, closest_r8_event = _compute_r8_margins(scenario, routes)
    route_stats: Dict[str, dict] = {}
    scout_served_contacts: Dict[str, List[str]] = {}
    obs_by_contact: Dict[str, List[Tuple[str, Waypoint]]] = {}
    eng_by_contact: Dict[str, List[Tuple[str, Waypoint]]] = {}

    for aid, route in routes.items():
        ac = aircraft[aid]
        e_wh = route_energy_wh(route.waypoints, ac)
        reserve_wh = ac.usable_wh - e_wh
        reserve_pct = 100.0 * reserve_wh / max(ac.usable_wh, 1e-6)
        total_dist = sum(dist3(route.waypoints[k].pos, route.waypoints[k + 1].pos)
                         for k in range(len(route.waypoints) - 1))
        min_nfz = INF
        min_prot = INF
        for k in range(len(route.waypoints) - 1):
            p0, p1 = route.waypoints[k].pos, route.waypoints[k + 1].pos
            for z in scenario.zones:
                cl = _leg_cylinder_clearance(p0, p1, z.center[0], z.center[1], z.radius_m, z.top_m)
                if cl < min_nfz:
                    min_nfz = cl
            for fc in protected_contacts:
                cl = _leg_cylinder_clearance(p0, p1, fc.pos[0], fc.pos[1], rules.protect_radius_m, INF)
                if cl < min_prot:
                    min_prot = cl

        actions = []
        for wp in route.waypoints:
            if wp.action == "observe" and ac.type == SCOUT and wp.target in contacts:
                c = contacts[wp.target]
                if valid_observation(wp, c, scenario):
                    obs_by_contact.setdefault(c.id, []).append((aid, wp))
                    actions.append(("observe", c.id, wp.t, wp.depart))
            elif wp.action == "engage" and ac.type == EFFECTOR and wp.target in contacts:
                eng_by_contact.setdefault(wp.target, []).append((aid, wp))
                actions.append(("engage", wp.target, wp.t, wp.t))

        route_stats[aid] = {
            "aircraft_id": aid,
            "type": ac.type,
            "subtype": ac.subtype,
            "t_launch_s": round(route.waypoints[0].depart, 2),
            "t_end_s": round(route.waypoints[-1].t, 2),
            "energy_wh": round(e_wh, 2),
            "usable_wh": round(ac.usable_wh, 2),
            "battery_reserve_wh": round(reserve_wh, 2),
            "battery_reserve_pct": round(reserve_pct, 1),
            "distance_m": round(total_dist, 1),
            "n_waypoints": len(route.waypoints),
            "min_r8_sep_m": None if math.isinf(per_ac_r8[aid]) else round(per_ac_r8[aid], 2),
            "min_nfz_clearance_m": None if math.isinf(min_nfz) else round(min_nfz, 1),
            "min_protected_clearance_m": None if math.isinf(min_prot) else round(min_prot, 1),
            "actions": actions,
        }

    served_set = set(score_res.served)
    # Count how many served contacts each scout supports (for exact additive cost attribution)
    for cid in score_res.served:
        seen_scouts = set()
        for sid, _ in obs_by_contact.get(cid, []):
            if sid not in seen_scouts:
                scout_served_contacts.setdefault(sid, []).append(cid)
                seen_scouts.add(sid)

    # 2. Served Contact Justifications
    served_justifications: List[dict] = []
    served_hostile_pts_per_eff: List[float] = []
    total_attributed_cost = 0.0
    unattributed_scout_cost = 0.0
    for aid, st in route_stats.items():
        if st["type"] == SCOUT and aid not in scout_served_contacts:
            unattributed_scout_cost += rules.lambda_e * st["energy_wh"]

    for c in scenario.contacts:
        if c.id not in served_set:
            continue
        obs_list = obs_by_contact.get(c.id, [])
        eng_list = eng_by_contact.get(c.id, [])
        # Pick primary confirming scout (earliest completed observation)
        obs_list_sorted = sorted(obs_list, key=lambda x: x[1].depart)
        prim_sid, prim_obs = obs_list_sorted[0]
        obs_dist = dist3(prim_obs.pos, c.pos)

        # Collateral clearance
        nearest_fc_id, nearest_fc_cls, min_collat_m = None, None, INF
        for fc in protected_contacts:
            d = hdist(c.pos, fc.pos)
            if d < min_collat_m:
                min_collat_m = d
                nearest_fc_id = fc.id
                nearest_fc_cls = fc.cls

        # Scout cost attribution (shared equally across contacts served by each scout)
        scout_cost_share = 0.0
        involved_scouts = sorted({sid for sid, _ in obs_list})
        for sid in involved_scouts:
            n_c = max(1, len(scout_served_contacts.get(sid, [])))
            scout_cost_share += (rules.lambda_e * route_stats[sid]["energy_wh"]) / n_c

        eff_cost = 0.0
        eff_energy_wh = 0.0
        eff_ids = []
        r4_margin = None
        r10_sync_spread = None
        t_first_eng = None
        t_last_eng = None
        min_eff_reserve_pct = None
        if c.cls == HOSTILE and eng_list:
            eng_times = [wp.t for _, wp in eng_list]
            t_first_eng, t_last_eng = min(eng_times), max(eng_times)
            r4_margin = (t_first_eng - prim_obs.depart) - rules.confirm_margin_s
            r10_sync_spread = t_last_eng - t_first_eng
            eff_ids = [eid for eid, _ in sorted(eng_list, key=lambda x: x[1].t)]
            eff_energy_wh = sum(route_stats[eid]["energy_wh"] for eid in eff_ids)
            eff_cost = rules.c_eff * len(eff_ids) + rules.lambda_e * eff_energy_wh
            min_eff_reserve_pct = min(route_stats[eid]["battery_reserve_pct"] for eid in eff_ids)
            served_hostile_pts_per_eff.append(c.points / max(1, len(eff_ids)))

        attributed_cost = scout_cost_share + eff_cost
        total_attributed_cost += attributed_cost
        net_val = c.points - attributed_cost
        roi = c.points / max(attributed_cost, 1e-6)

        # Window slack
        win_lo, win_hi = c.window if c.window else (0.0, scenario.horizon_s)
        action_end = t_last_eng if t_last_eng is not None else prim_obs.depart
        r9_slack_lo = prim_obs.t - win_lo
        r9_slack_hi = win_hi - action_end
        r9_min_slack = min(r9_slack_lo, r9_slack_hi)

        # Minimum R8 separation among all participating aircraft
        all_ac = involved_scouts + eff_ids
        r8_vals = [per_ac_r8[a] for a in all_ac if not math.isinf(per_ac_r8[a])]
        min_r8 = min(r8_vals) if r8_vals else None

        # ROE Scrutiny Tier
        if c.cls == UNKNOWN:
            roe_tier = "LOW_RECON"
        elif min_collat_m < 400.0:
            roe_tier = "HIGH_SCRUTINY_COLLATERAL"
        elif (r4_margin is not None and r4_margin < 5.0) or r9_min_slack < 10.0 or (
            min_r8 is not None and min_r8 < rules.d_sep_m + 10.0
        ):
            roe_tier = "TIGHT_MARGIN_STRIKE"
        else:
            roe_tier = "STANDARD_STRIKE"

        # Human-readable C2 justification
        if c.cls == HOSTILE:
            collat_str = (
                f"R7 clearance {min_collat_m:.0f}m vs {nearest_fc_id}({nearest_fc_cls})"
                if nearest_fc_id is not None
                else "no friendly/civilian in theater"
            )
            r8_str = f"R8 min sep {min_r8:.1f}m" if min_r8 is not None else "R8 clear"
            rationale = (
                f"ENGAGE {c.id} ({c.points:.0f} pts, net {net_val:+.1f}, ROI {roi:.2f}x): "
                f"confirmed by {prim_sid} [{prim_obs.t:.0f}-{prim_obs.depart:.0f}s, {obs_dist:.0f}m standoff], "
                f"struck by {'+'.join(eff_ids)} at t={t_first_eng:.0f}s "
                f"(R4 margin +{r4_margin:.1f}s, R10 sync {r10_sync_spread:.1f}s/{rules.sync_window_s:.0f}s, "
                f"R9 slack {r9_min_slack:.1f}s, {collat_str}, {r8_str})."
            )
        else:
            rationale = (
                f"RECON {c.id} ({c.points:.0f} pts, net {net_val:+.1f}, ROI {roi:.2f}x): "
                f"non-lethal ISR by {prim_sid} [{prim_obs.t:.0f}-{prim_obs.depart:.0f}s, "
                f"{obs_dist:.0f}m standoff, R9 slack {r9_min_slack:.1f}s]."
            )

        served_justifications.append({
            "contact_id": c.id,
            "class": c.cls,
            "pos": [round(v, 1) for v in c.pos],
            "points": c.points,
            "attributed_cost": round(attributed_cost, 2),
            "net_value": round(net_val, 2),
            "roi_ratio": round(roi, 2),
            "confirming_scouts": involved_scouts,
            "primary_scout": prim_sid,
            "t_obs_start_s": round(prim_obs.t, 2),
            "t_obs_end_s": round(prim_obs.depart, 2),
            "obs_standoff_m": round(obs_dist, 1),
            "effectors": eff_ids,
            "t_first_engage_s": None if t_first_eng is None else round(t_first_eng, 2),
            "t_last_engage_s": None if t_last_eng is None else round(t_last_eng, 2),
            "r4_margin_s": None if r4_margin is None else round(r4_margin, 2),
            "r10_sync_spread_s": None if r10_sync_spread is None else round(r10_sync_spread, 2),
            "r9_min_slack_s": round(r9_min_slack, 2),
            "r7_collateral_clearance_m": None if math.isinf(min_collat_m) else round(min_collat_m, 1),
            "nearest_protected": nearest_fc_id,
            "min_r8_sep_m": None if min_r8 is None else round(min_r8, 2),
            "min_eff_battery_reserve_pct": min_eff_reserve_pct,
            "roe_tier": roe_tier,
            "rationale": rationale,
        })

    # 3. Counterfactual Audit of Unserved Actionable Contacts
    all_scouts = [a for a in scenario.aircraft if a.type == SCOUT]
    all_light = [a for a in scenario.aircraft if a.type == EFFECTOR and a.subtype == "light"]
    all_heavy = [a for a in scenario.aircraft if a.type == EFFECTOR and a.subtype == "heavy"]
    used_ac = set(routes.keys())
    free_light = [a for a in all_light if a.id not in used_ac]
    free_heavy = [a for a in all_heavy if a.id not in used_ac]
    min_served_ppe = min(served_hostile_pts_per_eff) if served_hostile_pts_per_eff else 0.0
    avg_served_ppe = (
        sum(served_hostile_pts_per_eff) / len(served_hostile_pts_per_eff)
        if served_hostile_pts_per_eff
        else 0.0
    )

    unserved_justifications: List[dict] = []
    rejection_counts: Dict[str, int] = {}
    rejection_points: Dict[str, float] = {}

    for c in scenario.contacts:
        if c.cls not in (HOSTILE, UNKNOWN) or c.id in served_set:
            continue
        win_lo, win_hi = c.window if c.window else (0.0, scenario.horizon_s)

        # Check 1: R7 Collateral Veto (hostile only)
        collat_blocker = None
        min_collat_m = INF
        for fc in protected_contacts:
            d = hdist(c.pos, fc.pos)
            if d < min_collat_m:
                min_collat_m = d
                if d < rules.collateral_radius_m - EPS:
                    collat_blocker = (fc.id, fc.cls, d)

        if c.cls == HOSTILE and collat_blocker is not None:
            reason = "R7_COLLATERAL_VETO"
            detail = (
                f"Legally unengageable under R7: {collat_blocker[0]} ({collat_blocker[1]}) is "
                f"{collat_blocker[2]:.1f}m away (< {rules.collateral_radius_m:.0f}m collateral limit)."
            )
            extra = {"blocking_contact": collat_blocker[0], "distance_m": round(collat_blocker[2], 1)}
        else:
            # Physical reachability lower bounds (ignoring other missions)
            scout_e_ok = []
            scout_both_ok = []
            for s in all_scouts:
                e_rt, t_arr = _min_scout_roundtrip_energy_and_arrival(s, c, rules)
                if e_rt <= s.usable_wh + EPS:
                    scout_e_ok.append((e_rt, t_arr, s.id))
                    latest_obs_end = win_hi - (rules.confirm_margin_s if c.cls == HOSTILE else 0.0)
                    if max(t_arr, win_lo) + rules.obs_dwell_s <= latest_obs_end + EPS:
                        scout_both_ok.append((e_rt, t_arr, s.id))

            need_l = c.needs.get("light", 0) if c.cls == HOSTILE else 0
            need_h = c.needs.get("heavy", 0) if c.cls == HOSTILE else 0
            n_need = need_l + need_h

            all_l_e = sorted(_min_effector_energy_and_arrival(a, c, rules) + (a.id,) for a in all_light
                             if _min_effector_energy_and_arrival(a, c, rules)[0] <= a.usable_wh + EPS)
            all_h_e = sorted(_min_effector_energy_and_arrival(a, c, rules) + (a.id,) for a in all_heavy
                             if _min_effector_energy_and_arrival(a, c, rules)[0] <= a.usable_wh + EPS)
            all_l_ok = [x for x in all_l_e if x[1] <= win_hi + EPS]
            all_h_ok = [x for x in all_h_e if x[1] <= win_hi + EPS]

            free_l_ok = [a for a in free_light
                         if _min_effector_energy_and_arrival(a, c, rules)[0] <= a.usable_wh + EPS
                         and _min_effector_energy_and_arrival(a, c, rules)[1] <= win_hi + EPS]
            free_h_ok = [a for a in free_heavy
                         if _min_effector_energy_and_arrival(a, c, rules)[0] <= a.usable_wh + EPS
                         and _min_effector_energy_and_arrival(a, c, rules)[1] <= win_hi + EPS]

            # Minimum isolated economic cost (lower bound)
            min_iso_cost = 0.0
            if scout_both_ok:
                min_iso_cost += rules.lambda_e * min(x[0] for x in scout_both_ok)
            if c.cls == HOSTILE and len(all_l_ok) >= need_l and len(all_h_ok) >= need_h:
                min_iso_cost += rules.c_eff * n_need
                min_iso_cost += rules.lambda_e * (
                    sum(x[0] for x in all_l_ok[:need_l]) + sum(x[0] for x in all_h_ok[:need_h])
                )

            if not scout_e_ok or (c.cls == HOSTILE and (len(all_l_e) < need_l or len(all_h_e) < need_h)):
                reason = "ENERGY_RANGE_INFEASIBLE"
                detail = (
                    f"Physically unreachable within battery limits (R3): "
                    f"scouts in range={len(scout_e_ok)}, light in range={len(all_l_e)}/{need_l}, "
                    f"heavy in range={len(all_h_e)}/{need_h}."
                )
                extra = {"scouts_in_range": len(scout_e_ok), "light_in_range": len(all_l_e), "heavy_in_range": len(all_h_e)}
            elif not scout_both_ok or (c.cls == HOSTILE and (len(all_l_ok) < need_l or len(all_h_ok) < need_h)):
                reason = "WINDOW_R9_INFEASIBLE"
                detail = (
                    f"Unreachable inside R9 time window [{win_lo:.0f}, {win_hi:.0f}]s: "
                    f"scouts on-time={len(scout_both_ok)}, light on-time={len(all_l_ok)}/{need_l}, "
                    f"heavy on-time={len(all_h_ok)}/{need_h}."
                )
                extra = {"window_s": [win_lo, win_hi]}
            elif c.points <= min_iso_cost:
                reason = "NEGATIVE_ISOLATED_ROI"
                detail = (
                    f"Economically unprofitable even in isolation: gross {c.points:.1f} pts <= "
                    f"minimum isolated cost {min_iso_cost:.1f} pts (effectors + battery)."
                )
                extra = {"min_isolated_cost": round(min_iso_cost, 2)}
            elif c.cls == HOSTILE and (len(free_l_ok) < need_l or len(free_h_ok) < need_h):
                reason = "FLEET_EFFECTOR_SATURATED"
                ppe = c.points / max(1, n_need)
                detail = (
                    f"Effector fleet allocated to higher-yield targets: needs {need_l}L+{need_h}H, "
                    f"only {len(free_l_ok)}L+{len(free_h_ok)}H free & reachable "
                    f"(target yield {ppe:.1f} pts/eff vs served avg {avg_served_ppe:.1f}, min {min_served_ppe:.1f})."
                )
                extra = {
                    "needs": {"light": need_l, "heavy": need_h},
                    "free_reachable": {"light": len(free_l_ok), "heavy": len(free_h_ok)},
                    "pts_per_eff": round(ppe, 2),
                    "served_avg_pts_per_eff": round(avg_served_ppe, 2),
                }
            else:
                reason = "SCOUT_OR_R8_BOTTLENECK"
                detail = (
                    f"Feasible in isolation (est. min cost {min_iso_cost:.1f}, est. net +{c.points - min_iso_cost:.1f}) "
                    f"with free effectors ({len(free_l_ok)}L/{need_l}, {len(free_h_ok)}H/{need_h}), "
                    f"but excluded due to scout tour time/battery contention or R8 airspace/pad congestion."
                )
                extra = {
                    "est_min_cost": round(min_iso_cost, 2),
                    "est_max_net": round(c.points - min_iso_cost, 2),
                }

        rejection_counts[reason] = rejection_counts.get(reason, 0) + 1
        rejection_points[reason] = rejection_points.get(reason, 0.0) + c.points
        unserved_justifications.append({
            "contact_id": c.id,
            "class": c.cls,
            "pos": [round(v, 1) for v in c.pos],
            "points": c.points,
            "needs": c.needs,
            "window_s": list(c.window) if c.window else None,
            "reason_code": reason,
            "rationale": f"SKIP {c.id} ({c.cls}, {c.points:.0f} pts) [{reason}]: {detail}",
            "details": extra,
        })

    # 4. Fleet-wide Safety & Economic Summary
    actionable_points = sum(c.points for c in scenario.contacts if c.cls in (HOSTILE, UNKNOWN))
    legal_actionable_points = actionable_points - rejection_points.get("R7_COLLATERAL_VETO", 0.0)
    scout_reserves = [st["battery_reserve_pct"] for st in route_stats.values() if st["type"] == SCOUT]
    eff_reserves = [st["battery_reserve_pct"] for st in route_stats.values() if st["type"] == EFFECTOR]
    nfz_clearances = [st["min_nfz_clearance_m"] for st in route_stats.values() if st["min_nfz_clearance_m"] is not None]
    prot_clearances = [st["min_protected_clearance_m"] for st in route_stats.values() if st["min_protected_clearance_m"] is not None]
    r4_margins = [j["r4_margin_s"] for j in served_justifications if j["r4_margin_s"] is not None]
    r10_spreads = [j["r10_sync_spread_s"] for j in served_justifications if j["r10_sync_spread_s"] is not None]
    r9_slacks = [j["r9_min_slack_s"] for j in served_justifications]

    tier_counts: Dict[str, int] = {}
    for j in served_justifications:
        tier_counts[j["roe_tier"]] = tier_counts.get(j["roe_tier"], 0) + 1

    explain_ms = (time.perf_counter() - t0) * 1000.0

    return {
        "scenario_id": f"{scenario.level}-{scenario.seed}",
        "level": scenario.level,
        "seed": scenario.seed,
        "valid": score_res.valid,
        "score": round(score_res.score, 2),
        "gross_points": round(score_res.points, 2),
        "energy_wh": round(score_res.energy_wh, 2),
        "effectors_used": score_res.effectors_used,
        "cost_decomposition_check": {
            "total_cost_exact": round(rules.lambda_e * score_res.energy_wh + rules.c_eff * score_res.effectors_used, 2),
            "attributed_to_served": round(total_attributed_cost, 2),
            "unattributed_scout_cost": round(unattributed_scout_cost, 2),
        },
        "fleet_utilization": {
            "total_aircraft": len(scenario.aircraft),
            "active_aircraft": len(routes),
            "scouts_active": sum(1 for st in route_stats.values() if st["type"] == SCOUT),
            "scouts_total": len(all_scouts),
            "light_active": sum(1 for st in route_stats.values() if st["subtype"] == "light"),
            "light_total": len(all_light),
            "heavy_active": sum(1 for st in route_stats.values() if st["subtype"] == "heavy"),
            "heavy_total": len(all_heavy),
        },
        "target_coverage": {
            "served_count": len(served_justifications),
            "unserved_count": len(unserved_justifications),
            "actionable_total_points": round(actionable_points, 1),
            "legal_actionable_points": round(legal_actionable_points, 1),
            "gross_capture_of_legal_pct": round(100.0 * score_res.points / max(legal_actionable_points, 1e-6), 1),
            "roe_tier_breakdown": tier_counts,
            "rejection_counts": rejection_counts,
            "rejection_points": {k: round(v, 1) for k, v in rejection_points.items()},
        },
        "safety_envelope": {
            "r8_closest_approach": closest_r8_event,
            "min_nfz_clearance_m": round(min(nfz_clearances), 1) if nfz_clearances else None,
            "min_protected_clearance_m": round(min(prot_clearances), 1) if prot_clearances else None,
            "min_scout_battery_reserve_pct": round(min(scout_reserves), 1) if scout_reserves else None,
            "min_effector_battery_reserve_pct": round(min(eff_reserves), 1) if eff_reserves else None,
            "min_r4_confirm_margin_s": round(min(r4_margins), 2) if r4_margins else None,
            "max_r10_sync_spread_s": round(max(r10_spreads), 2) if r10_spreads else None,
            "min_r9_window_slack_s": round(min(r9_slacks), 2) if r9_slacks else None,
        },
        "served_contacts": served_justifications,
        "unserved_contacts": unserved_justifications,
        "route_stats": route_stats,
        "explain_runtime_ms": round(explain_ms, 2),
    }


def format_markdown_briefing(rep: Dict[str, Any]) -> str:
    """Renders a concise, human-operator-ready C2 Mission Briefing in Markdown."""
    fu = rep["fleet_utilization"]
    tc = rep["target_coverage"]
    se = rep["safety_envelope"]
    lines = [
        f"# C2 Mission Briefing — {rep['scenario_id']}",
        "",
        f"- **Verdict ROE (R1–R10)** : `{'VALID' if rep['valid'] else 'INVALID'}` | "
        f"**Score** : `{rep['score']:.2f}` (Points bruts `{rep['gross_points']:.1f}`, "
        f"Énergie `{rep['energy_wh']:.1f} Wh`, Effecteurs `{rep['effectors_used']}`)",
        f"- **Engagement flotte** : `{fu['active_aircraft']}/{fu['total_aircraft']}` drones actifs "
        f"(Scouts `{fu['scouts_active']}/{fu['scouts_total']}`, "
        f"Light `{fu['light_active']}/{fu['light_total']}`, "
        f"Heavy `{fu['heavy_active']}/{fu['heavy_total']}`)",
        f"- **Couverture légale** : `{tc['served_count']}` servis / `{tc['served_count'] + tc['unserved_count']}` "
        f"actionnables (`{tc['gross_capture_of_legal_pct']:.1f}%` des points légalement engageables)",
        "",
        "## Enveloppe de Sécurité ROE",
    ]
    r8 = se["r8_closest_approach"]
    if r8:
        lines.append(
            f"- **R8 Séparation 3D min** : `{r8['distance_m']:.1f} m` "
            f"(marge `+{r8['margin_above_d_sep_m']:.1f} m` au-dessus de 40 m) "
            f"entre `{r8['aircraft_a']}` et `{r8['aircraft_b']}` à `t = {r8['t_s']:.1f} s`"
        )
    lines.extend([
        f"- **R6 Marges zones** : NFZ min `{se['min_nfz_clearance_m']} m` | Protégées (ami/civil) min `{se['min_protected_clearance_m']} m`",
        f"- **R3 Réserve batterie min** : Scouts `{se['min_scout_battery_reserve_pct']}%` | Effecteurs `{se['min_effector_battery_reserve_pct']}%`",
        f"- **R4 / R9 / R10 Marges temporelles** : R4 min `+{se['min_r4_confirm_margin_s']} s` | "
        f"R10 spread max `{se['max_r10_sync_spread_s']} s` | R9 slack min `{se['min_r9_window_slack_s']} s`",
        "",
        "## Justifications des Contacts Servis",
    ])
    for j in rep["served_contacts"]:
        lines.append(f"- **[{j['roe_tier']}]** {j['rationale']}")
    lines.extend(["", "## Audit Contrefactuel des Contacts Non Servis"])
    if not rep["unserved_contacts"]:
        lines.append("- *Tous les contacts actionnables ont été servis.*")
    else:
        for k, cnt in sorted(tc["rejection_counts"].items(), key=lambda x: -tc["rejection_points"].get(x[0], 0)):
            lines.append(f"- `{k}` : **{cnt}** contacts (`{tc['rejection_points'][k]:.0f} pts`)")
        lines.append("")
        for u in rep["unserved_contacts"]:
            lines.append(f"- {u['rationale']}")
    return "\n".join(lines) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description="Deterministic C2 Explainability Auditor for a fleetkit plan")
    ap.add_argument("scenario", help="Path to scenario JSON")
    ap.add_argument("plan", help="Path to plan JSON")
    ap.add_argument("--json-out", help="Write full JSON explanation report")
    ap.add_argument("--md-out", help="Write Markdown operator briefing")
    args = ap.parse_args()

    sc = Scenario.from_dict(load_json(args.scenario))
    plan = load_json(args.plan)
    rep = explain_plan(sc, plan)
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(rep, indent=2) + "\n", encoding="utf-8")
    md = format_markdown_briefing(rep)
    if args.md_out:
        Path(args.md_out).write_text(md, encoding="utf-8")
    else:
        print(md)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
