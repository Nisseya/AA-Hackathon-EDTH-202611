"""Exact convex speed/timing optimization for scouts and effectors."""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

from fleetkit.energy import best_range_speed, route_energy_wh
from fleetkit.model import Aircraft, Contact, Scenario, Waypoint

from geometry3d import INF, ObstacleMap, Point2, Point3

TIME_PAD = 2e-3
WIN_PAD = 0.05
HOSTILE_ENG_PAD = 0.25


def _subleg_min_dt(a: Point3, b: Point3, ac: Aircraft) -> float:
    dh = math.hypot(b[0] - a[0], b[1] - a[1])
    dz = b[2] - a[2]
    need = dh / ac.v_h_mps
    if dz > 0.0:
        need = max(need, dz / ac.v_up_mps)
    elif dz < 0.0:
        need = max(need, -dz / ac.v_down_mps)
    return need + TIME_PAD if (dh > 1e-9 or abs(dz) > 1e-9) else 0.0


def _macro_leg_info(pts: Sequence[Point3], ac: Aircraft) -> Tuple[float, float, float, List[float], List[float]]:
    """Computes (total_dh, min_dt, ideal_dt, sub_dhs, sub_min_dts) for a 3D polyline."""
    v_opt = min(ac.v_h_mps, best_range_speed(ac.power))
    sub_dhs: List[float] = []
    sub_min: List[float] = []
    total_dh = 0.0
    total_min = 0.0
    for i in range(len(pts) - 1):
        dh = math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        dt_m = _subleg_min_dt(pts[i], pts[i + 1], ac)
        sub_dhs.append(dh)
        sub_min.append(dt_m)
        total_dh += dh
        total_min += dt_m
    ideal_dt = max(total_min, total_dh / v_opt if (v_opt > 1e-9 and total_dh > 1e-9) else total_min)
    return total_dh, total_min, ideal_dt, sub_dhs, sub_min


def _allocate_subleg_times(
    t_start: float,
    total_dt: float,
    sub_dhs: Sequence[float],
    sub_min: Sequence[float],
) -> List[float]:
    """Allocates total_dt across sub-legs respecting sub_min and minimizing cruise energy."""
    m = len(sub_dhs)
    if m == 0:
        return []
    if m == 1:
        return [t_start + total_dt]
    dts = list(sub_min)
    extra = max(0.0, total_dt - sum(dts))
    if extra > 1e-9:
        total_dh = sum(sub_dhs)
        if total_dh > 1e-9:
            fixed = [False] * m
            for _ in range(m):
                rem_t = total_dt - sum(dts[i] for i in range(m) if fixed[i])
                rem_d = sum(sub_dhs[i] for i in range(m) if not fixed[i])
                if rem_d <= 1e-9 or rem_t <= 1e-9:
                    break
                changed = False
                for i in range(m):
                    if not fixed[i]:
                        cand = rem_t * (sub_dhs[i] / rem_d)
                        if cand < sub_min[i]:
                            dts[i] = sub_min[i]
                            fixed[i] = True
                            changed = True
                if not changed:
                    for i in range(m):
                        if not fixed[i]:
                            dts[i] = rem_t * (sub_dhs[i] / rem_d)
                    break
            else:
                for i in range(m):
                    dts[i] += extra / m
        else:
            for i in range(m):
                dts[i] += extra / m
    out: List[float] = []
    cur = t_start
    for i in range(m - 1):
        cur += dts[i]
        out.append(cur)
    out.append(t_start + total_dt)
    return out


def optimize_scout_timing(
    ac: Aircraft,
    leg_dhs: Sequence[float],
    leg_min_dts: Sequence[float],
    leg_ideal_dts: Sequence[float],
    obs_windows: Sequence[Tuple[float, float]],
    dwell_s: float,
    horizon_s: float,
    min_t0: float = 0.0,
    max_t_land: Optional[float] = None,
) -> Optional[Tuple[float, List[float], float]]:
    """Exact convex timing optimization for a scout tour with k observations."""
    k = len(obs_windows)
    if k == 0:
        return None
    t_lo_bound = max(0.0, min_t0)
    t_hi_bound = horizon_s if max_t_land is None else min(horizon_s, max_t_land)
    if t_lo_bound >= t_hi_bound:
        return None

    # 1. Earliest feasible start times (forward pass)
    e = [0.0] * k
    e[0] = max(obs_windows[0][0], t_lo_bound + leg_min_dts[0])
    if e[0] > obs_windows[0][1] + 1e-9:
        return None
    for j in range(1, k):
        e[j] = max(obs_windows[j][0], e[j - 1] + dwell_s + leg_min_dts[j])
        if e[j] > obs_windows[j][1] + 1e-9:
            return None
    if e[k - 1] + dwell_s + leg_min_dts[k] > t_hi_bound + 1e-9:
        return None

    # 2. Latest feasible start times (backward pass)
    l = [0.0] * k
    l[k - 1] = min(obs_windows[k - 1][1], t_hi_bound - leg_min_dts[k] - dwell_s)
    if l[k - 1] < e[k - 1] - 1e-9:
        return None
    for j in range(k - 2, -1, -1):
        l[j] = min(obs_windows[j][1], l[j + 1] - dwell_s - leg_min_dts[j + 1])
        if l[j] < e[j] - 1e-9:
            return None

    # 3. Smart initialization
    s = [0.0] * k
    s[0] = min(l[0], max(e[0], t_lo_bound + leg_ideal_dts[0]))
    for j in range(1, k):
        s[j] = min(l[j], max(e[j], s[j - 1] + dwell_s + leg_ideal_dts[j]))
    for j in range(k - 2, -1, -1):
        target = s[j + 1] - dwell_s - leg_ideal_dts[j + 1]
        if target > s[j]:
            s[j] = min(l[j], target)

    t0 = max(t_lo_bound, s[0] - leg_ideal_dts[0])
    if s[0] - t0 < leg_min_dts[0]:
        t0 = max(t_lo_bound, s[0] - leg_min_dts[0])
    t_land = min(t_hi_bound, s[k - 1] + dwell_s + leg_ideal_dts[k])
    if t_land - (s[k - 1] + dwell_s) < leg_min_dts[k]:
        t_land = min(t_hi_bound, s[k - 1] + dwell_s + leg_min_dts[k])

    # Fast early-exit: if every leg is already at ideal_dt, initialization is globally optimal!
    if (
        abs((s[0] - t0) - leg_ideal_dts[0]) < 1e-4
        and abs((t_land - (s[k - 1] + dwell_s)) - leg_ideal_dts[k]) < 1e-4
        and all(abs((s[j] - (s[j - 1] + dwell_s)) - leg_ideal_dts[j]) < 1e-4 for j in range(1, k))
    ):
        return t0, s, t_land

    # 4. Exact coordinate descent on strictly convex energy
    for _ in range(16):
        max_delta = 0.0
        new_t0 = max(t_lo_bound, s[0] - leg_ideal_dts[0])
        if s[0] - new_t0 < leg_min_dts[0]:
            new_t0 = s[0] - leg_min_dts[0]
        new_t0 = max(t_lo_bound, new_t0)
        max_delta = max(max_delta, abs(new_t0 - t0))
        t0 = new_t0

        for j in range(k):
            lo_j = max(obs_windows[j][0], (t0 if j == 0 else s[j - 1] + dwell_s) + leg_min_dts[j])
            hi_j = min(obs_windows[j][1], (t_land if j == k - 1 else s[j + 1]) - dwell_s - leg_min_dts[j + 1])
            if lo_j > hi_j:
                lo_j = hi_j = 0.5 * (lo_j + hi_j)

            prev_dep = t0 if j == 0 else (s[j - 1] + dwell_s)
            next_arr = t_land if j == k - 1 else s[j + 1]

            if j == 0 and k > 1 and s[0] - leg_ideal_dts[0] > t_lo_bound + 1e-6:
                opt_s = max(t_lo_bound + leg_ideal_dts[0], s[1] - dwell_s - leg_ideal_dts[1])
                new_s = min(hi_j, max(lo_j, opt_s))
            elif j == k - 1 and k > 1 and s[k - 1] + dwell_s + leg_ideal_dts[k] < t_hi_bound - 1e-6:
                opt_s = min(t_hi_bound - dwell_s - leg_ideal_dts[k], s[k - 2] + dwell_s + leg_ideal_dts[k - 1])
                new_s = min(hi_j, max(lo_j, opt_s))
            elif k == 1:
                opt_lo = max(obs_windows[0][0], t_lo_bound + leg_ideal_dts[0])
                opt_hi = min(obs_windows[0][1], t_hi_bound - dwell_s - leg_ideal_dts[1])
                if opt_lo <= opt_hi:
                    new_s = opt_lo
                else:
                    total_avail = (t_hi_bound - t_lo_bound) - dwell_s
                    d_sum = leg_dhs[0] + leg_dhs[1]
                    opt_s = t_lo_bound + (total_avail * (leg_dhs[0] / d_sum) if d_sum > 1e-9 else 0.5 * total_avail)
                    new_s = min(hi_j, max(lo_j, opt_s))
            else:
                avail = (next_arr - dwell_s) - prev_dep
                d_prev = leg_dhs[j]
                d_next = leg_dhs[j + 1]
                if d_prev + d_next > 1e-9:
                    opt_s = prev_dep + avail * (d_prev / (d_prev + d_next))
                else:
                    opt_s = 0.5 * (lo_j + hi_j)
                new_s = min(hi_j, max(lo_j, opt_s))

            max_delta = max(max_delta, abs(new_s - s[j]))
            s[j] = new_s

            if j == 0:
                t0 = max(t_lo_bound, s[0] - leg_ideal_dts[0])
                if s[0] - t0 < leg_min_dts[0]:
                    t0 = max(t_lo_bound, s[0] - leg_min_dts[0])

        new_land = min(t_hi_bound, s[k - 1] + dwell_s + leg_ideal_dts[k])
        if new_land - (s[k - 1] + dwell_s) < leg_min_dts[k]:
            new_land = min(t_hi_bound, s[k - 1] + dwell_s + leg_min_dts[k])
        max_delta = max(max_delta, abs(new_land - t_land))
        t_land = new_land

        if max_delta < 1e-4:
            break

    return t0, s, t_land


def build_scout_route(
    ac: Aircraft,
    contacts: Sequence[Contact],
    obs_points: Sequence[Point2],
    z: float,
    obs_map: ObstacleMap,
    scenario: Scenario,
    max_obs_end_overrides: Optional[Dict[str, float]] = None,
    time_shift: float = 0.0,
    min_t0: float = 0.0,
    max_t_land: Optional[float] = None,
) -> Optional[Tuple[List[dict], float, Dict[str, float]]]:
    """Builds a fully valid, minimum-energy scout route for ordered contacts."""
    if not contacts:
        return None
    r = scenario.rules
    dwell = r.obs_dwell_s

    def _make_windows(use_stagger: bool) -> Optional[List[Tuple[float, float]]]:
        ws: List[Tuple[float, float]] = []
        for c in contacts:
            lo, hi = c.window
            a = lo + WIN_PAD
            if c.cls == "hostile":
                if use_stagger:
                    n_eff = max(1, sum(c.needs.values()))
                    avail = max(0.0, hi - lo - dwell - r.confirm_margin_s)
                    eng_buf = min(1.5 + 1.5 * (n_eff - 1), max(HOSTILE_ENG_PAD, 0.05 * avail))
                else:
                    eng_buf = HOSTILE_ENG_PAD
                b = hi - dwell - r.confirm_margin_s - eng_buf
            else:
                b = hi - dwell - WIN_PAD
            if max_obs_end_overrides and c.id in max_obs_end_overrides:
                b = min(b, max_obs_end_overrides[c.id] - dwell - WIN_PAD)
            if a > b:
                a = lo + 0.01
                b = hi - dwell - (r.confirm_margin_s + HOSTILE_ENG_PAD if c.cls == "hostile" else 0.01)
                if max_obs_end_overrides and c.id in max_obs_end_overrides:
                    b = min(b, max_obs_end_overrides[c.id] - dwell - 0.01)
                if a > b:
                    return None
            ws.append((a, b))
        return ws

    windows = _make_windows(use_stagger=True)
    if windows is None:
        return None

    nodes: List[Point3] = [ac.home] + [(p[0], p[1], z) for p in obs_points] + [ac.home]
    macro_pts: List[List[Point3]] = []
    leg_dhs: List[float] = []
    leg_min: List[float] = []
    leg_ideal: List[float] = []
    sub_dhs_list: List[List[float]] = []
    sub_min_list: List[List[float]] = []

    for i in range(len(nodes) - 1):
        p3 = obs_map.best_path_3d(nodes[i], nodes[i + 1], ac, z_cruise=z)
        if p3 is None:
            return None
        dh, dt_min, dt_id, sdhs, smins = _macro_leg_info(p3, ac)
        macro_pts.append(p3)
        leg_dhs.append(dh)
        leg_min.append(dt_min)
        leg_ideal.append(dt_id)
        sub_dhs_list.append(sdhs)
        sub_min_list.append(smins)

    timing = optimize_scout_timing(
        ac, leg_dhs, leg_min, leg_ideal, windows, dwell, scenario.horizon_s,
        min_t0=min_t0, max_t_land=max_t_land,
    )
    if timing is None:
        windows_tight = _make_windows(use_stagger=False)
        if windows_tight is None:
            return None
        timing = optimize_scout_timing(
            ac, leg_dhs, leg_min, leg_ideal, windows_tight, dwell, scenario.horizon_s,
            min_t0=min_t0, max_t_land=max_t_land,
        )
        if timing is None:
            return None
        windows = windows_tight
    t0, s_times, t_land = timing

    if abs(time_shift) > 1e-6:
        t0_s = t0 + time_shift
        t_land_s = t_land + time_shift
        s_s = [st + time_shift for st in s_times]
        if t0_s < 0.0 or t_land_s > scenario.horizon_s:
            return None
        for j, (a_w, b_w) in enumerate(windows):
            if s_s[j] < a_w - 1e-6 or s_s[j] > b_w + 1e-6:
                return None
        t0, s_times, t_land = t0_s, s_s, t_land_s

    t0_r = max(0.0, round(t0, 4))
    s_r = [round(st, 4) for st in s_times]
    for j in range(len(s_r)):
        prev_dep = t0_r if j == 0 else round(s_r[j - 1] + dwell, 4)
        if s_r[j] < prev_dep:
            s_r[j] = prev_dep
    t_land_r = min(scenario.horizon_s, max(round(s_r[-1] + dwell, 4), round(t_land, 4)))

    wps: List[dict] = [_wp_dict(t0_r, ac.home)]
    obs_ends: Dict[str, float] = {}

    for j in range(len(contacts)):
        t_dep = t0_r if j == 0 else round(s_r[j - 1] + dwell, 4)
        t_arr = s_r[j]
        p3 = macro_pts[j]
        sub_ts = _allocate_subleg_times(t_dep, t_arr - t_dep, sub_dhs_list[j], sub_min_list[j])
        for m_idx in range(1, len(p3) - 1):
            wps.append(_wp_dict(sub_ts[m_idx - 1], p3[m_idx]))
        wps.append(_wp_dict(t_arr, p3[-1], action="observe", target=contacts[j].id, dwell_s=dwell))
        obs_ends[contacts[j].id] = round(t_arr + dwell, 4)

    t_dep = round(s_r[-1] + dwell, 4)
    p3 = macro_pts[-1]
    sub_ts = _allocate_subleg_times(t_dep, t_land_r - t_dep, sub_dhs_list[-1], sub_min_list[-1])
    for m_idx in range(1, len(p3) - 1):
        wps.append(_wp_dict(sub_ts[m_idx - 1], p3[m_idx]))
    wps.append(_wp_dict(t_land_r, ac.home))

    parsed_wps = _to_waypoints(wps)
    energy = route_energy_wh(parsed_wps, ac)
    if energy > ac.usable_wh - 1e-3:
        return None
    if not _check_r2_fast(parsed_wps, ac, r.z_min_m, r.z_max_m, scenario.horizon_s):
        return None
    return wps, energy, obs_ends




def eval_scout_route_energy(
    ac: Aircraft,
    contacts: Sequence[Contact],
    obs_points: Sequence[Point2],
    z: float,
    obs_map: ObstacleMap,
    scenario: Scenario,
) -> Optional[float]:
    """Fast exact scout route energy evaluator for VRPTW search (skips JSON & Waypoint allocations)."""
    k = len(contacts)
    if k == 0:
        return 0.0
    r = scenario.rules
    dwell = r.obs_dwell_s

    def _make_win(use_stagger: bool) -> Optional[List[Tuple[float, float]]]:
        ws: List[Tuple[float, float]] = []
        for c in contacts:
            lo, hi = c.window
            a = lo + WIN_PAD
            if c.cls == "hostile":
                if use_stagger:
                    n_eff = max(1, sum(c.needs.values()))
                    avail = max(0.0, hi - lo - dwell - r.confirm_margin_s)
                    eng_buf = min(1.5 + 1.5 * (n_eff - 1), max(HOSTILE_ENG_PAD, 0.05 * avail))
                else:
                    eng_buf = HOSTILE_ENG_PAD
                b = hi - dwell - r.confirm_margin_s - eng_buf
            else:
                b = hi - dwell - WIN_PAD
            if a > b:
                a = lo + 0.01
                b = hi - dwell - (r.confirm_margin_s + HOSTILE_ENG_PAD if c.cls == "hostile" else 0.01)
                if a > b:
                    return None
            ws.append((a, b))
        return ws

    windows = _make_win(True)
    if windows is None:
        return None

    home = ac.home
    v_opt = min(ac.v_h_mps, best_range_speed(ac.power))
    inv_vh = 1.0 / ac.v_h_mps
    inv_vup = 1.0 / ac.v_up_mps
    inv_vdn = 1.0 / ac.v_down_mps
    inv_vopt = (1.0 / v_opt) if v_opt > 1e-9 else inv_vh

    leg_dhs: List[float] = []
    leg_min: List[float] = []
    leg_ideal: List[float] = []
    sub_dhs_list: List[Optional[List[float]]] = []
    sub_min_list: List[Optional[List[float]]] = []
    climb_m = 0.0

    prev_pt: Point3 = home
    for i in range(k + 1):
        nxt_pt: Point3 = home if i == k else (obs_points[i][0], obs_points[i][1], z)
        p3 = obs_map.best_path_3d(prev_pt, nxt_pt, ac, z_cruise=z)
        if p3 is None:
            return None
        if len(p3) == 2:
            dh = math.hypot(nxt_pt[0] - prev_pt[0], nxt_pt[1] - prev_pt[1])
            dz = nxt_pt[2] - prev_pt[2]
            need = dh * inv_vh
            if dz > 0.0:
                climb_m += dz
                t_z = dz * inv_vup
                if t_z > need:
                    need = t_z
            elif dz < 0.0:
                t_z = -dz * inv_vdn
                if t_z > need:
                    need = t_z
            dt_m = need + TIME_PAD if (dh > 1e-9 or abs(dz) > 1e-9) else 0.0
            dt_id = dh * inv_vopt
            if dt_id < dt_m:
                dt_id = dt_m
            leg_dhs.append(dh)
            leg_min.append(dt_m)
            leg_ideal.append(dt_id)
            sub_dhs_list.append(None)
            sub_min_list.append(None)
        else:
            sdhs: List[float] = []
            smins: List[float] = []
            tdh = 0.0
            tmin = 0.0
            for j in range(len(p3) - 1):
                a3, b3 = p3[j], p3[j + 1]
                dh = math.hypot(b3[0] - a3[0], b3[1] - a3[1])
                dz = b3[2] - a3[2]
                need = dh * inv_vh
                if dz > 0.0:
                    climb_m += dz
                    t_z = dz * inv_vup
                    if t_z > need:
                        need = t_z
                elif dz < 0.0:
                    t_z = -dz * inv_vdn
                    if t_z > need:
                        need = t_z
                dt_m = need + TIME_PAD if (dh > 1e-9 or abs(dz) > 1e-9) else 0.0
                sdhs.append(dh)
                smins.append(dt_m)
                tdh += dh
                tmin += dt_m
            dt_id = tdh * inv_vopt
            if dt_id < tmin:
                dt_id = tmin
            leg_dhs.append(tdh)
            leg_min.append(tmin)
            leg_ideal.append(dt_id)
            sub_dhs_list.append(sdhs)
            sub_min_list.append(smins)
        prev_pt = nxt_pt

    timing = optimize_scout_timing(ac, leg_dhs, leg_min, leg_ideal, windows, dwell, scenario.horizon_s)
    if timing is None:
        windows_tight = _make_win(False)
        if windows_tight is None:
            return None
        timing = optimize_scout_timing(ac, leg_dhs, leg_min, leg_ideal, windows_tight, dwell, scenario.horizon_s)
        if timing is None:
            return None
    t0, s_times, t_land = timing

    pw = ac.power
    p0_w = pw.p0_w
    kv = pw.k_v
    inv_3600 = 1.0 / 3600.0
    # Hover dwell energy + total climb energy
    energy = (k * dwell * p0_w) * inv_3600 + climb_m * pw.e_climb_wh_per_m

    for i in range(k + 1):
        t_dep = t0 if i == 0 else (s_times[i - 1] + dwell)
        t_arr = t_land if i == k else s_times[i]
        dt = t_arr - t_dep
        if dt <= 1e-9:
            continue
        sdhs = sub_dhs_list[i]
        if sdhs is None:
            dh = leg_dhs[i]
            v = dh / dt
            energy += (p0_w + kv * v * v) * dt * inv_3600
        else:
            smins = sub_min_list[i]
            sub_ts = _allocate_subleg_times(t_dep, dt, sdhs, smins)
            cur = t_dep
            for j, st in enumerate(sub_ts):
                sdt = st - cur
                if sdt > 1e-9:
                    v = sdhs[j] / sdt
                    energy += (p0_w + kv * v * v) * sdt * inv_3600
                cur = st

    if energy > ac.usable_wh - 1e-3:
        return None
    return energy


def build_effector_route(
    ac: Aircraft,
    contact: Contact,
    arrive_t: float,
    obs_map: ObstacleMap,
    scenario: Scenario,
    z_cruise: float = 0.0,
    angle_deg: float = 0.0,
    extra_duration: float = 0.0,
) -> Optional[Tuple[List[dict], float, float]]:
    """Builds a valid minimum-energy effector route arriving at `arrive_t`."""
    r = scenario.rules
    lo, hi = contact.window
    if arrive_t < lo - 1e-6 or arrive_t > hi + 1e-6:
        return None

    e_z = 0.0
    e_pt = obs_map.engage_point(ac.home[:2], contact, z=e_z, angle_deg=angle_deg)

    if z_cruise <= 1e-3:
        pts3 = obs_map.best_path_3d(ac.home, e_pt, ac, z_cruise=r.z_min_m)
    else:
        zc = max(r.z_min_m, min(r.z_max_m, z_cruise))
        vx, vy = e_pt[0] - ac.home[0], e_pt[1] - ac.home[1]
        d = math.hypot(vx, vy)
        climb_dist = max(zc / ac.v_up_mps * ac.v_h_mps, 55.0)
        dive_dist = max(zc / ac.v_down_mps * ac.v_h_mps, 55.0)
        if d > climb_dist + dive_dist + 40.0:
            ux, uy = vx / d, vy / d
            c_pt: Point3 = (ac.home[0] + ux * climb_dist, ac.home[1] + uy * climb_dist, zc)
            a_pt: Point3 = (e_pt[0] - ux * dive_dist, e_pt[1] - uy * dive_dist, zc)
            if obs_map.segment_valid_3d(ac.home, c_pt) and obs_map.segment_valid_3d(a_pt, e_pt):
                p_mid = obs_map.best_path_3d(c_pt, a_pt, ac, z_cruise=zc)
                if p_mid is not None:
                    pts3 = [ac.home] + p_mid + [e_pt]
                else:
                    pts3 = obs_map.best_path_3d(ac.home, e_pt, ac, z_cruise=zc)
            else:
                pts3 = obs_map.best_path_3d(ac.home, e_pt, ac, z_cruise=zc)
        elif d > dive_dist + 40.0:
            ux, uy = vx / d, vy / d
            a_pt = (e_pt[0] - ux * dive_dist, e_pt[1] - uy * dive_dist, zc)
            p1 = obs_map.best_path_3d(ac.home, a_pt, ac, z_cruise=zc)
            if p1 is not None and obs_map.segment_valid_3d(a_pt, e_pt):
                pts3 = p1 + [e_pt]
            else:
                pts3 = obs_map.best_path_3d(ac.home, e_pt, ac, z_cruise=zc)
        else:
            pts3 = obs_map.best_path_3d(ac.home, e_pt, ac, z_cruise=zc)

    if pts3 is None:
        return None

    dh, min_dt, ideal_dt, sub_dhs, sub_min = _macro_leg_info(pts3, ac)
    earliest = min_dt
    if arrive_t < earliest - 1e-6:
        return None

    dur = min(arrive_t, max(min_dt, ideal_dt + extra_duration))
    t0 = arrive_t - dur
    if t0 < 0.0:
        t0 = 0.0
        dur = arrive_t
        if dur < min_dt - 1e-6:
            return None

    sub_ts = _allocate_subleg_times(t0, dur, sub_dhs, sub_min)
    wps: List[dict] = [_wp_dict(t0, ac.home)]
    for m_idx in range(1, len(pts3) - 1):
        wps.append(_wp_dict(sub_ts[m_idx - 1], pts3[m_idx]))
    wps.append(_wp_dict(arrive_t, pts3[-1], action="engage", target=contact.id))

    parsed_wps = _to_waypoints(wps)
    energy = route_energy_wh(parsed_wps, ac)
    if energy > ac.usable_wh - 1e-3:
        return None
    if not _check_r2_fast(parsed_wps, ac, r.z_min_m, r.z_max_m, scenario.horizon_s):
        return None
    return wps, energy, earliest


def effector_min_cost(
    ac: Aircraft,
    contact: Contact,
    obs_map: ObstacleMap,
    scenario: Scenario,
) -> Optional[Tuple[float, float]]:
    """Fast exact minimum energy (Wh) and earliest arrival (s) for effector -> contact."""
    e_pt = obs_map.engage_point(ac.home[:2], contact, z=0.0)
    pts3 = obs_map.best_path_3d(ac.home, e_pt, ac, z_cruise=scenario.rules.z_min_m)
    if pts3 is None:
        return None
    dh, min_dt, ideal_dt, sub_dhs, sub_min = _macro_leg_info(pts3, ac)
    lo, hi = contact.window
    if min_dt > hi - 1e-3:
        return None
    dur = min(hi, ideal_dt)
    if dur < min_dt:
        dur = min_dt
    sub_ts = _allocate_subleg_times(0.0, dur, sub_dhs, sub_min)
    wps: List[Waypoint] = [Waypoint(0.0, pts3[0][0], pts3[0][1], pts3[0][2])]
    for i in range(1, len(pts3)):
        wps.append(Waypoint(sub_ts[i - 1], pts3[i][0], pts3[i][1], pts3[i][2]))
    e = route_energy_wh(wps, ac)
    if e > ac.usable_wh - 1e-3:
        return None
    return e, min_dt


def _wp_dict(t: float, p: Point3, **extra) -> dict:
    d = {"t": round(t, 4), "x": round(p[0], 4), "y": round(p[1], 4), "z": round(p[2], 4)}
    d.update(extra)
    return d


def _to_waypoints(wps: Sequence[dict]) -> List[Waypoint]:
    return [
        Waypoint(
            float(w["t"]),
            float(w["x"]),
            float(w["y"]),
            float(w["z"]),
            w.get("action"),
            w.get("target"),
            float(w.get("dwell_s", 0.0)),
        )
        for w in wps
    ]


def _check_r2_fast(
    wps: Sequence[Waypoint],
    ac: Aircraft,
    z_min: float,
    z_max: float,
    horizon_s: Optional[float] = None,
) -> bool:
    if not wps or wps[0].t < -1e-7:
        return False
    if horizon_s is not None and wps[-1].depart > horizon_s + 1e-7:
        return False
    for k in range(len(wps) - 1):
        a, b = wps[k], wps[k + 1]
        dur = b.t - a.depart
        if dur < -1e-7:
            return False
        dh = math.hypot(b.x - a.x, b.y - a.y)
        dz = b.z - a.z
        need = dh / ac.v_h_mps
        if dz > 0.0:
            need = max(need, dz / ac.v_up_mps)
        elif dz < 0.0:
            need = max(need, -dz / ac.v_down_mps)
        if dur < need - 1e-7:
            return False
    for k in range(1, len(wps)):
        low = 0.0 if k == len(wps) - 1 else z_min
        if wps[k].z < low - 1e-6 or wps[k].z > z_max + 1e-6:
            return False
    return True
