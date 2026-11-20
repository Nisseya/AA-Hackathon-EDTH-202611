"""Ultra-fast AABB-pruned incremental R8 separation index and 3D deconfliction engine."""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

from fleetkit.energy import route_energy_wh
from fleetkit.geometry import (
    Piece,
    close_intervals,
    min_distance,
    position_at,
    within_radius_interval,
)
from fleetkit.model import EFFECTOR, Aircraft, Contact, Route, Scenario
from fleetkit.validator import route_pieces, validate

from geometry3d import ObstacleMap, Point3
from timing import _check_r2_fast, _to_waypoints, _wp_dict, build_effector_route, build_scout_route

SEP_SAFETY_M = 0.35


class _IndexedRoute:
    __slots__ = ("aid", "pieces", "aabbs", "t_start", "t_end", "target", "home", "t_depart")

    def __init__(self, aid: str, pieces: List[Piece], target: Optional[str], home: Point3):
        self.aid = aid
        self.pieces = pieces
        self.t_start = pieces[0][0]
        self.t_end = pieces[-1][1]
        self.target = target
        self.home = home
        self.t_depart = pieces[0][0]
        aabbs = []
        for t0, t1, p0, p1 in pieces:
            aabbs.append((
                t0,
                t1,
                min(p0[0], p1[0]),
                max(p0[0], p1[0]),
                min(p0[1], p1[1]),
                max(p0[1], p1[1]),
                min(p0[2], p1[2]),
                max(p0[2], p1[2]),
            ))
        self.aabbs = aabbs


class SeparationIndex:
    """Maintains all currently placed routes and performs fast exact R8 checks."""

    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.rules = scenario.rules
        self.d_sep = self.rules.d_sep_m + SEP_SAFETY_M
        self.contacts = scenario.contact_by_id()
        self.aircraft = scenario.aircraft_by_id()
        self.items: Dict[str, _IndexedRoute] = {}
        self.routes_json: Dict[str, List[dict]] = {}

    def _make_item(self, aid: str, wps_json: List[dict]) -> Optional[_IndexedRoute]:
        wps = _to_waypoints(wps_json)
        route = Route(aid, wps)
        pieces = route_pieces(route)
        if not pieces:
            return None
        ac = self.aircraft[aid]
        last = wps[-1]
        target = last.target if (last.action == "engage" and ac.type == EFFECTOR) else None
        return _IndexedRoute(aid, pieces, target, ac.home)

    def _pair_conflict(
        self, a: _IndexedRoute, b: _IndexedRoute
    ) -> Optional[Tuple[float, float, Point3, Point3]]:
        """Returns (t_conflict, min_dist, pos_a, pos_b) if a and b violate R8, else None."""
        if min(a.t_end, b.t_end) - max(a.t_start, b.t_start) <= 1e-12:
            return None
        d_sep = self.d_sep
        exempt_center = None
        if a.target is not None and a.target == b.target and a.target in self.contacts:
            exempt_center = self.contacts[a.target].pos
        exempt_r = self.rules.exempt_radius_m - 0.15

        pa, pb = a.pieces, b.pieces
        aa, ab = a.aabbs, b.aabbs
        i = j = 0
        na, nb = len(pa), len(pb)
        while i < na and j < nb:
            t0a, t1a, x0a, x1a, y0a, y1a, z0a, z1a = aa[i]
            t0b, t1b, x0b, x1b, y0b, y1b, z0b, z1b = ab[j]
            lo = t0a if t0a > t0b else t0b
            hi = t1a if t1a < t1b else t1b
            if hi - lo > 1e-12:
                if (
                    x1a + d_sep >= x0b
                    and x1b + d_sep >= x0a
                    and y1a + d_sep >= y0b
                    and y1b + d_sep >= y0a
                    and z1a + d_sep >= z0b
                    and z1b + d_sep >= z0a
                ):
                    piece_a = pa[i]
                    piece_b = pb[j]
                    bad = close_intervals(piece_a, piece_b, lo, hi, d_sep)
                    if bad is not None:
                        if exempt_center is not None:
                            ea = within_radius_interval(piece_a, lo, hi, exempt_center, exempt_r)
                            eb = within_radius_interval(piece_b, lo, hi, exempt_center, exempt_r)
                            if ea is not None and eb is not None:
                                e1 = max(ea[0], eb[0])
                                e2 = min(ea[1], eb[1])
                                if e1 <= e2:
                                    parts = []
                                    if e1 - bad[0] > 1e-6:
                                        parts.append((bad[0], min(bad[1], e1)))
                                    if bad[1] - e2 > 1e-6:
                                        parts.append((max(bad[0], e2), bad[1]))
                                    bad = parts[0] if parts else None
                        if bad is not None and bad[1] - bad[0] > 1e-6:
                            t_mid = 0.5 * (bad[0] + bad[1])
                            md = min_distance(piece_a, piece_b, lo, hi)
                            return (t_mid, md, position_at(piece_a, t_mid), position_at(piece_b, t_mid))
            if t1a < t1b:
                i += 1
            else:
                j += 1
        return None

    def first_conflict(
        self,
        aid: str,
        wps_json: List[dict],
        ignore_aids: Optional[set] = None,
    ) -> Optional[Tuple[str, float, float, Point3, Point3]]:
        """Checks wps_json against all indexed routes (except aid and ignore_aids)."""
        item = self._make_item(aid, wps_json)
        if item is None:
            return None
        for other_id, other_item in self.items.items():
            if other_id == aid or (ignore_aids and other_id in ignore_aids):
                continue
            c = self._pair_conflict(item, other_item)
            if c is not None:
                return (other_id, c[0], c[1], c[2], c[3])
        return None

    def add(self, aid: str, wps_json: List[dict]) -> None:
        item = self._make_item(aid, wps_json)
        if item is not None:
            self.items[aid] = item
            self.routes_json[aid] = wps_json

    def remove(self, aid: str) -> None:
        self.items.pop(aid, None)
        self.routes_json.pop(aid, None)

    def launch_crowd_score(self, home: Point3, t_depart: float) -> int:
        """Counts how many already-placed routes are near `home` around `t_depart`."""
        cnt = 0
        for item in self.items.values():
            if abs(item.t_depart - t_depart) <= 5.0:
                if math.hypot(item.home[0] - home[0], item.home[1] - home[1]) <= 120.0:
                    cnt += 1
            elif abs(item.t_end - t_depart) <= 5.0:
                if math.hypot(item.home[0] - home[0], item.home[1] - home[1]) <= 120.0:
                    cnt += 1
        return cnt


def _insert_local_dodge(
    wps_json: List[dict],
    ac: Aircraft,
    t_conflict: float,
    pos_self: Point3,
    pos_other: Point3,
    obs_map: ObstacleMap,
    scenario: Scenario,
    offset_m: float = 52.0,
    side: int = 1,
    vertical: bool = False,
) -> Optional[List[dict]]:
    """Inserts a local 3D bypass waypoint into wps_json around t_conflict."""
    r = scenario.rules
    wps = _to_waypoints(wps_json)
    leg_idx = -1
    for k in range(len(wps) - 1):
        if wps[k].depart <= t_conflict <= wps[k + 1].t:
            leg_idx = k
            break
    if leg_idx == -1:
        return None

    a, b = wps[leg_idx], wps[leg_idx + 1]
    leg_dur = b.t - a.depart
    if leg_dur < 1.5:
        return None

    f = max(0.015, min(0.985, (t_conflict - a.depart) / leg_dur))
    mx = a.x + f * (b.x - a.x)
    my = a.y + f * (b.y - a.y)
    mz = a.z + f * (b.z - a.z)

    if vertical:
        nz = mz + side * offset_m
        nz = max(r.z_min_m, min(r.z_max_m, nz))
        if abs(nz - pos_other[2]) < r.d_sep_m + 2.0:
            nz = max(r.z_min_m, min(r.z_max_m, pos_other[2] + side * (r.d_sep_m + 6.0)))
        dodge: Point3 = (mx, my, nz)
    else:
        dx, dy = b.x - a.x, b.y - a.y
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            nx, ny = 1.0, 0.0
        else:
            nx, ny = -dy / norm, dx / norm
        vx, vy = mx - pos_other[0], my - pos_other[1]
        if vx * nx + vy * ny < 0:
            nx, ny = -nx, -ny
        nx *= side
        ny *= side
        nz = max(r.z_min_m, min(r.z_max_m, mz))
        dodge = (mx + nx * offset_m, my + ny * offset_m, nz)

    if not obs_map.segment_valid_3d(a.pos, dodge) or not obs_map.segment_valid_3d(dodge, b.pos):
        return None

    pad_d = 0.0005 if vertical else 0.005
    dt1 = max(
        math.hypot(dodge[0] - a.x, dodge[1] - a.y) / ac.v_h_mps,
        (dodge[2] - a.z) / ac.v_up_mps if dodge[2] > a.z else (a.z - dodge[2]) / ac.v_down_mps,
    ) + pad_d
    dt2 = max(
        math.hypot(b.x - dodge[0], b.y - dodge[1]) / ac.v_h_mps,
        (b.z - dodge[2]) / ac.v_up_mps if b.z > dodge[2] else (dodge[2] - b.z) / ac.v_down_mps,
    ) + pad_d

    if dt1 + dt2 > leg_dur - 1e-4:
        return None

    slack = leg_dur - (dt1 + dt2)
    t_dodge = a.depart + dt1 + slack * (dt1 / (dt1 + dt2))
    new_wps = list(wps_json[: leg_idx + 1]) + [_wp_dict(t_dodge, dodge)] + list(wps_json[leg_idx + 1 :])
    parsed = _to_waypoints(new_wps)
    if route_energy_wh(parsed, ac) > ac.usable_wh - 1e-3:
        return None
    if not _check_r2_fast(parsed, ac, r.z_min_m, r.z_max_m, scenario.horizon_s):
        return None
    return new_wps


def _try_dodge_chain(
    wps: List[dict],
    ac: Aircraft,
    obs_map: ObstacleMap,
    scenario: Scenario,
    sep_index: SeparationIndex,
    max_dodges: int = 4,
) -> Optional[Tuple[List[dict], float]]:
    """Repeatedly inserts local 3D bypass waypoints to clear up to max_dodges conflicts."""
    cur = wps
    for _ in range(max_dodges):
        conf = sep_index.first_conflict(ac.id, cur)
        if conf is None:
            return cur, route_energy_wh(_to_waypoints(cur), ac)
        _, tc, _, pa, pb = conf
        progressed = False
        for vert, side, off in (
            (False, 1, 52.0),
            (False, -1, 52.0),
            (True, 1, 46.0),
            (True, -1, 46.0),
            (False, 1, 88.0),
            (False, -1, 88.0),
        ):
            cand = _insert_local_dodge(
                cur, ac, tc, pa, pb, obs_map, scenario,
                offset_m=off, side=side, vertical=vert,
            )
            if cand is None:
                continue
            nc = sep_index.first_conflict(ac.id, cand)
            if nc is None:
                return cand, route_energy_wh(_to_waypoints(cand), ac)
            if nc[1] > tc + 2.0 or nc[0] != conf[0]:
                cur = cand
                progressed = True
                break
        if not progressed:
            break
    return None


def _add_vertical_profile(
    wps_json: List[dict],
    ac: Aircraft,
    obs_map: ObstacleMap,
    scenario: Scenario,
    climb_z: Optional[float] = None,
    descend_z: Optional[float] = None,
) -> Optional[Tuple[List[dict], float]]:
    """Adds early-climb (leg 0) and/or late-descent (last leg) waypoints along the exact horizontal path (0 extra time!)."""
    r = scenario.rules
    out = list(wps_json)
    changed = False
    if climb_z is not None and len(out) >= 2:
        wps = _to_waypoints(out)
        a, b = wps[0], wps[1]
        leg_dt = b.t - a.depart
        cz = max(r.z_min_m, min(r.z_max_m, climb_z))
        tz = max(0.0, cz - a.z) / (ac.v_up_mps * 0.985)
        if tz > 0.2 and leg_dt > tz + 0.4:
            f = tz / leg_dt
            p = (a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), cz)
            if obs_map.segment_valid_3d(a.pos, p) and obs_map.segment_valid_3d(p, b.pos):
                out.insert(1, _wp_dict(a.depart + tz, p))
                changed = True
    if descend_z is not None and len(out) >= 2:
        wps2 = _to_waypoints(out)
        a, b = wps2[-2], wps2[-1]
        leg_dt = b.t - a.depart
        dz = max(r.z_min_m, min(r.z_max_m, descend_z))
        tz = max(0.0, dz - b.z) / (ac.v_down_mps * 0.985)
        if tz > 0.2 and leg_dt > tz + 0.4:
            f = 1.0 - tz / leg_dt
            p = (a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), dz)
            if obs_map.segment_valid_3d(a.pos, p) and obs_map.segment_valid_3d(p, b.pos):
                out.insert(len(out) - 1, _wp_dict(b.t - tz, p))
                changed = True
    if not changed:
        return None
    parsed = _to_waypoints(out)
    en = route_energy_wh(parsed, ac)
    if en > ac.usable_wh - 1e-3 or not _check_r2_fast(parsed, ac, r.z_min_m, r.z_max_m, scenario.horizon_s):
        return None
    return out, en


def place_scout_deconflicted(
    ac: Aircraft,
    contacts: Sequence[Contact],
    obs_map: ObstacleMap,
    scenario: Scenario,
    sep_index: SeparationIndex,
    preferred_z: float = 30.0,
    max_obs_end_overrides: Optional[Dict[str, float]] = None,
) -> Optional[Tuple[List[dict], float, Dict[str, float]]]:
    """Places a scout route cleanly into `sep_index` minimizing energy penalty.

    Uses targeted launch-delay (min_t0), landing-advance (max_t_land), uniform shifts,
    altitude strata, and local 3D dodge chains.
    """
    if not contacts:
        return None
    r = scenario.rules
    z_candidates = [preferred_z]
    for zc in (r.z_min_m, 71.0, 112.0, 149.0, 50.0, 91.0, 132.0):
        zc = max(r.z_min_m, min(r.z_max_m, zc))
        if all(abs(zc - existing) > 1.0 for existing in z_candidates):
            z_candidates.append(zc)

    best_clean: Optional[Tuple[List[dict], float, Dict[str, float]]] = None

    for z_idx, z in enumerate(z_candidates):
        obs_pts = obs_map.optimize_disk_tour(ac.home[:2], contacts, z)

        # 1. Try nominal route first
        base = build_scout_route(
            ac, contacts, obs_pts, z, obs_map, scenario,
            max_obs_end_overrides=max_obs_end_overrides,
        )
        if base is None:
            continue
        base_wps, base_e, base_oends = base
        if best_clean is not None and base_e >= best_clean[1] - 1e-4:
            continue

        conf0 = sep_index.first_conflict(ac.id, base_wps)
        if conf0 is None:
            best_clean = (base_wps, base_e, base_oends)
            if z_idx == 0:
                return best_clean
            continue

        t0_orig = base_wps[0]["t"]
        t_land_orig = base_wps[-1]["t"]

        # 1b. Early-climb / late-descent vertical profile (0.00 s extra time, 0.00 Wh at z!)
        for cz, dz in ((z, z), (z + 25.0, None), (None, z + 25.0), (z + 45.0, z + 45.0)):
            vp = _add_vertical_profile(base_wps, ac, obs_map, scenario, climb_z=cz, descend_z=dz)
            if vp is not None and (best_clean is None or vp[1] < best_clean[1] - 1e-4):
                if sep_index.first_conflict(ac.id, vp[0]) is None:
                    best_clean = (vp[0], vp[1], base_oends)
                    if z_idx == 0 and vp[1] <= base_e + 0.05:
                        return best_clean
                    break

        # 2. Uniform time shifts (0 Wh!) — check early before paying any compression Wh!
        for dt in (1.5, -1.5, 3.5, -3.5, 7.0, -7.0, 12.0, -12.0, 20.0, -20.0, 32.0, -32.0, 50.0, -50.0):
            cand = build_scout_route(
                ac, contacts, obs_pts, z, obs_map, scenario,
                max_obs_end_overrides=max_obs_end_overrides,
                time_shift=dt,
            )
            if cand is None:
                continue
            if best_clean is not None and cand[1] >= best_clean[1] - 1e-4:
                continue
            if sep_index.first_conflict(ac.id, cand[0]) is None:
                best_clean = cand
                if z_idx == 0:
                    return best_clean
                break

        # 3. Targeted launch delay (min_t0) & landing advance (max_t_land) — fine monotone grid!
        launch_delays = (1.0, 2.0, 3.0, 4.0, 5.0, 6.5, 8.5, 11.0, 15.0, 21.0, 30.0, 44.0, 62.0)
        for d_t0 in launch_delays:
            cand = build_scout_route(
                ac, contacts, obs_pts, z, obs_map, scenario,
                max_obs_end_overrides=max_obs_end_overrides,
                min_t0=t0_orig + d_t0,
            )
            if cand is None:
                break  # Monotone in min_t0: larger delays are also infeasible
            wps, en, oends = cand
            if best_clean is not None and en >= best_clean[1] - 1e-4:
                continue
            c_hit = sep_index.first_conflict(ac.id, wps)
            if c_hit is None:
                best_clean = (wps, en, oends)
                if z_idx == 0 and en <= base_e + 0.25:
                    return best_clean
                break
            # Try 0-Wh vertical profile on delayed launch
            vp = _add_vertical_profile(wps, ac, obs_map, scenario, climb_z=z, descend_z=z)
            if vp is not None and (best_clean is None or vp[1] < best_clean[1] - 1e-4):
                if sep_index.first_conflict(ac.id, vp[0]) is None:
                    best_clean = (vp[0], vp[1], oends)
                    if z_idx == 0 and vp[1] <= base_e + 0.25:
                        return best_clean
                    break
            if c_hit[1] > wps[-1]["t"] - 45.0:
                for d_land in (1.5, 3.0, 4.5, 6.5, 10.0, 16.0, 25.0, 40.0):
                    cand2 = build_scout_route(
                        ac, contacts, obs_pts, z, obs_map, scenario,
                        max_obs_end_overrides=max_obs_end_overrides,
                        min_t0=t0_orig + d_t0,
                        max_t_land=wps[-1]["t"] - d_land,
                    )
                    if cand2 is None:
                        break
                    if sep_index.first_conflict(ac.id, cand2[0]) is None:
                        if best_clean is None or cand2[1] < best_clean[1]:
                            best_clean = cand2
                        break
            else:
                dodged = _try_dodge_chain(wps, ac, obs_map, scenario, sep_index, max_dodges=3)
                if dodged is not None and (best_clean is None or dodged[1] < best_clean[1]):
                    best_clean = (dodged[0], dodged[1], oends)
                    break

        # 4. Landing-only advance (fine monotone grid — catches exact tight slack like d_land=4.5s!)
        if conf0[1] > t_land_orig - 65.0:
            for d_land in (1.5, 3.0, 4.5, 6.0, 8.0, 11.0, 16.0, 24.0, 38.0):
                cand = build_scout_route(
                    ac, contacts, obs_pts, z, obs_map, scenario,
                    max_obs_end_overrides=max_obs_end_overrides,
                    max_t_land=t_land_orig - d_land,
                )
                if cand is None:
                    break  # Monotone in max_t_land
                if best_clean is not None and cand[1] >= best_clean[1] - 1e-4:
                    continue
                c_hit = sep_index.first_conflict(ac.id, cand[0])
                if c_hit is None:
                    best_clean = cand
                    if z_idx == 0 and cand[1] <= base_e + 0.25:
                        return best_clean
                    break
                vp = _add_vertical_profile(cand[0], ac, obs_map, scenario, climb_z=z, descend_z=z)
                if vp is not None and sep_index.first_conflict(ac.id, vp[0]) is None:
                    if best_clean is None or vp[1] < best_clean[1]:
                        best_clean = (vp[0], vp[1], cand[2])
                    break
                dodged = _try_dodge_chain(cand[0], ac, obs_map, scenario, sep_index, max_dodges=3)
                if dodged is not None and (best_clean is None or dodged[1] < best_clean[1]):
                    best_clean = (dodged[0], dodged[1], cand[2])
                    break

        # 5. Local 3D dodge chain on the nominal route
        if best_clean is None or base_e + 0.5 < best_clean[1]:
            dodged = _try_dodge_chain(base_wps, ac, obs_map, scenario, sep_index, max_dodges=4)
            if dodged is not None and (best_clean is None or dodged[1] < best_clean[1]):
                best_clean = (dodged[0], dodged[1], base_oends)

        if best_clean is not None and z_idx >= 1:
            return best_clean

    return best_clean


def place_effector_group_deconflicted(
    contact: Contact,
    effectors: Sequence[Aircraft],
    obs_end: float,
    obs_map: ObstacleMap,
    scenario: Scenario,
    sep_index: SeparationIndex,
    eff_earliest: Dict[str, float],
    use_direct_z0: bool = True,
) -> Optional[Tuple[Dict[str, List[dict]], float]]:
    """Places all effectors for `contact` respecting R4, R8, R9, R10 with minimal energy."""
    if not effectors:
        return {}, 0.0
    r = scenario.rules
    lo, hi = contact.window
    t_min_base = max(obs_end + r.confirm_margin_s + 0.05, lo + 0.02)
    for eff in effectors:
        t_min_base = max(t_min_base, eff_earliest.get(eff.id, 0.0) + 0.05)
    t_max = hi - 0.02
    if t_min_base > t_max:
        return None

    sync_max = r.sync_window_s - 0.15
    span = t_max - t_min_base

    raw_bases: List[float] = [t_min_base]
    step = max(3.5, span / 28.0) if span > 10.0 else max(1.0, span / 6.0)
    t_cur = t_min_base + step
    while t_cur <= t_max - 1e-3 and len(raw_bases) < 34:
        raw_bases.append(t_cur)
        t_cur += step
    if span > 1.0 and abs(raw_bases[-1] - t_max) > 1.0:
        raw_bases.append(max(t_min_base, t_max - min(sync_max, 3.0 * max(0, len(effectors) - 1))))

    def base_crowd(tb: float) -> Tuple[int, float]:
        score = 0
        for idx, eff in enumerate(effectors):
            t_arr = min(t_max, tb + 3.0 * idx)
            t_dep_est = max(0.0, t_arr - eff_earliest.get(eff.id, 0.0))
            score += sep_index.launch_crowd_score(eff.home, t_dep_est)
        return (score, tb)

    raw_bases.sort(key=base_crowd)

    ordered_effs = sorted(effectors, key=lambda e: -eff_earliest.get(e.id, 0.0))

    z_levels = (0.0, r.z_min_m, 71.0, 112.0, 149.0, 50.0, 91.0, 132.0) if use_direct_z0 else (
        r.z_min_m, 71.0, 112.0, 149.0, 50.0, 91.0, 132.0
    )

    best_group: Optional[Tuple[Dict[str, List[dict]], float]] = None
    clean_found = 0

    for base_idx, t_base in enumerate(raw_bases):
        placed: Dict[str, List[dict]] = {}
        placed_times: List[float] = []
        group_e = 0.0
        ok_all = True
        used_high_z = False
        temp_added: List[str] = []

        for e_idx, eff in enumerate(ordered_effs):
            if placed_times:
                lo_e = max(t_min_base, max(placed_times) - sync_max)
                hi_e = min(t_max, min(placed_times) + sync_max)
            else:
                lo_e = t_base
                hi_e = min(t_max, t_base + sync_max)

            if lo_e > hi_e + 1e-6:
                ok_all = False
                break

            cand_arrs: List[float] = []
            pref = min(hi_e, max(lo_e, t_base + 3.0 * e_idx))
            cand_arrs.append(pref)
            for off in (0.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 1.5, 4.5, 7.5, 10.5, 13.5, 16.5, 19.0):
                ta = lo_e + off
                if ta <= hi_e + 1e-6 and all(abs(ta - x) > 0.4 for x in cand_arrs):
                    cand_arrs.append(min(hi_e, ta))

            found_eff: Optional[Tuple[List[dict], float, float]] = None

            # Tier 1: z0 (0.0 direct or 30.0) across arrival offsets
            z0 = z_levels[0]
            for ta in cand_arrs:
                built = build_effector_route(eff, contact, ta, obs_map, scenario, z_cruise=z0)
                if built is None:
                    continue
                wps, en, _ = built
                if sep_index.first_conflict(eff.id, wps) is None:
                    found_eff = (wps, en, ta)
                    break

            # Tier 2: small extra_duration or engagement angle shift at z0
            if found_eff is None:
                for extra_d, ang in (
                    (3.0, 0.0), (7.0, 0.0), (14.0, 0.0), (24.0, 0.0),
                    (0.0, 45.0), (0.0, -45.0), (0.0, 95.0), (0.0, -95.0), (0.0, 155.0),
                ):
                    for ta in cand_arrs[:8]:
                        built = build_effector_route(
                            eff, contact, ta, obs_map, scenario,
                            z_cruise=z0, angle_deg=ang, extra_duration=extra_d,
                        )
                        if built is None:
                            continue
                        wps, en, _ = built
                        if sep_index.first_conflict(eff.id, wps) is None:
                            found_eff = (wps, en, ta)
                            break
                    if found_eff is not None:
                        break

            # Tier 2b: Lateral Approach Fan-Out at z_min (solves 0-slack multi-effector convergence for ~0.05 Wh!)
            if found_eff is None:
                for lat_off in (75.0, -75.0, 115.0, -115.0):
                    for extra_d in (0.0, 4.0):
                        for ta in cand_arrs[:6]:
                            built = build_effector_route(
                                eff, contact, ta, obs_map, scenario,
                                z_cruise=r.z_min_m, extra_duration=extra_d, approach_lat=lat_off,
                            )
                            if built is None:
                                continue
                            wps, en, _ = built
                            if sep_index.first_conflict(eff.id, wps) is None:
                                found_eff = (wps, en, ta)
                                break
                        if found_eff is not None:
                            break
                    if found_eff is not None:
                        break

            # Tier 2c: Cheap local 3D dodge at z0 BEFORE expensive high-altitude climbs!
            if found_eff is None:
                for ed in (2.0, 6.0):
                    for ta in cand_arrs[:4]:
                        built = build_effector_route(
                            eff, contact, ta, obs_map, scenario, z_cruise=z0, extra_duration=ed
                        )
                        if built is None:
                            continue
                        dodged = _try_dodge_chain(built[0], eff, obs_map, scenario, sep_index, max_dodges=3)
                        if dodged is not None:
                            found_eff = (dodged[0], dodged[1], ta)
                            break
                    if found_eff is not None:
                        break

            # Tier 3: altitude layers (with optional lateral fan-out for tight convergence!)
            if found_eff is None:
                for zc in z_levels[1:]:
                    for lat_off, extra_d in ((0.0, 0.0), (0.0, 6.0), (75.0, 0.0), (-75.0, 0.0)):
                        for ta in cand_arrs[:6]:
                            built = build_effector_route(
                                eff, contact, ta, obs_map, scenario,
                                z_cruise=zc, extra_duration=extra_d, approach_lat=lat_off,
                            )
                            if built is None:
                                continue
                            wps, en, _ = built
                            if sep_index.first_conflict(eff.id, wps) is None:
                                found_eff = (wps, en, ta)
                                used_high_z = True
                                break
                        if found_eff is not None:
                            break
                    if found_eff is not None:
                        break

            # Tier 4: deeper local 3D dodge chain across altitudes & extra_duration
            if found_eff is None:
                for zc, ed in ((z0, 14.0), (r.z_min_m, 6.0), (71.0, 6.0), (112.0, 6.0)):
                    for ta in cand_arrs[:4]:
                        built = build_effector_route(
                            eff, contact, ta, obs_map, scenario, z_cruise=zc, extra_duration=ed
                        )
                        if built is None:
                            continue
                        dodged = _try_dodge_chain(built[0], eff, obs_map, scenario, sep_index, max_dodges=4)
                        if dodged is not None:
                            found_eff = (dodged[0], dodged[1], ta)
                            used_high_z = True
                            break
                    if found_eff is not None:
                        break

            if found_eff is None:
                ok_all = False
                break

            wps_ok, en_ok, ta_ok = found_eff
            placed[eff.id] = wps_ok
            placed_times.append(ta_ok)
            group_e += en_ok
            sep_index.add(eff.id, wps_ok)
            temp_added.append(eff.id)

        for aid in temp_added:
            sep_index.remove(aid)

        if ok_all:
            if best_group is None or group_e < best_group[1]:
                best_group = (placed, group_e)
            clean_found += 1
            if not used_high_z or clean_found >= 3:
                break

    return best_group


def verify_and_repair_plan(scenario: Scenario, routes_json: Dict[str, List[dict]]) -> dict:
    """Runs official validator and guarantees 100% validity via minimal-loss surgical repair."""
    routes = dict(routes_json)
    aircraft = scenario.aircraft_by_id()
    contacts = scenario.contact_by_id()

    for _ in range(len(routes) + 5):
        plan = {
            "seed": scenario.seed,
            "routes": [{"aircraft": aid, "waypoints": wps} for aid, wps in routes.items()],
        }
        violations = validate(scenario, plan)
        if not violations:
            return plan

        v = violations[0]
        removed = False
        if v.contact and v.contact in contacts:
            cid = v.contact
            to_drop = [
                aid for aid, wps in routes.items()
                if aircraft[aid].type == EFFECTOR and wps and wps[-1].get("target") == cid
            ]
            if to_drop:
                for aid in to_drop:
                    routes.pop(aid, None)
                removed = True
        if not removed and v.aircraft and v.aircraft in routes:
            aid = v.aircraft
            if aircraft[aid].type == EFFECTOR and routes[aid]:
                cid = routes[aid][-1].get("target")
                for other in [a for a, w in routes.items() if aircraft[a].type == EFFECTOR and w and w[-1].get("target") == cid]:
                    routes.pop(other, None)
            else:
                routes.pop(aid, None)
            removed = True
        if not removed and v.rule == "R8":
            tokens = v.message.split()
            cand_aids = [t for t in tokens[:4] if t in routes]
            if cand_aids:
                eff_cands = [a for a in cand_aids if aircraft[a].type == EFFECTOR]
                drop_a = eff_cands[0] if eff_cands else cand_aids[-1]
                if aircraft[drop_a].type == EFFECTOR:
                    cid = routes[drop_a][-1].get("target")
                    for other in [a for a, w in routes.items() if aircraft[a].type == EFFECTOR and w and w[-1].get("target") == cid]:
                        routes.pop(other, None)
                else:
                    routes.pop(drop_a, None)
                removed = True
        if not removed:
            if routes:
                routes.pop(next(reversed(routes)))
            else:
                return {"seed": scenario.seed, "routes": []}

    return {"seed": scenario.seed, "routes": []}
