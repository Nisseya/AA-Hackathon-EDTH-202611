"""Joint Fleet Orchestration solver: 2D Knapsack + Exact Hungarian Duals + Disk-TSP ALNS + 3D Deconfliction."""
from __future__ import annotations

import math
import random
import time
from typing import Dict, List, Optional, Sequence, Set, Tuple

from fleetkit.energy import best_range_speed
from fleetkit.geometry import hdist
from fleetkit.model import (
    CIVILIAN,
    EFFECTOR,
    FRIENDLY,
    HEAVY,
    HOSTILE,
    LIGHT,
    SCOUT,
    UNKNOWN,
    Aircraft,
    Contact,
    Scenario,
)
from fleetkit.scorer import evaluate

from assignment import INF, hungarian_min_cost, hungarian_min_cost_duals, knapsack_2d
from deconflict import (
    SeparationIndex,
    place_effector_group_deconflicted,
    place_scout_deconflicted,
    verify_and_repair_plan,
)
from geometry3d import ObstacleMap, Point2
from timing import build_effector_route, build_scout_route, effector_min_cost, eval_scout_route_energy


def _min_e_per_m(ac: Aircraft) -> float:
    v = min(ac.v_h_mps, best_range_speed(ac.power))
    return (ac.power.p0_w + ac.power.k_v * v * v) / (v * 3600.0)


class FleetSolver:
    """High-performance solver for the Fleet Orchestration problem."""

    def __init__(self, scenario: Scenario, use_direct_z0: bool = True):
        self.sc = scenario
        self.r = scenario.rules
        self.use_direct_z0 = use_direct_z0
        self.obs_map = ObstacleMap(scenario)

        self.aircraft_map = scenario.aircraft_by_id()
        self.contact_map = scenario.contact_by_id()

        self.scouts: List[Aircraft] = sorted(
            [a for a in scenario.aircraft if a.type == SCOUT],
            key=lambda a: (_min_e_per_m(a), -a.usable_wh),
        )
        self.lights: List[Aircraft] = sorted(
            [a for a in scenario.aircraft if a.type == EFFECTOR and a.subtype == LIGHT],
            key=lambda a: (_min_e_per_m(a), -a.v_h_mps),
        )
        self.heavies: List[Aircraft] = sorted(
            [a for a in scenario.aircraft if a.type == EFFECTOR and a.subtype == HEAVY],
            key=lambda a: (_min_e_per_m(a), -a.v_h_mps),
        )

        if self.scouts:
            self.avg_home: Point2 = (
                sum(s.home[0] for s in self.scouts) / len(self.scouts),
                sum(s.home[1] for s in self.scouts) / len(self.scouts),
            )
        else:
            self.avg_home = (150.0, 150.0)

        protected = [c for c in scenario.contacts if c.cls in (FRIENDLY, CIVILIAN)]
        self.legal_hostiles: List[Contact] = []
        self.legal_unknowns: List[Contact] = []

        self.eff_cost: Dict[str, Dict[str, Optional[Tuple[float, float]]]] = {
            a.id: {} for a in self.lights + self.heavies
        }

        z_min = self.r.z_min_m
        self.r_obs_horiz = math.sqrt(max(0.0, self.r.R_obs_m ** 2 - z_min ** 2)) - 0.25

        for c in scenario.contacts:
            if c.cls == HOSTILE:
                if c.window is None:
                    continue
                if c.window[1] - c.window[0] < self.r.obs_dwell_s + self.r.confirm_margin_s - 1e-6:
                    continue
                if any(hdist(c.pos, f.pos) < self.r.collateral_radius_m - 1e-6 for f in protected):
                    continue
                nl = c.needs.get(LIGHT, 0)
                nh = c.needs.get(HEAVY, 0)
                if nl > len(self.lights) or nh > len(self.heavies):
                    continue
                ok_l = 0
                for a in self.lights:
                    res = effector_min_cost(a, c, self.obs_map, scenario)
                    self.eff_cost[a.id][c.id] = res
                    if res is not None:
                        ok_l += 1
                ok_h = 0
                for a in self.heavies:
                    res = effector_min_cost(a, c, self.obs_map, scenario)
                    self.eff_cost[a.id][c.id] = res
                    if res is not None:
                        ok_h += 1
                if ok_l >= nl and ok_h >= nh:
                    self.legal_hostiles.append(c)
            elif c.cls == UNKNOWN:
                if c.window is None:
                    continue
                if c.window[1] - c.window[0] < self.r.obs_dwell_s - 1e-6:
                    continue
                self.legal_unknowns.append(c)

        all_targets = self.legal_hostiles + self.legal_unknowns
        self.all_targets = all_targets
        self.c_dist: Dict[Tuple[str, str], float] = {}
        self.home_dist: Dict[str, float] = {}
        for c in all_targets:
            self.home_dist[c.id] = max(0.0, hdist(self.avg_home, c.pos) - self.r_obs_horiz)
        for i, c1 in enumerate(all_targets):
            for j in range(i + 1, len(all_targets)):
                c2 = all_targets[j]
                d = max(0.0, hdist(c1.pos, c2.pos) - 2.0 * self.r_obs_horiz)
                self.c_dist[(c1.id, c2.id)] = d
                self.c_dist[(c2.id, c1.id)] = d

        self._disk_cache: Dict[Tuple[str, ...], List[Point2]] = {}
        self._scout_eval_cache: Dict[Tuple[str, Tuple[str, ...]], Optional[float]] = {}

        self.legal_hostiles = [c for c in self.legal_hostiles if self._any_scout_can_reach(c)]
        self.legal_unknowns = [c for c in self.legal_unknowns if self._any_scout_can_reach(c)]

        self.est_eff_e: Dict[str, float] = {}
        self.solo_scout_e: Dict[str, float] = {}
        for c in self.legal_hostiles:
            nl = c.needs.get(LIGHT, 0)
            nh = c.needs.get(HEAVY, 0)
            le = sorted(self.eff_cost[a.id][c.id][0] for a in self.lights if self.eff_cost[a.id][c.id] is not None)
            he = sorted(self.eff_cost[a.id][c.id][0] for a in self.heavies if self.eff_cost[a.id][c.id] is not None)
            l_cost = sum(le[:nl]) if nl else 0.0
            h_cost = sum(he[:nh]) if nh else 0.0
            self.est_eff_e[c.id] = l_cost + h_cost
            se = self._best_solo_scout_energy(c.id)
            self.solo_scout_e[c.id] = se if se is not None else 10.0

        for c in self.legal_unknowns:
            se = self._best_solo_scout_energy(c.id)
            self.solo_scout_e[c.id] = se if se is not None else 10.0

        self._relatedness: Dict[str, List[str]] = {}
        all_t = self.legal_hostiles + self.legal_unknowns
        for c in all_t:
            scored = []
            for o in all_t:
                if o.id == c.id:
                    continue
                d = hdist(c.pos, o.pos) / 1000.0
                tw = abs(0.5 * (c.window[0] + c.window[1]) - 0.5 * (o.window[0] + o.window[1])) / 200.0
                scored.append((d + tw, o.id))
            scored.sort()
            self._relatedness[c.id] = [cid for _, cid in scored]

    def _candidate_empty_scouts(self, tours: Sequence[Sequence[str]]) -> List[int]:
        """Returns up to 3 diverse idle scouts: most efficient, largest battery, fastest."""
        empty = [i for i in range(len(self.scouts)) if not tours[i]]
        if not empty:
            return []
        cands = [empty[0]]
        if len(empty) > 1:
            max_bat = max(empty, key=lambda idx: self.scouts[idx].usable_wh)
            if max_bat not in cands:
                cands.append(max_bat)
            fastest = max(empty, key=lambda idx: self.scouts[idx].v_h_mps)
            if fastest not in cands:
                cands.append(fastest)
        return cands

    def _any_scout_can_reach(self, c: Contact) -> bool:
        for s in self.scouts:
            if self.eval_scout_tour(s, (c.id,)) is not None:
                return True
        return False

    def _best_solo_scout_energy(self, cid: str) -> Optional[float]:
        best = None
        for s in self.scouts:
            e = self.eval_scout_tour(s, (cid,))
            if e is not None and (best is None or e < best):
                best = e
        return best

    def _get_disk_pts(self, cids: Tuple[str, ...]) -> List[Point2]:
        pts = self._disk_cache.get(cids)
        if pts is None:
            contacts = [self.contact_map[cid] for cid in cids]
            pts = self.obs_map.optimize_disk_tour(self.avg_home, contacts, self.r.z_min_m, n_iters=3)
            self._disk_cache[cids] = pts
        return pts

    def eval_scout_tour(self, ac: Aircraft, cids: Tuple[str, ...]) -> Optional[float]:
        """Exact minimum energy (Wh) for scout `ac` flying tour `cids` at z_min, or None if infeasible."""
        if not cids:
            return 0.0
        key = (ac.id, cids)
        if key in self._scout_eval_cache:
            return self._scout_eval_cache[key]

        dwell = self.r.obs_dwell_s
        vh = ac.v_h_mps
        margin = self.r.confirm_margin_s
        horizon = self.sc.horizon_s
        t = 0.0
        prev_cid = None
        min_dist_sum = 0.0
        for cid in cids:
            c = self.contact_map[cid]
            d = self.home_dist[cid] if prev_cid is None else self.c_dist[(prev_cid, cid)]
            d_lb = max(0.0, d - 120.0) if prev_cid is None else d
            min_dist_sum += d_lb
            t = max(c.window[0], t + d_lb / vh)
            latest_start = c.window[1] - dwell - (margin if c.cls == HOSTILE else 0.0)
            if t > latest_start + 1e-3:
                self._scout_eval_cache[key] = None
                return None
            t += dwell
            prev_cid = cid
        d_back = max(0.0, self.home_dist[prev_cid] - 120.0)
        min_dist_sum += d_back
        if t + d_back / vh > horizon + 1e-3:
            self._scout_eval_cache[key] = None
            return None
        e_lb = (
            min_dist_sum * _min_e_per_m(ac)
            + len(cids) * ac.power.p0_w * dwell / 3600.0
            + ac.power.e_climb_wh_per_m * self.r.z_min_m
        )
        if e_lb > ac.usable_wh + 0.5:
            self._scout_eval_cache[key] = None
            return None

        contacts = [self.contact_map[cid] for cid in cids]
        pts = self._get_disk_pts(cids)
        res = eval_scout_route_energy(ac, contacts, pts, self.r.z_min_m, self.obs_map, self.sc)
        self._scout_eval_cache[key] = res
        return res

    def assign_effectors(
        self, hostile_ids: Sequence[str]
    ) -> Optional[Tuple[float, Dict[str, List[Aircraft]], Dict[str, float], Tuple[Dict[str, float], List[float], List[float]]]]:
        """Exact globally optimal effector assignment via Hungarian algorithm + dual potentials."""
        l_slots: List[str] = []
        h_slots: List[str] = []
        for cid in hostile_ids:
            c = self.contact_map[cid]
            nl = c.needs.get(LIGHT, 0)
            nh = c.needs.get(HEAVY, 0)
            l_slots.extend([cid] * nl)
            h_slots.extend([cid] * nh)

        if len(l_slots) > len(self.lights) or len(h_slots) > len(self.heavies):
            return None

        eff_by_cid: Dict[str, List[Aircraft]] = {cid: [] for cid in hostile_ids}
        earliest_by_cid: Dict[str, float] = {cid: 0.0 for cid in hostile_ids}
        u_by_cid: Dict[str, float] = {cid: 0.0 for cid in hostile_ids}
        v_light: List[float] = [0.0] * len(self.lights)
        v_heavy: List[float] = [0.0] * len(self.heavies)
        total_e = 0.0

        if l_slots:
            mat_l = []
            for cid in l_slots:
                row = []
                for a in self.lights:
                    info = self.eff_cost[a.id].get(cid)
                    row.append(info[0] if info is not None else INF)
                mat_l.append(row)
            cost_l, assign_l, u_l, v_l = hungarian_min_cost_duals(mat_l)
            if cost_l >= INF * 0.25:
                return None
            total_e += cost_l
            max_vl = max(v_l) if v_l else 0.0
            v_light = [vj - max_vl for vj in v_l]
            for slot_idx, eff_idx in enumerate(assign_l):
                cid = l_slots[slot_idx]
                eff = self.lights[eff_idx]
                eff_by_cid[cid].append(eff)
                u_by_cid[cid] += u_l[slot_idx] + max_vl
                et = self.eff_cost[eff.id][cid][1]
                if et > earliest_by_cid[cid]:
                    earliest_by_cid[cid] = et

        if h_slots:
            mat_h = []
            for cid in h_slots:
                row = []
                for a in self.heavies:
                    info = self.eff_cost[a.id].get(cid)
                    row.append(info[0] if info is not None else INF)
                mat_h.append(row)
            cost_h, assign_h, u_h, v_h = hungarian_min_cost_duals(mat_h)
            if cost_h >= INF * 0.25:
                return None
            total_e += cost_h
            max_vh = max(v_h) if v_h else 0.0
            v_heavy = [vj - max_vh for vj in v_h]
            for slot_idx, eff_idx in enumerate(assign_h):
                cid = h_slots[slot_idx]
                eff = self.heavies[eff_idx]
                eff_by_cid[cid].append(eff)
                u_by_cid[cid] += u_h[slot_idx] + max_vh
                et = self.eff_cost[eff.id][cid][1]
                if et > earliest_by_cid[cid]:
                    earliest_by_cid[cid] = et

        return total_e, eff_by_cid, earliest_by_cid, (u_by_cid, v_light, v_heavy)

    # ------------------------------------------------------------------
    # Scout VRPTW: Regret-2 Construction + VND (incl. Or-opt-2) + Hungarian + ALNS
    # ------------------------------------------------------------------

    def _best_insertion_in_tour(
        self, ac: Aircraft, tour: List[str], base_e: float, cid: str
    ) -> Tuple[float, int]:
        best_de = INF
        best_pos = -1
        for pos in range(len(tour) + 1):
            cand = tuple(tour[:pos] + [cid] + tour[pos:])
            e = self.eval_scout_tour(ac, cand)
            if e is not None:
                de = e - base_e
                if de < best_de:
                    best_de = de
                    best_pos = pos
        return best_de, best_pos

    def _best_pair_insertion_in_tour(
        self, ac: Aircraft, tour: List[str], base_e: float, c1: str, c2: str
    ) -> Tuple[float, int, bool]:
        """Best insertion of 2-contact chain [c1, c2] or [c2, c1] into tour."""
        best_de = INF
        best_pos = -1
        best_rev = False
        for pos in range(len(tour) + 1):
            cand = tuple(tour[:pos] + [c1, c2] + tour[pos:])
            e = self.eval_scout_tour(ac, cand)
            if e is not None:
                de = e - base_e
                if de < best_de:
                    best_de, best_pos, best_rev = de, pos, False
            cand_r = tuple(tour[:pos] + [c2, c1] + tour[pos:])
            er = self.eval_scout_tour(ac, cand_r)
            if er is not None:
                der = er - base_e
                if der < best_de:
                    best_de, best_pos, best_rev = der, pos, True
        return best_de, best_pos, best_rev

    def _regret2_construct(
        self,
        hostile_ids: Sequence[str],
        unknown_ids: Sequence[str],
        hostile_priority: Dict[str, float],
        deadline: float,
        rng: Optional[random.Random] = None,
        noise: float = 0.0,
        seed_tours: Optional[List[List[str]]] = None,
    ) -> Tuple[List[List[str]], List[float], Set[str]]:
        """Regret-2 parallel insertion for hostiles, then greedy profit insertion for unknowns."""
        m = len(self.scouts)
        if seed_tours is not None:
            tours = [list(t) for t in seed_tours]
            tour_e = []
            for i in range(m):
                ev = self.eval_scout_tour(self.scouts[i], tuple(tours[i]))
                if ev is None:
                    tours[i] = []
                    tour_e.append(0.0)
                else:
                    tour_e.append(ev)
        else:
            tours = [[] for _ in range(m)]
            tour_e = [0.0] * m

        already = {cid for t in tours for cid in t}
        remaining_h = [cid for cid in hostile_ids if cid not in already]

        ins_cache: Dict[Tuple[str, int], Tuple[float, int]] = {}

        def get_ins(cid: str, s_idx: int) -> Tuple[float, int]:
            k = (cid, s_idx)
            if k not in ins_cache:
                ins_cache[k] = self._best_insertion_in_tour(self.scouts[s_idx], tours[s_idx], tour_e[s_idx], cid)
            return ins_cache[k]

        unserved_h: Set[str] = set()

        while remaining_h and time.monotonic() < deadline:
            active_scouts = [i for i in range(m) if tours[i]]
            empty_cands = self._candidate_empty_scouts(tours)
            eval_scouts = active_scouts + empty_cands

            best_pick = None
            best_key = (-INF, -INF)
            best_target = None

            infeas_now = []
            for cid in remaining_h:
                b1_de, b1_s, b1_p = INF, -1, -1
                b2_de = INF
                for s_idx in eval_scouts:
                    de, pos = get_ins(cid, s_idx)
                    if de < b1_de:
                        b2_de = b1_de
                        b1_de, b1_s, b1_p = de, s_idx, pos
                    elif de < b2_de:
                        b2_de = de
                if b1_s == -1:
                    for s_idx in range(m):
                        if not tours[s_idx] and s_idx not in eval_scouts:
                            de, pos = get_ins(cid, s_idx)
                            if de < b1_de:
                                b2_de = b1_de
                                b1_de, b1_s, b1_p = de, s_idx, pos
                    if b1_s == -1:
                        infeas_now.append(cid)
                        continue
                regret = (b2_de - b1_de) if b2_de < INF * 0.5 else (15.0 + hostile_priority.get(cid, 50.0) * 0.1)
                if rng is not None and noise > 0.0:
                    regret += rng.uniform(-noise, noise)
                c = self.contact_map[cid]
                score_key = (regret + 0.02 * hostile_priority.get(cid, 0.0), -(c.window[1] - c.window[0]))
                if score_key > best_key:
                    best_key = score_key
                    best_pick = cid
                    best_target = (b1_s, b1_p, b1_de)

            for cid in infeas_now:
                remaining_h.remove(cid)
                unserved_h.add(cid)

            if best_pick is None:
                break

            s_idx, pos, de = best_target
            tours[s_idx].insert(pos, best_pick)
            tour_e[s_idx] += de
            remaining_h.remove(best_pick)
            for cid in remaining_h:
                ins_cache.pop((cid, s_idx), None)

        if remaining_h:
            unserved_h.update(remaining_h)

        # Phase 2: Insert profitable unknowns greedily (checking efficient + max-battery + fastest empty scouts!)
        remaining_u = [cid for cid in unknown_ids if cid not in already]
        lam = self.r.lambda_e
        ins_u: Dict[Tuple[str, int], Tuple[float, int]] = {}

        while remaining_u and time.monotonic() < deadline:
            active_scouts = [i for i in range(m) if tours[i]]
            empty_cands = self._candidate_empty_scouts(tours)
            eval_scouts = active_scouts + empty_cands

            best_u = None
            best_profit = 1e-4
            best_u_target = None

            for cid in remaining_u:
                pts_u = self.contact_map[cid].points
                b_de, b_s, b_p = INF, -1, -1
                for s_idx in eval_scouts:
                    k = (cid, s_idx)
                    if k not in ins_u:
                        ins_u[k] = self._best_insertion_in_tour(self.scouts[s_idx], tours[s_idx], tour_e[s_idx], cid)
                    de, pos = ins_u[k]
                    if de < b_de:
                        b_de, b_s, b_p = de, s_idx, pos
                if b_s != -1:
                    profit = pts_u - lam * b_de
                    if rng is not None and noise > 0.0:
                        profit += rng.uniform(-0.3 * noise, 0.3 * noise)
                    if profit > best_profit:
                        best_profit = profit
                        best_u = cid
                        best_u_target = (b_s, b_p, b_de)

            if best_u is None:
                break
            s_idx, pos, de = best_u_target
            tours[s_idx].insert(pos, best_u)
            tour_e[s_idx] += de
            remaining_u.remove(best_u)
            for cid in remaining_u:
                ins_u.pop((cid, s_idx), None)

        return tours, tour_e, unserved_h

    def _try_hungarian_scout_match(
        self, non_empty: Sequence[List[str]]
    ) -> Optional[Tuple[List[List[str]], List[float]]]:
        """Exact globally optimal assignment of non-empty tours to physical scouts, or None if infeasible."""
        k = len(non_empty)
        m = len(self.scouts)
        if k == 0:
            return [[] for _ in range(m)], [0.0] * m
        if k > m:
            return None

        mat = []
        for t in non_empty:
            tup = tuple(t)
            row = []
            for s in self.scouts:
                e = self.eval_scout_tour(s, tup)
                row.append(e if e is not None else INF)
            mat.append(row)

        cost, assign = hungarian_min_cost(mat)
        if cost >= INF * 0.25:
            return None

        new_tours = [[] for _ in range(m)]
        new_e = [0.0] * m
        for t_idx, s_idx in enumerate(assign):
            new_tours[s_idx] = list(non_empty[t_idx])
            new_e[s_idx] = mat[t_idx][s_idx]
        return new_tours, new_e

    def _hungarian_scout_match(self, tours: List[List[str]]) -> Tuple[List[List[str]], List[float]]:
        """Exact globally optimal assignment of non-empty tours to physical scouts."""
        res = self._try_hungarian_scout_match([t for t in tours if t])
        if res is not None:
            return res
        m = len(self.scouts)
        return tours, [self.eval_scout_tour(self.scouts[i], tuple(tours[i])) or 0.0 for i in range(m)]

    def _vnd_scout_routes(
        self,
        tours: List[List[str]],
        tour_e: List[float],
        unknown_ids: Sequence[str],
        deadline: float,
        max_passes: int = 5,
        deep: bool = False,
    ) -> Tuple[List[List[str]], List[float]]:
        """Variable Neighborhood Descent on scout tours: 2-opt, Or-opt-1/2, Relocate-1/2, Swap, 2-opt*, Hungarian, Unknowns."""
        m = len(self.scouts)
        lam = self.r.lambda_e

        for _pass in range(max_passes):
            if time.monotonic() > deadline:
                break
            improved = False

            # 1. Intra-route 2-opt & Or-opt-1 & Or-opt-2
            for s_idx in range(m):
                t = tours[s_idx]
                n = len(t)
                if n < 2:
                    continue
                ac = self.scouts[s_idx]
                for i in range(n - 1):
                    for j in range(i + 1, n):
                        cand = tuple(t[:i] + t[i : j + 1][::-1] + t[j + 1 :])
                        e = self.eval_scout_tour(ac, cand)
                        if e is not None and e < tour_e[s_idx] - 1e-4:
                            tours[s_idx] = list(cand)
                            tour_e[s_idx] = e
                            t = tours[s_idx]
                            improved = True
                n = len(t)
                for i in range(n):
                    cid = t[i]
                    base = t[:i] + t[i + 1 :]
                    for j in range(n):
                        if j == i:
                            continue
                        cand = tuple(base[:j] + [cid] + base[j:])
                        e = self.eval_scout_tour(ac, cand)
                        if e is not None and e < tour_e[s_idx] - 1e-4:
                            tours[s_idx] = list(cand)
                            tour_e[s_idx] = e
                            t = tours[s_idx]
                            improved = True
                            break
                n = len(t)
                if n >= 4 and (deep or n <= 8):
                    for i in range(n - 1):
                        pair = t[i : i + 2]
                        base = t[:i] + t[i + 2 :]
                        for j in range(len(base) + 1):
                            if j == i:
                                continue
                            cand = tuple(base[:j] + pair + base[j:])
                            e = self.eval_scout_tour(ac, cand)
                            if e is not None and e < tour_e[s_idx] - 1e-4:
                                tours[s_idx] = list(cand)
                                tour_e[s_idx] = e
                                t = tours[s_idx]
                                improved = True
                                break

            # 2. Inter-route Relocate-1 (to active or diverse empty scouts)
            target_scouts = [i for i in range(m) if tours[i]] + self._candidate_empty_scouts(tours)
            for a_idx in range(m):
                if not tours[a_idx] or time.monotonic() > deadline:
                    continue
                i = 0
                while i < len(tours[a_idx]):
                    if time.monotonic() > deadline:
                        break
                    cid = tours[a_idx][i]
                    rem_a = tours[a_idx][:i] + tours[a_idx][i + 1 :]
                    e_a_new = self.eval_scout_tour(self.scouts[a_idx], tuple(rem_a))
                    if e_a_new is None:
                        i += 1
                        continue
                    saved_a = tour_e[a_idx] - e_a_new
                    best_gain = 1e-4
                    best_move = None
                    for b_idx in target_scouts:
                        if b_idx == a_idx:
                            continue
                        de_b, pos_b = self._best_insertion_in_tour(self.scouts[b_idx], tours[b_idx], tour_e[b_idx], cid)
                        gain = saved_a - de_b
                        if gain > best_gain:
                            best_gain = gain
                            best_move = (b_idx, pos_b, tour_e[b_idx] + de_b)
                    if best_move is not None:
                        b_idx, pos_b, e_b_new = best_move
                        tours[a_idx] = rem_a
                        tour_e[a_idx] = e_a_new
                        tours[b_idx].insert(pos_b, cid)
                        tour_e[b_idx] = e_b_new
                        improved = True
                        target_scouts = [k for k in range(m) if tours[k]] + self._candidate_empty_scouts(tours)
                    else:
                        i += 1

            # 2b. Inter-route Or-opt-2 (only in deep polish & for spatially close pairs -> fast!)
            if deep and time.monotonic() < deadline:
                target_scouts = [i for i in range(m) if tours[i]] + self._candidate_empty_scouts(tours)
                for a_idx in range(m):
                    if len(tours[a_idx]) < 3 or time.monotonic() > deadline:
                        continue
                    i = 0
                    while i < len(tours[a_idx]) - 1:
                        if time.monotonic() > deadline:
                            break
                        c1, c2 = tours[a_idx][i], tours[a_idx][i + 1]
                        if self.c_dist.get((c1, c2), INF) > 600.0:
                            i += 1
                            continue
                        rem_a = tours[a_idx][:i] + tours[a_idx][i + 2 :]
                        e_a_new = self.eval_scout_tour(self.scouts[a_idx], tuple(rem_a))
                        if e_a_new is None:
                            i += 1
                            continue
                        saved_a = tour_e[a_idx] - e_a_new
                        best_gain = 1e-4
                        best_move = None
                        for b_idx in target_scouts:
                            if b_idx == a_idx:
                                continue
                            de_b, pos_b, rev = self._best_pair_insertion_in_tour(
                                self.scouts[b_idx], tours[b_idx], tour_e[b_idx], c1, c2
                            )
                            gain = saved_a - de_b
                            if gain > best_gain:
                                best_gain = gain
                                best_move = (b_idx, pos_b, rev, tour_e[b_idx] + de_b)
                        if best_move is not None:
                            b_idx, pos_b, rev, e_b_new = best_move
                            tours[a_idx] = rem_a
                            tour_e[a_idx] = e_a_new
                            seg = [c2, c1] if rev else [c1, c2]
                            tours[b_idx] = tours[b_idx][:pos_b] + seg + tours[b_idx][pos_b:]
                            tour_e[b_idx] = e_b_new
                            improved = True
                            target_scouts = [k for k in range(m) if tours[k]] + self._candidate_empty_scouts(tours)
                        else:
                            i += 1

            # 3. Inter-route 1-1 Swap & 2-opt* (tail swap)
            non_empty = [i for i in range(m) if tours[i]]
            for ii in range(len(non_empty)):
                if time.monotonic() > deadline:
                    break
                a_idx = non_empty[ii]
                for jj in range(ii + 1, len(non_empty)):
                    b_idx = non_empty[jj]
                    ta, tb = tours[a_idx], tours[b_idx]
                    if not ta or not tb:
                        continue
                    cur_sum = tour_e[a_idx] + tour_e[b_idx]
                    changed_pair = False
                    for ia, ca in enumerate(ta):
                        rem_a = ta[:ia] + ta[ia + 1 :]
                        for ib, cb in enumerate(tb):
                            rem_b = tb[:ib] + tb[ib + 1 :]
                            cand_a = tuple(rem_a[:ia] + [cb] + rem_a[ia:])
                            ea = self.eval_scout_tour(self.scouts[a_idx], cand_a)
                            if ea is None:
                                continue
                            cand_b = tuple(rem_b[:ib] + [ca] + rem_b[ib:])
                            eb = self.eval_scout_tour(self.scouts[b_idx], cand_b)
                            if eb is not None and (ea + eb) < cur_sum - 1e-4:
                                tours[a_idx] = list(cand_a)
                                tours[b_idx] = list(cand_b)
                                tour_e[a_idx] = ea
                                tour_e[b_idx] = eb
                                improved = True
                                changed_pair = True
                                break
                        if changed_pair:
                            break
                    if not changed_pair and len(ta) >= 2 and len(tb) >= 2:
                        for ia in range(1, len(ta)):
                            for ib in range(1, len(tb)):
                                cand_a = tuple(ta[:ia] + tb[ib:])
                                ea = self.eval_scout_tour(self.scouts[a_idx], cand_a)
                                if ea is None:
                                    continue
                                cand_b = tuple(tb[:ib] + ta[ia:])
                                eb = self.eval_scout_tour(self.scouts[b_idx], cand_b)
                                if eb is not None and (ea + eb) < cur_sum - 1e-4:
                                    tours[a_idx] = list(cand_a)
                                    tours[b_idx] = list(cand_b)
                                    tour_e[a_idx] = ea
                                    tour_e[b_idx] = eb
                                    improved = True
                                    changed_pair = True
                                    break
                            if changed_pair:
                                break

            # 4. Exact Hungarian Scout-to-Tour Reassignment
            old_total = sum(tour_e)
            tours, tour_e = self._hungarian_scout_match(tours)
            if sum(tour_e) < old_total - 1e-4:
                improved = True

            # 5. Unknown drop/add polish (checking ALL diverse empty scouts!)
            served_set = {cid for t in tours for cid in t}
            for s_idx in range(m):
                i = 0
                while i < len(tours[s_idx]):
                    cid = tours[s_idx][i]
                    c = self.contact_map[cid]
                    if c.cls == UNKNOWN:
                        rem = tours[s_idx][:i] + tours[s_idx][i + 1 :]
                        e_new = self.eval_scout_tour(self.scouts[s_idx], tuple(rem))
                        if e_new is not None and lam * (tour_e[s_idx] - e_new) > c.points + 1e-4:
                            tours[s_idx] = rem
                            tour_e[s_idx] = e_new
                            served_set.discard(cid)
                            improved = True
                            continue
                    i += 1
            unserved_u = [cid for cid in unknown_ids if cid not in served_set]
            if unserved_u and time.monotonic() < deadline:
                for cid in sorted(unserved_u, key=lambda x: -self.contact_map[x].points):
                    eval_scouts = [i for i in range(m) if tours[i]] + self._candidate_empty_scouts(tours)
                    pts_u = self.contact_map[cid].points
                    best_profit = 1e-4
                    best_ins = None
                    for s_idx in eval_scouts:
                        de, pos = self._best_insertion_in_tour(self.scouts[s_idx], tours[s_idx], tour_e[s_idx], cid)
                        if de < INF * 0.5:
                            prof = pts_u - lam * de
                            if prof > best_profit:
                                best_profit = prof
                                best_ins = (s_idx, pos, de)
                    if best_ins is not None:
                        s_idx, pos, de = best_ins
                        tours[s_idx].insert(pos, cid)
                        tour_e[s_idx] += de
                        served_set.add(cid)
                        improved = True

            if not improved:
                # 6. Tour Merge at VND local optimum (uses Hungarian reassignment so occupied big-battery scouts can absorb merged tours!)
                ne_now = [i for i in range(m) if tours[i]]
                if len(ne_now) >= 2 and time.monotonic() < deadline:
                    cand_hosts = sorted(
                        range(m), key=lambda idx: (-self.scouts[idx].usable_wh, _min_e_per_m(self.scouts[idx]))
                    )[:4]
                    cur_total_e = sum(tour_e)
                    merged_any = False
                    for ii in range(len(ne_now)):
                        if merged_any or time.monotonic() > deadline:
                            break
                        a_idx = ne_now[ii]
                        for jj in range(ii + 1, len(ne_now)):
                            b_idx = ne_now[jj]
                            ta, tb = tours[a_idx], tours[b_idx]
                            if not ta or not tb or len(ta) + len(tb) > 14:
                                continue
                            cur_sum = tour_e[a_idx] + tour_e[b_idx]
                            base_t, add_t = (ta, tb) if len(ta) >= len(tb) else (tb, ta)
                            best_merge = None
                            for h_idx in cand_hosts:
                                host_ac = self.scouts[h_idx]
                                if host_ac.usable_wh < cur_sum * 0.45:
                                    continue
                                e_cur = self.eval_scout_tour(host_ac, tuple(base_t))
                                if e_cur is None:
                                    continue
                                cur_t = list(base_t)
                                ok_m = True
                                for cid in add_t:
                                    de, pos = self._best_insertion_in_tour(host_ac, cur_t, e_cur, cid)
                                    if pos == -1 or e_cur + de >= cur_sum - 1e-4:
                                        ok_m = False
                                        break
                                    cur_t.insert(pos, cid)
                                    e_cur += de
                                if ok_m and e_cur < cur_sum - 1e-4:
                                    if best_merge is None or e_cur < best_merge[0]:
                                        best_merge = (e_cur, cur_t, h_idx)
                            if best_merge is not None:
                                _, best_t_m, _ = best_merge
                                cand_list = [tours[k] for k in ne_now if k != a_idx and k != b_idx] + [best_t_m]
                                matched = self._try_hungarian_scout_match(cand_list)
                                if matched is not None and sum(matched[1]) < cur_total_e - 1e-4:
                                    tours, tour_e = matched
                                    improved = True
                                    merged_any = True
                                    break
                if not improved:
                    break

        return tours, tour_e

    def solve_scout_vrptw(
        self,
        hostile_ids: Sequence[str],
        unknown_ids: Sequence[str],
        hostile_priority: Dict[str, float],
        deadline: float,
        alns_iters: int = 12,
        rng: Optional[random.Random] = None,
        seed_tours: Optional[List[List[str]]] = None,
        vnd_passes: int = 5,
        deep_vnd: bool = False,
    ) -> Tuple[List[List[str]], List[float], Set[str]]:
        """Full Scout VRPTW solver: Regret-2 + VND + Hungarian + ALNS."""
        if rng is None:
            rng = random.Random(self.sc.seed + 1337)
        lam = self.r.lambda_e

        def scout_obj(tours: List[List[str]], te: List[float], unserved_h: Set[str]) -> float:
            u_pts = sum(self.contact_map[cid].points for t in tours for cid in t if self.contact_map[cid].cls == UNKNOWN)
            h_pen = sum(hostile_priority.get(cid, 80.0) for cid in unserved_h)
            return u_pts - lam * sum(te) - h_pen

        best_tours, best_e, best_unserved = self._regret2_construct(
            hostile_ids, unknown_ids, hostile_priority, deadline, seed_tours=seed_tours
        )
        best_tours, best_e = self._vnd_scout_routes(best_tours, best_e, unknown_ids, deadline, max_passes=vnd_passes)
        best_val = scout_obj(best_tours, best_e, best_unserved)

        total_stops = sum(len(t) for t in best_tours)
        if total_stops <= 2 or alns_iters <= 0:
            return best_tours, best_e, best_unserved

        cur_tours = [list(t) for t in best_tours]
        cur_e = list(best_e)

        for it in range(alns_iters):
            if time.monotonic() > deadline:
                break
            served = [cid for t in cur_tours for cid in t]
            if len(served) < 3:
                break
            q = max(2, min(len(served) - 1, int(len(served) * rng.uniform(0.2, 0.4))))

            mode = it % 3
            to_remove: Set[str] = set()
            if mode == 0:
                seed_c = rng.choice(served)
                to_remove.add(seed_c)
                served_set = set(served)
                rel = [x for x in self._relatedness.get(seed_c, []) if x in served_set]
                for x in rel[: q - 1]:
                    to_remove.add(x)
            elif mode == 1:
                marginals = []
                for s_idx, t in enumerate(cur_tours):
                    ac = self.scouts[s_idx]
                    for idx, cid in enumerate(t):
                        rem = tuple(t[:idx] + t[idx + 1 :])
                        e_rem = self.eval_scout_tour(ac, rem)
                        saving = (cur_e[s_idx] - e_rem) if e_rem is not None else 0.0
                        marginals.append((saving, cid))
                marginals.sort(reverse=True)
                for _, cid in marginals[:q]:
                    to_remove.add(cid)
            else:
                non_empty = [i for i, t in enumerate(cur_tours) if t]
                if non_empty:
                    pick_r = rng.choice(non_empty)
                    to_remove.update(cur_tours[pick_r])
                    if len(non_empty) > 2 and rng.random() < 0.4:
                        pick_r2 = rng.choice(non_empty)
                        to_remove.update(cur_tours[pick_r2])

            partial = [[cid for cid in t if cid not in to_remove] for t in cur_tours]
            cand_tours, cand_e, cand_unserved = self._regret2_construct(
                hostile_ids, unknown_ids, hostile_priority, deadline,
                rng=rng, noise=1.2, seed_tours=partial,
            )
            cand_tours, cand_e = self._vnd_scout_routes(cand_tours, cand_e, unknown_ids, deadline, max_passes=3)
            cand_val = scout_obj(cand_tours, cand_e, cand_unserved)

            if cand_val > best_val + 1e-4:
                best_val = cand_val
                best_tours = [list(t) for t in cand_tours]
                best_e = list(cand_e)
                best_unserved = set(cand_unserved)
                cur_tours = [list(t) for t in cand_tours]
                cur_e = list(cand_e)
            elif cand_val > best_val - 2.0 and rng.random() < 0.25:
                cur_tours = [list(t) for t in cand_tours]
                cur_e = list(cand_e)

        if deep_vnd and time.monotonic() < deadline:
            best_tours, best_e = self._vnd_scout_routes(
                best_tours, best_e, unknown_ids, deadline, max_passes=3, deep=True
            )
        return best_tours, best_e, best_unserved

    # ------------------------------------------------------------------
    # Joint Evaluation & Deconfliction
    # ------------------------------------------------------------------

    def evaluate_subset(
        self,
        hostile_ids: Sequence[str],
        deadline: float,
        alns_iters: int = 8,
        rng: Optional[random.Random] = None,
        seed_tours: Optional[List[List[str]]] = None,
        vnd_passes: int = 5,
        deep_vnd: bool = False,
    ) -> Optional[dict]:
        """Evaluates a hostile subset S with Exact Hungarian Effectors + Scout VRPTW."""
        eff_res = self.assign_effectors(hostile_ids)
        if eff_res is None:
            return None
        eff_e, eff_by_cid, earliest_by_cid, duals = eff_res

        lam = self.r.lambda_e
        c_eff = self.r.c_eff
        h_prio: Dict[str, float] = {}
        for cid in hostile_ids:
            c = self.contact_map[cid]
            ce = sum(self.eff_cost[a.id][cid][0] for a in eff_by_cid[cid])
            h_prio[cid] = c.points - c_eff * len(eff_by_cid[cid]) - lam * ce

        u_ids = [c.id for c in self.legal_unknowns]
        clean_seed = None
        if seed_tours is not None:
            keep = set(hostile_ids) | set(u_ids)
            clean_seed = [[cid for cid in t if cid in keep] for t in seed_tours]

        tours, tour_e, unserved_h = self.solve_scout_vrptw(
            hostile_ids, u_ids, h_prio, deadline,
            alns_iters=alns_iters, rng=rng, seed_tours=clean_seed, vnd_passes=vnd_passes, deep_vnd=deep_vnd,
        )

        if unserved_h:
            active_h = [cid for cid in hostile_ids if cid not in unserved_h]
            eff_res2 = self.assign_effectors(active_h)
            if eff_res2 is None:
                return None
            eff_e, eff_by_cid, earliest_by_cid, duals = eff_res2
        else:
            active_h = list(hostile_ids)

        served_u = [cid for t in tours for cid in t if self.contact_map[cid].cls == UNKNOWN]
        pts = sum(self.contact_map[cid].points for cid in active_h) + sum(
            self.contact_map[cid].points for cid in served_u
        )
        n_eff = sum(len(v) for v in eff_by_cid.values())
        scout_e = sum(tour_e)
        approx_score = pts - lam * (eff_e + scout_e) - c_eff * n_eff

        return {
            "score": approx_score,
            "hostiles": active_h,
            "unknowns": served_u,
            "tours": tours,
            "tour_e": tour_e,
            "eff_by_cid": eff_by_cid,
            "earliest_by_cid": earliest_by_cid,
            "eff_duals": duals,
            "eff_e": eff_e,
            "scout_e": scout_e,
        }

    def deconflict_and_build_plan(self, sol: dict, deadline: float) -> Tuple[dict, float]:
        """Converts an un-deconflicted solution into a 100% R1-R10 valid plan + post-deconfliction polish."""
        sep = SeparationIndex(self.sc)
        obs_end_by_cid: Dict[str, float] = {}
        placed_scout_tours: Dict[str, List[str]] = {}
        scout_headroom: Dict[str, float] = {}

        # 1. Place scouts sorted by SMALLEST BATTERY HEADROOM first so tight scouts get 0-cost nominal routes!
        scout_entries = [(i, list(sol["tours"][i])) for i in range(len(self.scouts)) if sol["tours"][i]]
        scout_entries.sort(
            key=lambda pair: (self.scouts[pair[0]].usable_wh - sol["tour_e"][pair[0]], -len(pair[1]))
        )

        unplaced_scouts: List[Tuple[int, List[str]]] = []

        for s_idx, cids in scout_entries:
            ac = self.scouts[s_idx]
            contacts = [self.contact_map[cid] for cid in cids]
            res = place_scout_deconflicted(ac, contacts, self.obs_map, self.sc, sep, preferred_z=self.r.z_min_m)
            if res is not None:
                wps, en, o_ends = res
                sep.add(ac.id, wps)
                obs_end_by_cid.update(o_ends)
                placed_scout_tours[ac.id] = cids
                scout_headroom[ac.id] = ac.usable_wh - en
            else:
                # Rip-up & Re-order rescue: if conflict is with a roomier scout, swap placement order!
                ripped_ok = False
                obs_pts = self.obs_map.optimize_disk_tour(ac.home[:2], contacts, self.r.z_min_m)
                nom = build_scout_route(ac, contacts, obs_pts, self.r.z_min_m, self.obs_map, self.sc)
                if nom is not None:
                    cf = sep.first_conflict(ac.id, nom[0])
                    if cf is not None and cf[0] in placed_scout_tours:
                        other_id = cf[0]
                        my_hr = ac.usable_wh - nom[1]
                        if scout_headroom.get(other_id, 0.0) > my_hr + 1.0:
                            old_other_wps = sep.routes_json[other_id]
                            old_other_cids = placed_scout_tours[other_id]
                            sep.remove(other_id)
                            res_me = place_scout_deconflicted(
                                ac, contacts, self.obs_map, self.sc, sep, preferred_z=self.r.z_min_m
                            )
                            if res_me is not None:
                                sep.add(ac.id, res_me[0])
                                res_other = place_scout_deconflicted(
                                    self.aircraft_map[other_id],
                                    [self.contact_map[x] for x in old_other_cids],
                                    self.obs_map, self.sc, sep, preferred_z=self.r.z_min_m,
                                )
                                if res_other is not None:
                                    sep.add(other_id, res_other[0])
                                    obs_end_by_cid.update(res_me[2])
                                    obs_end_by_cid.update(res_other[2])
                                    placed_scout_tours[ac.id] = cids
                                    scout_headroom[ac.id] = ac.usable_wh - res_me[1]
                                    scout_headroom[other_id] = self.aircraft_map[other_id].usable_wh - res_other[1]
                                    ripped_ok = True
                                else:
                                    sep.remove(ac.id)
                                    sep.add(other_id, old_other_wps)
                            else:
                                sep.add(other_id, old_other_wps)
                if not ripped_ok:
                    unplaced_scouts.append((s_idx, cids))

        # Rescue any remaining unplaced scout tour using idle scouts or smart splitting!
        for s_idx, cids in unplaced_scouts:
            ac = self.scouts[s_idx]
            contacts = [self.contact_map[cid] for cid in cids]
            idle_scouts = [a for a in self.scouts if a.id not in placed_scout_tours]
            placed_ok = False

            # Try putting the whole tour on the best feasible idle scout (sorted by energy!)
            idle_scored = []
            for idle_ac in idle_scouts:
                ev = self.eval_scout_tour(idle_ac, tuple(cids))
                if ev is not None:
                    idle_scored.append((ev, idle_ac))
            idle_scored.sort(key=lambda x: x[0])
            for _, idle_ac in idle_scored:
                res = place_scout_deconflicted(idle_ac, contacts, self.obs_map, self.sc, sep, preferred_z=self.r.z_min_m)
                if res is not None:
                    sep.add(idle_ac.id, res[0])
                    obs_end_by_cid.update(res[2])
                    placed_scout_tours[idle_ac.id] = cids
                    scout_headroom[idle_ac.id] = idle_ac.usable_wh - res[1]
                    placed_ok = True
                    break
            if placed_ok:
                continue

            # Try smart tour split across (ac, idle_ac) or (idle1, idle2) over all split points!
            if len(cids) >= 2 and idle_scouts:
                # Candidate pairs of scouts: prioritize keeping original ac + best/largest-battery idle scouts
                other_idles = [a for a in idle_scouts if a.id != ac.id]
                other_idles.sort(key=lambda a: (-a.usable_wh, _min_e_per_m(a)))
                scout_pairs: List[Tuple[Aircraft, Aircraft]] = []
                for oi in other_idles[:3]:
                    scout_pairs.append((ac, oi))
                    scout_pairs.append((oi, ac))
                if len(other_idles) >= 2:
                    scout_pairs.append((other_idles[0], other_idles[1]))

                best_split = None
                for mid in range(1, len(cids)):
                    p1, p2 = cids[:mid], cids[mid:]
                    c1 = [self.contact_map[x] for x in p1]
                    c2 = [self.contact_map[x] for x in p2]
                    for sa, sb in scout_pairs:
                        e1 = self.eval_scout_tour(sa, tuple(p1))
                        if e1 is None:
                            continue
                        e2 = self.eval_scout_tour(sb, tuple(p2))
                        if e2 is None:
                            continue
                        if best_split is not None and e1 + e2 >= best_split[0] - 1e-4:
                            continue
                        r1 = place_scout_deconflicted(sa, c1, self.obs_map, self.sc, sep, preferred_z=self.r.z_min_m)
                        if r1 is None:
                            continue
                        sep.add(sa.id, r1[0])
                        r2 = place_scout_deconflicted(sb, c2, self.obs_map, self.sc, sep, preferred_z=self.r.z_min_m)
                        sep.remove(sa.id)
                        if r2 is not None and (best_split is None or r1[1] + r2[1] < best_split[0]):
                            best_split = (r1[1] + r2[1], sa, p1, r1, sb, p2, r2)
                if best_split is not None:
                    _, sa, p1, r1, sb, p2, r2 = best_split
                    sep.add(sa.id, r1[0])
                    sep.add(sb.id, r2[0])
                    obs_end_by_cid.update(r1[2])
                    obs_end_by_cid.update(r2[2])
                    placed_scout_tours[sa.id] = p1
                    placed_scout_tours[sb.id] = p2
                    placed_ok = True

            if not placed_ok:
                # Last resort: trim lowest-value contact one by one on original scout
                cur_cids = list(cids)
                while len(cur_cids) >= 1:
                    res = place_scout_deconflicted(
                        ac, [self.contact_map[x] for x in cur_cids], self.obs_map, self.sc, sep, preferred_z=self.r.z_min_m
                    )
                    if res is not None:
                        sep.add(ac.id, res[0])
                        obs_end_by_cid.update(res[2])
                        placed_scout_tours[ac.id] = cur_cids
                        break
                    drop_c = min(
                        cur_cids,
                        key=lambda x: (0 if self.contact_map[x].cls == UNKNOWN else 1, self.contact_map[x].points),
                    )
                    cur_cids.remove(drop_c)

        # 2. Place effector groups (tightest slack first)
        eff_by_cid: Dict[str, List[Aircraft]] = sol["eff_by_cid"]
        hostiles_to_place = [cid for cid in sol["hostiles"] if cid in obs_end_by_cid and cid in eff_by_cid]

        def group_hardness(cid: str) -> Tuple[float, int, float]:
            c = self.contact_map[cid]
            effs = eff_by_cid[cid]
            t_min = max(obs_end_by_cid[cid] + self.r.confirm_margin_s, c.window[0])
            for e in effs:
                info = self.eff_cost[e.id].get(cid)
                if info is not None:
                    t_min = max(t_min, info[1])
            slack = c.window[1] - t_min
            return (slack, -len(effs), -c.points)

        hostiles_to_place.sort(key=group_hardness)

        placed_hostiles: Set[str] = set()
        used_effectors: Set[str] = set()
        placed_eff_by_cid: Dict[str, List[Aircraft]] = {}
        failed_hostiles: List[str] = []

        for cid in hostiles_to_place:
            c = self.contact_map[cid]
            effs = eff_by_cid[cid]
            eff_earliest = {e.id: self.eff_cost[e.id][cid][1] for e in effs}
            res = place_effector_group_deconflicted(
                c, effs, obs_end_by_cid[cid], self.obs_map, self.sc, sep, eff_earliest,
                use_direct_z0=self.use_direct_z0,
            )
            if res is not None:
                for aid, wps in res[0].items():
                    sep.add(aid, wps)
                    used_effectors.add(aid)
                placed_hostiles.add(cid)
                placed_eff_by_cid[cid] = list(effs)
            else:
                failed_hostiles.append(cid)

        # 3. Retry failed hostiles with alternative free effectors + scout earlier-shift rescue
        for cid in failed_hostiles:
            if time.monotonic() > deadline:
                break
            c = self.contact_map[cid]
            nl = c.needs.get(LIGHT, 0)
            nh = c.needs.get(HEAVY, 0)
            free_l = sorted(
                [a for a in self.lights if a.id not in used_effectors and self.eff_cost[a.id].get(cid) is not None],
                key=lambda a: self.eff_cost[a.id][cid][0],
            )
            free_h = sorted(
                [a for a in self.heavies if a.id not in used_effectors and self.eff_cost[a.id].get(cid) is not None],
                key=lambda a: self.eff_cost[a.id][cid][0],
            )
            if len(free_l) < nl or len(free_h) < nh:
                continue

            # Try up to 3 distinct free-effector combinations (avoids a single blocked pad!)
            placed_now = False
            for shift_l in range(min(3, max(1, len(free_l) - nl + 1))):
                for shift_h in range(min(2, max(1, len(free_h) - nh + 1))):
                    cand_effs = free_l[shift_l : shift_l + nl] + free_h[shift_h : shift_h + nh]
                    eff_earliest = {e.id: self.eff_cost[e.id][cid][1] for e in cand_effs}
                    res = place_effector_group_deconflicted(
                        c, cand_effs, obs_end_by_cid[cid], self.obs_map, self.sc, sep, eff_earliest,
                        use_direct_z0=self.use_direct_z0,
                    )
                    if res is not None:
                        for aid, wps in res[0].items():
                            sep.add(aid, wps)
                            used_effectors.add(aid)
                        placed_hostiles.add(cid)
                        placed_eff_by_cid[cid] = cand_effs
                        placed_now = True
                        break
                if placed_now:
                    break

            # If still failed and slack after obs_end was tight, try shifting its scout's observation earlier!
            if not placed_now and c.window[1] - (obs_end_by_cid[cid] + self.r.confirm_margin_s) < 45.0:
                owner_scout = next((aid for aid, tc in placed_scout_tours.items() if cid in tc), None)
                if owner_scout is not None:
                    old_wps = sep.routes_json.get(owner_scout)
                    tc = placed_scout_tours[owner_scout]
                    max_obs: Dict[str, float] = {cid: obs_end_by_cid[cid] - 25.0}
                    for rc in tc:
                        if rc != cid and rc in placed_hostiles:
                            min_eng_t = min(
                                wps[-1]["t"]
                                for ea, wps in sep.routes_json.items()
                                if self.aircraft_map[ea].type == EFFECTOR and wps[-1].get("target") == rc
                            )
                            max_obs[rc] = min_eng_t - self.r.confirm_margin_s - 0.05
                    sep.remove(owner_scout)
                    s_res = place_scout_deconflicted(
                        self.aircraft_map[owner_scout],
                        [self.contact_map[x] for x in tc],
                        self.obs_map, self.sc, sep,
                        preferred_z=self.r.z_min_m,
                        max_obs_end_overrides=max_obs,
                    )
                    if s_res is not None:
                        sep.add(owner_scout, s_res[0])
                        cand_effs = free_l[:nl] + free_h[:nh]
                        eff_earliest = {e.id: self.eff_cost[e.id][cid][1] for e in cand_effs}
                        e_res = place_effector_group_deconflicted(
                            c, cand_effs, s_res[2][cid], self.obs_map, self.sc, sep, eff_earliest,
                            use_direct_z0=self.use_direct_z0,
                        )
                        if e_res is not None:
                            for aid, wps in e_res[0].items():
                                sep.add(aid, wps)
                                used_effectors.add(aid)
                            placed_hostiles.add(cid)
                            placed_eff_by_cid[cid] = cand_effs
                            obs_end_by_cid.update(s_res[2])
                        else:
                            sep.remove(owner_scout)
                            if old_wps is not None:
                                sep.add(owner_scout, old_wps)
                    elif old_wps is not None:
                        sep.add(owner_scout, old_wps)

        # 4. Post-deconfliction Rescue: fill any spare effectors with unserved profitable hostiles
        if time.monotonic() < deadline:
            unserved_hostiles = [c for c in self.legal_hostiles if c.id not in placed_hostiles]
            unserved_hostiles.sort(
                key=lambda c: -(c.points - self.r.c_eff * sum(c.needs.values()) - self.r.lambda_e * self.est_eff_e.get(c.id, 10.0))
            )
            for c in unserved_hostiles:
                if time.monotonic() > deadline:
                    break
                nl = c.needs.get(LIGHT, 0)
                nh = c.needs.get(HEAVY, 0)
                free_l = sorted(
                    [a for a in self.lights if a.id not in used_effectors and self.eff_cost[a.id].get(c.id) is not None],
                    key=lambda a: self.eff_cost[a.id][c.id][0],
                )
                free_h = sorted(
                    [a for a in self.heavies if a.id not in used_effectors and self.eff_cost[a.id].get(c.id) is not None],
                    key=lambda a: self.eff_cost[a.id][c.id][0],
                )
                if len(free_l) < nl or len(free_h) < nh:
                    continue
                cand_effs = free_l[:nl] + free_h[:nh]
                est_eff_cost = sum(self.eff_cost[e.id][c.id][0] for e in cand_effs)
                net_before_scout = c.points - self.r.c_eff * (nl + nh) - self.r.lambda_e * est_eff_cost
                if net_before_scout <= 1.0:
                    continue
                if c.id in obs_end_by_cid:
                    eff_earliest = {e.id: self.eff_cost[e.id][c.id][1] for e in cand_effs}
                    res = place_effector_group_deconflicted(
                        c, cand_effs, obs_end_by_cid[c.id], self.obs_map, self.sc, sep, eff_earliest,
                        use_direct_z0=self.use_direct_z0,
                    )
                    if res is not None:
                        for aid, wps in res[0].items():
                            sep.add(aid, wps)
                            used_effectors.add(aid)
                        placed_hostiles.add(c.id)
                else:
                    idle_s = [a for a in self.scouts if a.id not in placed_scout_tours]
                    scored_idle = []
                    for ac in idle_s:
                        e_solo = self.eval_scout_tour(ac, (c.id,))
                        if e_solo is not None and net_before_scout - self.r.lambda_e * e_solo > 0.5:
                            scored_idle.append((e_solo, ac))
                    scored_idle.sort(key=lambda x: x[0])
                    for _, ac in scored_idle:
                        s_res = place_scout_deconflicted(ac, [c], self.obs_map, self.sc, sep, preferred_z=self.r.z_min_m)
                        if s_res is not None and net_before_scout - self.r.lambda_e * s_res[1] > 0.5:
                            sep.add(ac.id, s_res[0])
                            eff_earliest = {e.id: self.eff_cost[e.id][c.id][1] for e in cand_effs}
                            e_res = place_effector_group_deconflicted(
                                c, cand_effs, s_res[2][c.id], self.obs_map, self.sc, sep, eff_earliest,
                                use_direct_z0=self.use_direct_z0,
                            )
                            if e_res is not None:
                                for aid, wps in e_res[0].items():
                                    sep.add(aid, wps)
                                    used_effectors.add(aid)
                                placed_hostiles.add(c.id)
                                obs_end_by_cid.update(s_res[2])
                                placed_scout_tours[ac.id] = [c.id]
                                break
                            else:
                                sep.remove(ac.id)

        # 5. Clean up dead scout observations of unengaged hostiles
        dead_hostiles = [cid for cid in obs_end_by_cid if self.contact_map[cid].cls == HOSTILE and cid not in placed_hostiles]
        if dead_hostiles:
            dead_set = set(dead_hostiles)
            for aid, cids in list(placed_scout_tours.items()):
                if any(cid in dead_set for cid in cids):
                    new_cids = [cid for cid in cids if cid not in dead_set]
                    old_wps = sep.routes_json.get(aid)
                    sep.remove(aid)
                    if not new_cids:
                        placed_scout_tours.pop(aid, None)
                    else:
                        ac = self.aircraft_map[aid]
                        max_obs = {}
                        for rc in new_cids:
                            if rc in placed_hostiles:
                                min_eng_t = min(
                                    wps[-1]["t"]
                                    for ea, wps in sep.routes_json.items()
                                    if self.aircraft_map[ea].type == EFFECTOR and wps[-1].get("target") == rc
                                )
                                max_obs[rc] = min_eng_t - self.r.confirm_margin_s - 0.05
                        res = place_scout_deconflicted(
                            ac, [self.contact_map[x] for x in new_cids], self.obs_map, self.sc, sep,
                            preferred_z=self.r.z_min_m, max_obs_end_overrides=max_obs,
                        )
                        if res is not None:
                            sep.add(aid, res[0])
                            placed_scout_tours[aid] = new_cids
                        elif old_wps is not None:
                            sep.add(aid, old_wps)

        # 6. Post-deconfliction unknown fill on idle scouts (sorted by best solo energy, incl. large-battery scouts!)
        if time.monotonic() < deadline:
            served_u_now = {cid for cids in placed_scout_tours.values() for cid in cids if self.contact_map[cid].cls == UNKNOWN}
            for u in sorted(self.legal_unknowns, key=lambda c: -c.points):
                if u.id in served_u_now or time.monotonic() > deadline:
                    continue
                scored_idle = []
                for ac in self.scouts:
                    if ac.id in placed_scout_tours:
                        continue
                    e_solo = self.eval_scout_tour(ac, (u.id,))
                    if e_solo is not None and u.points - self.r.lambda_e * e_solo > 0.5:
                        scored_idle.append((e_solo, ac))
                scored_idle.sort(key=lambda x: x[0])
                for _, ac in scored_idle:
                    res = place_scout_deconflicted(ac, [u], self.obs_map, self.sc, sep, preferred_z=self.r.z_min_m)
                    if res is not None and u.points - self.r.lambda_e * res[1] > 0.5:
                        sep.add(ac.id, res[0])
                        placed_scout_tours[ac.id] = [u.id]
                        served_u_now.add(u.id)
                        break

        plan = verify_and_repair_plan(self.sc, sep.routes_json)
        result = evaluate(self.sc, plan)
        return plan, (result.score if result.valid else 0.0)

    # ------------------------------------------------------------------
    # Outer Joint Hostile Subset Optimization + Exact Hungarian Dual Pricing + Swaps
    # ------------------------------------------------------------------

    def _knapsack_candidates(
        self,
        marginal_scout: Dict[str, float],
        marginal_eff: Dict[str, float],
        jitter: float = 0.0,
        rng: Optional[random.Random] = None,
    ) -> List[str]:
        lam = self.r.lambda_e
        c_eff = self.r.c_eff
        items: List[Tuple[int, int, float]] = []
        for c in self.legal_hostiles:
            nl = c.needs.get(LIGHT, 0)
            nh = c.needs.get(HEAVY, 0)
            val = c.points - c_eff * (nl + nh) - lam * (marginal_eff[c.id] + marginal_scout[c.id])
            if jitter > 0.0 and rng is not None:
                val += rng.uniform(-jitter, jitter)
            items.append((nl, nh, val))
        _, chosen_idx = knapsack_2d(items, len(self.lights), len(self.heavies))
        return [self.legal_hostiles[i].id for i in chosen_idx]

    def _compute_true_marginals(self, sol: dict) -> Tuple[Dict[str, float], Dict[str, float]]:
        """Computes exact marginal scout & Hungarian-dual effector energies for EVERY legal hostile."""
        active_set = set(sol["hostiles"])
        tours = sol["tours"]
        tour_e = sol["tour_e"]
        eff_by_cid = sol["eff_by_cid"]
        u_by_cid, v_light, v_heavy = sol["eff_duals"]
        m = len(self.scouts)

        m_scout: Dict[str, float] = {}
        m_eff: Dict[str, float] = {}

        cid_to_scout: Dict[str, int] = {}
        for s_idx, t in enumerate(tours):
            for cid in t:
                cid_to_scout[cid] = s_idx

        eval_scouts = [i for i in range(m) if tours[i]] + self._candidate_empty_scouts(tours)

        for c in self.legal_hostiles:
            if c.id in active_set:
                # Exact Hungarian dual marginal cost (captures displacement of cheap effectors!)
                m_eff[c.id] = u_by_cid.get(c.id, sum(self.eff_cost[a.id][c.id][0] for a in eff_by_cid[c.id]))
                s_idx = cid_to_scout.get(c.id)
                if s_idx is not None:
                    rem = tuple(x for x in tours[s_idx] if x != c.id)
                    e_rem = self.eval_scout_tour(self.scouts[s_idx], rem)
                    m_scout[c.id] = max(0.0, tour_e[s_idx] - e_rem) if e_rem is not None else self.solo_scout_e[c.id]
                else:
                    m_scout[c.id] = self.solo_scout_e[c.id]
            else:
                # Exact 1st-order augmenting-path marginal effector energy from Hungarian column duals!
                nl = c.needs.get(LIGHT, 0)
                nh = c.needs.get(HEAVY, 0)
                me = 0.0
                if nl:
                    rc_l = sorted(
                        self.eff_cost[a.id][c.id][0] - v_light[j]
                        for j, a in enumerate(self.lights)
                        if self.eff_cost[a.id].get(c.id) is not None
                    )
                    me += sum(rc_l[:nl]) if len(rc_l) >= nl else INF
                if nh:
                    rc_h = sorted(
                        self.eff_cost[a.id][c.id][0] - v_heavy[j]
                        for j, a in enumerate(self.heavies)
                        if self.eff_cost[a.id].get(c.id) is not None
                    )
                    me += sum(rc_h[:nh]) if len(rc_h) >= nh else INF
                m_eff[c.id] = me

                best_de = self.solo_scout_e[c.id]
                for s_idx in eval_scouts:
                    de, _ = self._best_insertion_in_tour(self.scouts[s_idx], tours[s_idx], tour_e[s_idx], c.id)
                    if de < best_de:
                        best_de = de
                m_scout[c.id] = best_de

        return m_scout, m_eff

    def _hostile_swap_local_search(
        self,
        sol: dict,
        deadline: float,
        rng: random.Random,
    ) -> dict:
        """Fast warm-started Drop / Add / 1-1 / 1-2 / 2-1 Swap local search on hostile subset."""
        best = sol
        cap_l = len(self.lights)
        cap_h = len(self.heavies)
        lam = self.r.lambda_e
        c_eff = self.r.c_eff

        for _round in range(6):
            if time.monotonic() > deadline:
                break
            improved = False
            active = list(best["hostiles"])
            active_set = set(active)
            inactive = [c.id for c in self.legal_hostiles if c.id not in active_set]
            used_l = sum(self.contact_map[cid].needs.get(LIGHT, 0) for cid in active)
            used_h = sum(self.contact_map[cid].needs.get(HEAVY, 0) for cid in active)

            m_scout, m_eff = self._compute_true_marginals(best)

            def net_est(cid: str) -> float:
                c = self.contact_map[cid]
                return c.points - c_eff * (c.needs.get(LIGHT, 0) + c.needs.get(HEAVY, 0)) - lam * (m_eff[cid] + m_scout[cid])

            # 1. Drop unprofitable hostile
            for cid in sorted(active, key=net_est):
                if time.monotonic() > deadline or net_est(cid) > 3.5:
                    break
                cand_h = [x for x in active if x != cid]
                cand = self.evaluate_subset(
                    cand_h, deadline, alns_iters=0, rng=rng, seed_tours=best["tours"], vnd_passes=2
                )
                if cand is not None and cand["score"] > best["score"] + 1e-3:
                    best = cand
                    improved = True
                    break
            if improved:
                continue

            # 2. Add inactive hostile
            add_cands = [
                cid for cid in inactive
                if used_l + self.contact_map[cid].needs.get(LIGHT, 0) <= cap_l
                and used_h + self.contact_map[cid].needs.get(HEAVY, 0) <= cap_h
            ]
            add_cands.sort(key=lambda cid: -net_est(cid))
            for cid in add_cands[:12]:
                if time.monotonic() > deadline or net_est(cid) < -5.0:
                    break
                cand = self.evaluate_subset(
                    active + [cid], deadline, alns_iters=0, rng=rng, seed_tours=best["tours"], vnd_passes=2
                )
                if cand is not None and cand["score"] > best["score"] + 1e-3:
                    best = cand
                    improved = True
                    break
            if improved:
                continue

            # 3. 1-for-1 Swap (warm-started -> super fast!)
            swaps_11: List[Tuple[float, str, str]] = []
            for c_out in active:
                co = self.contact_map[c_out]
                ol, oh = co.needs.get(LIGHT, 0), co.needs.get(HEAVY, 0)
                vo = net_est(c_out)
                for c_in in inactive:
                    ci = self.contact_map[c_in]
                    il, ih = ci.needs.get(LIGHT, 0), ci.needs.get(HEAVY, 0)
                    if used_l - ol + il <= cap_l and used_h - oh + ih <= cap_h:
                        delta = net_est(c_in) - vo
                        if delta > -6.0:
                            swaps_11.append((delta, c_out, c_in))
            swaps_11.sort(reverse=True)
            for _, c_out, c_in in swaps_11[:20]:
                if time.monotonic() > deadline:
                    break
                cand_h = [x for x in active if x != c_out] + [c_in]
                cand = self.evaluate_subset(
                    cand_h, deadline, alns_iters=0, rng=rng, seed_tours=best["tours"], vnd_passes=2
                )
                if cand is not None and cand["score"] > best["score"] + 1e-3:
                    best = cand
                    improved = True
                    break
            if improved:
                continue

            # 4. 1-for-2 Swap
            if len(inactive) >= 2:
                top_in = sorted(inactive, key=lambda cid: -net_est(cid))[:16]
                swaps_12: List[Tuple[float, str, str, str]] = []
                for c_out in sorted(active, key=net_est)[:14]:
                    co = self.contact_map[c_out]
                    ol, oh = co.needs.get(LIGHT, 0), co.needs.get(HEAVY, 0)
                    if ol + oh < 2:
                        continue
                    vo = net_est(c_out)
                    for i1 in range(len(top_in)):
                        c1 = self.contact_map[top_in[i1]]
                        l1, h1 = c1.needs.get(LIGHT, 0), c1.needs.get(HEAVY, 0)
                        v1 = net_est(top_in[i1])
                        for i2 in range(i1 + 1, len(top_in)):
                            c2 = self.contact_map[top_in[i2]]
                            if used_l - ol + l1 + c2.needs.get(LIGHT, 0) <= cap_l and used_h - oh + h1 + c2.needs.get(HEAVY, 0) <= cap_h:
                                delta = v1 + net_est(top_in[i2]) - vo
                                if delta > -4.0:
                                    swaps_12.append((delta, c_out, top_in[i1], top_in[i2]))
                swaps_12.sort(reverse=True)
                for _, c_out, c1, c2 in swaps_12[:10]:
                    if time.monotonic() > deadline:
                        break
                    cand_h = [x for x in active if x != c_out] + [c1, c2]
                    cand = self.evaluate_subset(
                        cand_h, deadline, alns_iters=0, rng=rng, seed_tours=best["tours"], vnd_passes=2
                    )
                    if cand is not None and cand["score"] > best["score"] + 1e-3:
                        best = cand
                        improved = True
                        break
            if improved:
                continue

            # 5. 2-for-1 Swap (replace 2 weaker hostiles with 1 stronger multi-effector hostile)
            if len(active) >= 2 and inactive:
                bot_out = sorted(active, key=net_est)[:12]
                top_in = sorted(inactive, key=lambda cid: -net_est(cid))[:12]
                swaps_21: List[Tuple[float, str, str, str]] = []
                for c_in in top_in:
                    ci = self.contact_map[c_in]
                    il, ih = ci.needs.get(LIGHT, 0), ci.needs.get(HEAVY, 0)
                    vi = net_est(c_in)
                    for i1 in range(len(bot_out)):
                        c1 = self.contact_map[bot_out[i1]]
                        l1, h1 = c1.needs.get(LIGHT, 0), c1.needs.get(HEAVY, 0)
                        v1 = net_est(bot_out[i1])
                        for i2 in range(i1 + 1, len(bot_out)):
                            c2 = self.contact_map[bot_out[i2]]
                            if used_l - l1 - c2.needs.get(LIGHT, 0) + il <= cap_l and used_h - h1 - c2.needs.get(HEAVY, 0) + ih <= cap_h:
                                delta = vi - (v1 + net_est(bot_out[i2]))
                                if delta > -4.0:
                                    swaps_21.append((delta, bot_out[i1], bot_out[i2], c_in))
                swaps_21.sort(reverse=True)
                for _, o1, o2, c_in in swaps_21[:10]:
                    if time.monotonic() > deadline:
                        break
                    cand_h = [x for x in active if x != o1 and x != o2] + [c_in]
                    cand = self.evaluate_subset(
                        cand_h, deadline, alns_iters=0, rng=rng, seed_tours=best["tours"], vnd_passes=2
                    )
                    if cand is not None and cand["score"] > best["score"] + 1e-3:
                        best = cand
                        improved = True
                        break

            if not improved:
                break

        return best

    def solve(self, time_budget_s: float = 45.0) -> dict:
        """Main anytime solver entry point, strictly respecting time_budget_s."""
        t_start = time.monotonic()
        hard_deadline = t_start + time_budget_s
        search_deadline = t_start + time_budget_s * 0.72

        rng = random.Random(self.sc.seed * 1009 + 42)
        n_h = len(self.legal_hostiles)
        n_u = len(self.legal_unknowns)

        if n_h == 0 and n_u == 0:
            return {"seed": self.sc.seed, "routes": []}

        alns_base = 16 if n_h <= 15 else 8

        seen_subsets: Set[frozenset] = set()
        top_sols: List[dict] = []

        def record_sol(sol: Optional[dict]) -> None:
            if sol is None:
                return
            top_sols.append(sol)
            top_sols.sort(key=lambda s: -s["score"])
            if len(top_sols) > 4:
                del top_sols[4:]

        # Stage 1: Initial Exact 2D Knapsack seeds across scout-sharing factors
        alphas = (0.20, 0.45, 0.80, 0.05) if n_h > 6 else (0.15, 0.50, 1.0)
        for alpha in alphas:
            if time.monotonic() > search_deadline:
                break
            m_scout = {c.id: alpha * self.solo_scout_e[c.id] + 0.45 for c in self.legal_hostiles}
            subset = self._knapsack_candidates(m_scout, self.est_eff_e)
            key = frozenset(subset)
            if key in seen_subsets:
                continue
            seen_subsets.add(key)
            sol = self.evaluate_subset(subset, search_deadline, alns_iters=alns_base, rng=rng)
            record_sol(sol)

        # Exhaustive top-subset enumeration for small scenarios (L1/L2)
        if n_h <= 8 and time.monotonic() < search_deadline:
            cap_l = len(self.lights)
            cap_h = len(self.heavies)
            all_cands = []
            for mask in range(1, 1 << n_h):
                nl = nh = 0
                pts = 0.0
                for i in range(n_h):
                    if (mask >> i) & 1:
                        c = self.legal_hostiles[i]
                        nl += c.needs.get(LIGHT, 0)
                        nh += c.needs.get(HEAVY, 0)
                        pts += c.points - self.r.c_eff * (c.needs.get(LIGHT, 0) + c.needs.get(HEAVY, 0)) - self.r.lambda_e * self.est_eff_e[c.id]
                if nl <= cap_l and nh <= cap_h:
                    all_cands.append((pts, mask))
            all_cands.sort(reverse=True)
            for _, mask in all_cands[:12]:
                if time.monotonic() > search_deadline:
                    break
                subset = [self.legal_hostiles[i].id for i in range(n_h) if (mask >> i) & 1]
                key = frozenset(subset)
                if key in seen_subsets:
                    continue
                seen_subsets.add(key)
                sol = self.evaluate_subset(subset, search_deadline, alns_iters=alns_base, rng=rng)
                record_sol(sol)

        if not top_sols:
            sol = self.evaluate_subset([], search_deadline, alns_iters=alns_base, rng=rng)
            record_sol(sol)

        # Stage 2: Iterative Pricing (TRUE marginal scout & Hungarian-dual effector costs -> Exact 2D Knapsack)
        if top_sols and n_h > 0:
            for it in range(10):
                if time.monotonic() > search_deadline:
                    break
                cur_best = top_sols[0]
                m_scout, m_eff = self._compute_true_marginals(cur_best)
                jitter = 0.0 if it == 0 else (2.0 + 1.5 * it)
                subset = self._knapsack_candidates(m_scout, m_eff, jitter=jitter, rng=rng)
                key = frozenset(subset)
                if key in seen_subsets:
                    continue
                seen_subsets.add(key)
                sol = self.evaluate_subset(subset, search_deadline, alns_iters=alns_base, rng=rng)
                record_sol(sol)

        # Stage 3: Fast Warm-Started Hostile Swap Local Search (Drop / Add / 1-1 / 1-2 / 2-1)
        if top_sols and time.monotonic() < search_deadline:
            improved = self._hostile_swap_local_search(top_sols[0], search_deadline, rng)
            if improved["score"] > top_sols[0]["score"] + 1e-4:
                record_sol(improved)

        # Stage 4: Deep ALNS + deep VND polish on the top-1 solution's scout routes
        if top_sols and time.monotonic() < search_deadline:
            deep_iters = 45 if n_h <= 15 else 35
            polished = self.evaluate_subset(
                top_sols[0]["hostiles"], search_deadline,
                alns_iters=deep_iters, rng=rng, seed_tours=top_sols[0]["tours"], deep_vnd=True,
            )
            if polished is not None and polished["score"] > top_sols[0]["score"] + 1e-4:
                record_sol(polished)

        # Stage 5: Deconflict top distinct candidate solutions & pick best 100%-validated plan!
        best_plan: dict = {"seed": self.sc.seed, "routes": []}
        best_valid_score: float = 0.0

        dedup_sols: List[dict] = []
        seen_keys: Set[Tuple[frozenset, float]] = set()
        for s in top_sols:
            k = (frozenset(s["hostiles"]), round(s["score"], 2))
            if k not in seen_keys:
                seen_keys.add(k)
                dedup_sols.append(s)

        for idx, sol in enumerate(dedup_sols[:3]):
            if idx > 0 and time.monotonic() > hard_deadline - 2.0:
                break
            plan, score = self.deconflict_and_build_plan(sol, hard_deadline)
            if score > best_valid_score:
                best_valid_score = score
                best_plan = plan

        # Stage 6: Anytime extra perturbed restarts if time remains on larger scenarios
        if n_h > 8 and time.monotonic() < t_start + time_budget_s * 0.78 and top_sols:
            extra_deadline = t_start + time_budget_s * 0.88
            cur_best = top_sols[0]
            m_scout, m_eff = self._compute_true_marginals(cur_best)
            for p_it in range(12):
                if time.monotonic() > extra_deadline:
                    break
                subset = self._knapsack_candidates(m_scout, m_eff, jitter=4.5 + 2.0 * p_it, rng=rng)
                key = frozenset(subset)
                if key in seen_subsets:
                    continue
                seen_subsets.add(key)
                sol = self.evaluate_subset(subset, extra_deadline, alns_iters=10, rng=rng)
                if sol is not None and sol["score"] > best_valid_score + 0.5:
                    sol = self._hostile_swap_local_search(sol, extra_deadline, rng)
                    plan, score = self.deconflict_and_build_plan(sol, hard_deadline)
                    if score > best_valid_score:
                        best_valid_score = score
                        best_plan = plan
                        top_sols.insert(0, sol)
                        m_scout, m_eff = self._compute_true_marginals(sol)

        return best_plan


def solve_scenario(scenario: Scenario, time_budget_s: float = 45.0, use_direct_z0: bool = True) -> dict:
    """Solves a Scenario and returns a 100%-validated plan dict."""
    solver = FleetSolver(scenario, use_direct_z0=use_direct_z0)
    return solver.solve(time_budget_s=time_budget_s)
