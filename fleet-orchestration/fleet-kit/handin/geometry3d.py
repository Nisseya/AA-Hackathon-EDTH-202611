"""Exact 2D/3D obstacle-aware routing and observation/engagement point optimization."""
from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Sequence, Tuple

from fleetkit.energy import best_range_speed
from fleetkit.geometry import INF, hdist, segment_enters_cylinder
from fleetkit.model import CIVILIAN, FRIENDLY, Aircraft, Contact, Scenario

Point2 = Tuple[float, float]
Point3 = Tuple[float, float, float]
# (cx, cy, radius_m, top_m)
Cylinder = Tuple[float, float, float, float]

OBS_RADIUS_PAD = 0.5
ZONE_TOP_PAD = 0.15
POLY_SIDES = 16
POLY_SCALE = 1.0 / math.cos(math.pi / POLY_SIDES)


class ObstacleMap:
    """Precomputed obstacles and cached 2D/3D minimum-energy path queries."""

    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.rules = scenario.rules
        self.protected: List[Cylinder] = []
        self.zones: List[Cylinder] = []
        for c in scenario.contacts:
            if c.cls in (FRIENDLY, CIVILIAN):
                self.protected.append((c.pos[0], c.pos[1], self.rules.protect_radius_m, INF))
        for z in scenario.zones:
            self.zones.append((z.center[0], z.center[1], z.radius_m, z.top_m))
        self.all_cylinders: List[Cylinder] = self.zones + self.protected

        # Precompute circumscribed polygon vertices for every cylinder (with safety pad)
        self._poly_verts: List[List[Point2]] = []
        for cx, cy, r, top in self.all_cylinders:
            r_eff = r + OBS_RADIUS_PAD
            r_poly = r_eff * POLY_SCALE + 0.2
            verts = []
            for k in range(POLY_SIDES):
                ang = 2.0 * math.pi * k / POLY_SIDES
                verts.append((cx + r_poly * math.cos(ang), cy + r_poly * math.sin(ang)))
            self._poly_verts.append(verts)

        # Coarse-region cache for 2D detour waypoints (reuses intermediate vertices across nearby endpoints)
        self._path2d_cache: Dict[Tuple[int, int, int, int, int], List[Point2]] = {}

    def segment_hits_cyl_2d(self, a: Point2, b: Point2, cyl_idx: int, pad: float = OBS_RADIUS_PAD) -> bool:
        cx, cy, r, _ = self.all_cylinders[cyl_idx]
        r_eff = r + pad
        ax, ay = a[0], a[1]
        bx, by = b[0], b[1]
        if (ax if ax > bx else bx) < cx - r_eff or (ax if ax < bx else bx) > cx + r_eff:
            return False
        if (ay if ay > by else by) < cy - r_eff or (ay if ay < by else by) > cy + r_eff:
            return False
        dx, dy = bx - ax, by - ay
        fx, fy = ax - cx, ay - cy
        qa = dx * dx + dy * dy
        qc = fx * fx + fy * fy - r_eff * r_eff
        if qa < 1e-12:
            return qc < 0.0
        qb = 2.0 * (fx * dx + fy * dy)
        disc = qb * qb - 4.0 * qa * qc
        if disc <= 0.0:
            return False
        root = math.sqrt(disc)
        inv2a = 0.5 / qa
        lo = (-qb - root) * inv2a
        hi = (-qb + root) * inv2a
        if lo < 0.0:
            lo = 0.0
        if hi > 1.0:
            hi = 1.0
        return hi - lo > 1e-9

    def point_inside_any(self, p: Point2, blocked_indices: Sequence[int], pad: float = OBS_RADIUS_PAD) -> bool:
        px, py = p[0], p[1]
        cyls = self.all_cylinders
        for idx in blocked_indices:
            cx, cy, r, _ = cyls[idx]
            r_eff = r + pad
            dx, dy = px - cx, py - cy
            if dx * dx + dy * dy < r_eff * r_eff:
                return True
        return False

    def blocked_indices_at_z(self, z: float) -> List[int]:
        """Indices of all cylinders that cannot be crossed at altitude z."""
        return [i for i, (_, _, _, top) in enumerate(self.all_cylinders) if z < top + 1e-3]

    def segment_valid_3d(self, p0: Point3, p1: Point3) -> bool:
        """Exact validator check (with tiny safety pad) against all cylinders."""
        x0, y0, z0 = p0
        x1, y1, z1 = p1
        xmin = x0 if x0 < x1 else x1
        xmax = x1 if x0 < x1 else x0
        ymin = y0 if y0 < y1 else y1
        ymax = y1 if y0 < y1 else y0
        zmin = z0 if z0 < z1 else z1
        for cx, cy, r, top in self.all_cylinders:
            if zmin >= top:
                continue
            re = r + 0.05
            if xmax < cx - re or xmin > cx + re or ymax < cy - re or ymin > cy + re:
                continue
            if segment_enters_cylinder(p0, p1, (cx, cy), re, top):
                return False
        return True

    def polyline_valid_3d(self, pts: Sequence[Point3]) -> bool:
        for i in range(len(pts) - 1):
            if not self.segment_valid_3d(pts[i], pts[i + 1]):
                return False
        return True

    def _visible_2d(self, a: Point2, b: Point2, blocked: Sequence[int]) -> bool:
        for idx in blocked:
            if self.segment_hits_cyl_2d(a, b, idx, pad=OBS_RADIUS_PAD):
                return False
        return True

    def _tangent_points(self, p: Point2, cyl_idx: int) -> List[Point2]:
        cx, cy, r, _ = self.all_cylinders[cyl_idx]
        r_eff = r + OBS_RADIUS_PAD + 0.25
        dx, dy = p[0] - cx, p[1] - cy
        d2 = dx * dx + dy * dy
        if d2 <= r_eff * r_eff + 1e-6:
            return []
        d = math.sqrt(d2)
        base_ang = math.atan2(dy, dx)
        alpha = math.acos(min(1.0, r_eff / d))
        r_out = r_eff + 0.2
        return [
            (cx + r_out * math.cos(base_ang + alpha), cy + r_out * math.sin(base_ang + alpha)),
            (cx + r_out * math.cos(base_ang - alpha), cy + r_out * math.sin(base_ang - alpha)),
        ]

    def shortest_path_2d(self, a: Point2, b: Point2, blocked: Sequence[int]) -> Optional[List[Point2]]:
        """Exact shortest 2D polyline from a to b avoiding all cylinders in `blocked`."""
        if not blocked:
            return [a, b]
        hit = [idx for idx in blocked if self.segment_hits_cyl_2d(a, b, idx, pad=OBS_RADIUS_PAD)]
        if not hit:
            return [a, b]

        mask = 0
        for idx in blocked:
            mask |= (1 << idx)

        # Fast coarse-region cache: reuse intermediate detour vertices across nearby pads / obs points
        coarse_key = (int(a[0] // 160), int(a[1] // 160), int(b[0] // 160), int(b[1] // 160), mask)
        cached = self._path2d_cache.get(coarse_key)
        if cached is not None and len(cached) >= 3:
            if self._visible_2d(a, cached[1], blocked) and self._visible_2d(cached[-2], b, blocked):
                cand = [a] + cached[1:-1] + [b]
                return self._shortcut_2d(cand, blocked)

        # Iteratively expand active obstacles near the corridor
        active_set = set(hit)
        changed = True
        while changed:
            changed = False
            for idx in blocked:
                if idx in active_set:
                    continue
                cx, cy, r, _ = self.all_cylinders[idx]
                for a_idx in list(active_set):
                    ax, ay, ar, _ = self.all_cylinders[a_idx]
                    if math.hypot(cx - ax, cy - ay) <= r + ar + 220.0:
                        active_set.add(idx)
                        changed = True
                        break

        for _attempt in range(2):
            nodes: List[Point2] = [a, b]
            for idx in active_set:
                for v in self._poly_verts[idx]:
                    if not self.point_inside_any(v, blocked, pad=OBS_RADIUS_PAD):
                        nodes.append(v)
                for tp in self._tangent_points(a, idx) + self._tangent_points(b, idx):
                    if not self.point_inside_any(tp, blocked, pad=OBS_RADIUS_PAD):
                        nodes.append(tp)

            n = len(nodes)
            bx, by = b[0], b[1]
            h_to_b = [math.hypot(nodes[i][0] - bx, nodes[i][1] - by) for i in range(n)]
            g_dist = [INF] * n
            parent = [-1] * n
            g_dist[0] = 0.0
            # A* search with lazy edge visibility checks
            heap: List[Tuple[float, float, int]] = [(h_to_b[0], 0.0, 0)]
            closed = [False] * n

            while heap:
                _, d_u, u = heapq.heappop(heap)
                if closed[u]:
                    continue
                closed[u] = True
                if u == 1:
                    break
                pu = nodes[u]
                ux, uy = pu[0], pu[1]
                for v in range(1, n):
                    if closed[v]:
                        continue
                    pv = nodes[v]
                    w = math.hypot(pv[0] - ux, pv[1] - uy)
                    nd = d_u + w
                    if nd < g_dist[v] - 1e-6:
                        if self._visible_2d(pu, pv, blocked):
                            g_dist[v] = nd
                            parent[v] = u
                            heapq.heappush(heap, (nd + h_to_b[v], nd, v))

            if parent[1] != -1:
                path: List[Point2] = []
                cur = 1
                while cur != -1:
                    path.append(nodes[cur])
                    cur = parent[cur]
                path.reverse()
                path = self._shortcut_2d(path, blocked)
                self._path2d_cache[coarse_key] = path
                return path

            if len(active_set) < len(blocked):
                active_set = set(blocked)
            else:
                break

        return None

    def _shortcut_2d(self, path: List[Point2], blocked: Sequence[int]) -> List[Point2]:
        if len(path) <= 2:
            return path
        out = [path[0]]
        i = 0
        n = len(path)
        while i < n - 1:
            j = n - 1
            while j > i + 1:
                if self._visible_2d(path[i], path[j], blocked):
                    break
                j -= 1
            out.append(path[j])
            i = j
        return out

    def _estimate_polyline_energy(self, pts: Sequence[Point3], ac: Aircraft) -> Tuple[float, float]:
        """Returns (min_energy_wh, min_time_s) for flying a 3D polyline at optimal speed."""
        p = ac.power
        v_opt = min(ac.v_h_mps, best_range_speed(p))
        total_e = 0.0
        total_t = 0.0
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            dh = math.hypot(b[0] - a[0], b[1] - a[1])
            dz = b[2] - a[2]
            t_min = dh / ac.v_h_mps
            if dz > 0.0:
                t_min = max(t_min, dz / ac.v_up_mps)
            elif dz < 0.0:
                t_min = max(t_min, -dz / ac.v_down_mps)
            t_use = max(t_min, dh / v_opt if v_opt > 1e-9 else t_min)
            if t_use > 1e-9:
                v = dh / t_use
                total_e += (p.p0_w + p.k_v * v * v) * t_use / 3600.0
            if dz > 0.0:
                total_e += p.e_climb_wh_per_m * dz
            total_t += t_use
        return total_e, total_t

    def _overflight_polyline(
        self,
        a: Point3,
        b: Point3,
        z_over: float,
        z_mid_floor: float,
    ) -> Optional[List[Point3]]:
        """Builds a 3D polyline from a to b overflying all zones with top_m < z_over."""
        blocked = [i for i, (_, _, _, top) in enumerate(self.all_cylinders) if top > z_over - 0.05]
        path_2d = self.shortest_path_2d(a[:2], b[:2], blocked)
        if path_2d is None:
            return None

        over_zones = [i for i, (_, _, _, top) in enumerate(self.all_cylinders) if top <= z_over - 0.05]
        cum_len = [0.0]
        for i in range(len(path_2d) - 1):
            cum_len.append(cum_len[-1] + hdist(path_2d[i], path_2d[i + 1]))
        total_len = cum_len[-1]
        if total_len < 1e-6:
            return [a, b]

        first_s = INF
        last_s = -1.0
        for i in range(len(path_2d) - 1):
            p0, p1 = path_2d[i], path_2d[i + 1]
            seg_len = cum_len[i + 1] - cum_len[i]
            if seg_len < 1e-9:
                continue
            dx, dy = p1[0] - p0[0], p1[1] - p0[1]
            qa = dx * dx + dy * dy
            for zi in over_zones:
                cx, cy, r, _ = self.all_cylinders[zi]
                r_eff = r + OBS_RADIUS_PAD
                fx, fy = p0[0] - cx, p0[1] - cy
                qb = 2.0 * (fx * dx + fy * dy)
                qc = fx * fx + fy * fy - r_eff * r_eff
                disc = qb * qb - 4.0 * qa * qc
                if disc <= 0.0:
                    continue
                root = math.sqrt(disc)
                lo = max((-qb - root) / (2.0 * qa), 0.0)
                hi = min((-qb + root) / (2.0 * qa), 1.0)
                if hi - lo > 1e-9:
                    first_s = min(first_s, cum_len[i] + lo * seg_len)
                    last_s = max(last_s, cum_len[i] + hi * seg_len)

        if first_s >= last_s:
            pts3: List[Point3] = [a]
            for p in path_2d[1:-1]:
                pts3.append((p[0], p[1], max(z_mid_floor, min(a[2], b[2]))))
            pts3.append(b)
            return pts3 if self.polyline_valid_3d(pts3) else None

        s_in = max(0.0, first_s - 0.5)
        s_out = min(total_len, last_s + 0.5)

        def interp_at(s: float) -> Point2:
            for i in range(len(path_2d) - 1):
                if s <= cum_len[i + 1] + 1e-9:
                    seg = cum_len[i + 1] - cum_len[i]
                    f = 0.0 if seg < 1e-9 else max(0.0, min(1.0, (s - cum_len[i]) / seg))
                    return (
                        path_2d[i][0] + f * (path_2d[i + 1][0] - path_2d[i][0]),
                        path_2d[i][1] + f * (path_2d[i + 1][1] - path_2d[i][1]),
                    )
            return path_2d[-1]

        pts3 = [a]
        for i in range(1, len(path_2d) - 1):
            if cum_len[i] < s_in - 1.0:
                pts3.append((path_2d[i][0], path_2d[i][1], max(z_mid_floor, a[2])))
        if s_in > 1.0:
            p_in = interp_at(s_in)
            pts3.append((p_in[0], p_in[1], z_over))
        else:
            if a[2] < z_over - 1e-3:
                pts3.append((a[0], a[1], z_over))
        for i in range(1, len(path_2d) - 1):
            if s_in - 1.0 <= cum_len[i] <= s_out + 1.0:
                pts3.append((path_2d[i][0], path_2d[i][1], z_over))
        if s_out < total_len - 1.0:
            p_out = interp_at(s_out)
            pts3.append((p_out[0], p_out[1], z_over))
        else:
            if b[2] < z_over - 1e-3:
                pts3.append((b[0], b[1], z_over))
        for i in range(1, len(path_2d) - 1):
            if cum_len[i] > s_out + 1.0:
                pts3.append((path_2d[i][0], path_2d[i][1], max(z_mid_floor, b[2])))
        pts3.append(b)

        clean: List[Point3] = [pts3[0]]
        for pt in pts3[1:]:
            if math.hypot(pt[0] - clean[-1][0], pt[1] - clean[-1][1]) > 0.2 or abs(pt[2] - clean[-1][2]) > 0.2:
                clean.append(pt)
        if clean[-1] != b:
            clean[-1] = b

        if not self.polyline_valid_3d(clean):
            return None
        return clean

    def best_path_3d(
        self,
        a: Point3,
        b: Point3,
        ac: Aircraft,
        z_cruise: Optional[float] = None,
        allow_overflight: bool = True,
    ) -> Optional[List[Point3]]:
        """Finds the minimum-energy valid 3D polyline from a to b for aircraft ac."""
        r = self.rules
        z_floor = r.z_min_m
        z_mid = z_cruise if z_cruise is not None else max(z_floor, a[2], b[2])
        z_mid = max(z_floor, min(r.z_max_m, z_mid))

        if self.segment_valid_3d(a, b):
            return [a, b]

        candidates: List[List[Point3]] = []

        z_low = min(a[2], b[2], z_mid)
        blocked_2d = self.blocked_indices_at_z(z_low)
        path_2d = self.shortest_path_2d(a[:2], b[:2], blocked_2d)
        if path_2d is not None:
            pts3: List[Point3] = [a]
            for p in path_2d[1:-1]:
                pts3.append((p[0], p[1], z_mid))
            pts3.append(b)
            if self.polyline_valid_3d(pts3):
                candidates.append(pts3)

        if z_low < z_mid - 1e-3:
            blocked_mid = self.blocked_indices_at_z(z_mid)
            if len(blocked_mid) < len(blocked_2d):
                p2 = self.shortest_path_2d(a[:2], b[:2], blocked_mid)
                if p2 is not None:
                    pts3 = [a] + [(p[0], p[1], z_mid) for p in p2[1:-1]] + [b]
                    if self.polyline_valid_3d(pts3):
                        candidates.append(pts3)

        if allow_overflight:
            ceilings = sorted({
                top + ZONE_TOP_PAD
                for i, (_, _, _, top) in enumerate(self.all_cylinders)
                if top < INF and top + ZONE_TOP_PAD <= r.z_max_m and self.segment_hits_cyl_2d(a[:2], b[:2], i)
            })
            for z_over in ceilings:
                z_over_clamped = max(z_floor, min(r.z_max_m, z_over))
                ov = self._overflight_polyline(a, b, z_over_clamped, z_floor)
                if ov is not None:
                    candidates.append(ov)

        if not candidates:
            return None

        best_pts = None
        best_e = INF
        for cand in candidates:
            e, _ = self._estimate_polyline_energy(cand, ac)
            if e < best_e:
                best_e = e
                best_pts = cand
        return best_pts

    def valid_obs_point(self, p: Point2, contact: Contact, z: float) -> bool:
        r = self.rules
        r_horiz2 = r.R_obs_m * r.R_obs_m - z * z
        if r_horiz2 <= 0.0:
            return False
        dx, dy = p[0] - contact.pos[0], p[1] - contact.pos[1]
        if dx * dx + dy * dy > (math.sqrt(r_horiz2) - 0.1) ** 2:
            return False
        blocked = self.blocked_indices_at_z(z)
        return not self.point_inside_any(p, blocked, pad=OBS_RADIUS_PAD)

    def project_to_obs_disk(self, target: Point2, contact: Contact, z: float) -> Point2:
        """Closest valid observation point to `target` inside contact's obs disk at altitude z."""
        r = self.rules
        r_horiz = math.sqrt(max(0.0, r.R_obs_m * r.R_obs_m - z * z)) - 0.2
        cx, cy = contact.pos[0], contact.pos[1]
        dx, dy = target[0] - cx, target[1] - cy
        d = math.hypot(dx, dy)
        if d <= r_horiz:
            cand = target
        else:
            cand = (cx + dx / d * r_horiz, cy + dy / d * r_horiz)
        blocked = self.blocked_indices_at_z(z)
        if not self.point_inside_any(cand, blocked, pad=OBS_RADIUS_PAD):
            return cand
        best = (cx, cy)
        best_d = INF
        base_ang = math.atan2(dy, dx) if d > 1e-6 else 0.0
        for rad in (r_horiz, 0.5 * r_horiz, 0.0):
            n_ang = 36 if rad > 0 else 1
            for k in range(n_ang):
                ang = base_ang + 2.0 * math.pi * k / n_ang
                p = (cx + rad * math.cos(ang), cy + rad * math.sin(ang))
                if not self.point_inside_any(p, blocked, pad=OBS_RADIUS_PAD):
                    dd = math.hypot(p[0] - target[0], p[1] - target[1])
                    if dd < best_d:
                        best_d = dd
                        best = p
            if best_d < INF:
                break
        return best

    def optimize_disk_tour(
        self,
        home: Point2,
        contacts: Sequence[Contact],
        z: float,
        n_iters: int = 8,
    ) -> List[Point2]:
        """Exact Disk-TSP coordinate descent for observation points along a scout tour."""
        k = len(contacts)
        if k == 0:
            return []
        r = self.rules
        r_horiz = math.sqrt(max(0.0, r.R_obs_m * r.R_obs_m - z * z)) - 0.25
        blocked = self.blocked_indices_at_z(z)

        pts: List[Point2] = [(c.pos[0], c.pos[1]) for c in contacts]
        for i in range(k):
            prev_p = home if i == 0 else pts[i - 1]
            next_p = home if i == k - 1 else (contacts[i + 1].pos[0], contacts[i + 1].pos[1])
            mid = (0.5 * (prev_p[0] + next_p[0]), 0.5 * (prev_p[1] + next_p[1]))
            pts[i] = self.project_to_obs_disk(mid, contacts[i], z)

        if k == 1:
            pts[0] = self.project_to_obs_disk(home, contacts[0], z)
            return pts

        for _ in range(n_iters):
            max_move = 0.0
            for i in range(k):
                a = home if i == 0 else pts[i - 1]
                b = home if i == k - 1 else pts[i + 1]
                cx, cy = contacts[i].pos[0], contacts[i].pos[1]
                new_p = _best_point_in_disk(a, b, cx, cy, r_horiz)
                if blocked and self.point_inside_any(new_p, blocked, pad=OBS_RADIUS_PAD):
                    new_p = self.project_to_obs_disk(new_p, contacts[i], z)
                move = math.hypot(new_p[0] - pts[i][0], new_p[1] - pts[i][1])
                if move > max_move:
                    max_move = move
                pts[i] = new_p
            if max_move < 0.08:
                break
        return pts

    def engage_point(self, ref: Point2, contact: Contact, z: float = 0.0, angle_deg: float = 0.0) -> Point3:
        """Returns optimal engagement point on contact's R_eng_m sphere facing `ref` (rotated by angle_deg)."""
        r_max = self.rules.R_eng_m
        r_h = math.sqrt(max(0.0, r_max * r_max - z * z)) - 0.1
        if r_h <= 0.0:
            return (contact.pos[0], contact.pos[1], min(z, r_max - 0.05))
        vx, vy = ref[0] - contact.pos[0], ref[1] - contact.pos[1]
        norm = math.hypot(vx, vy)
        if norm < 1e-6:
            ux, uy = 1.0, 0.0
        else:
            ux, uy = vx / norm, vy / norm
        if abs(angle_deg) > 1e-6:
            rad = math.radians(angle_deg)
            ca, sa = math.cos(rad), math.sin(rad)
            ux, uy = ux * ca - uy * sa, ux * sa + uy * ca
        p = (contact.pos[0] + ux * r_h, contact.pos[1] + uy * r_h)
        blocked = self.blocked_indices_at_z(z)
        if not self.point_inside_any(p, blocked, pad=OBS_RADIUS_PAD):
            return (p[0], p[1], z)
        return (contact.pos[0], contact.pos[1], z)


def _best_point_in_disk(a: Point2, b: Point2, cx: float, cy: float, r: float) -> Point2:
    """Minimizes ||p - a|| + ||p - b|| subject to ||p - (cx, cy)|| <= r."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    if l2 < 1e-12:
        vx, vy = a[0] - cx, a[1] - cy
        d = math.hypot(vx, vy)
        if d <= r:
            return a
        return (cx + vx / d * r, cy + vy / d * r)
    s = max(0.0, min(1.0, ((cx - a[0]) * dx + (cy - a[1]) * dy) / l2))
    proj = (a[0] + s * dx, a[1] + s * dy)
    if (proj[0] - cx) ** 2 + (proj[1] - cy) ** 2 <= r * r:
        return proj

    ang_a = math.atan2(a[1] - cy, a[0] - cx)
    ang_b = math.atan2(b[1] - cy, b[0] - cx)
    diff = (ang_b - ang_a + math.pi) % (2.0 * math.pi) - math.pi
    lo = ang_a
    hi = ang_a + diff
    if lo > hi:
        lo, hi = hi, lo

    inv_phi = 0.6180339887498949
    inv_phi2 = 0.3819660112501051

    def f(theta: float) -> float:
        px = cx + r * math.cos(theta)
        py = cy + r * math.sin(theta)
        return math.hypot(px - a[0], py - a[1]) + math.hypot(px - b[0], py - b[1])

    x1 = lo + inv_phi2 * (hi - lo)
    x2 = lo + inv_phi * (hi - lo)
    f1, f2 = f(x1), f(x2)
    for _ in range(12):
        if f1 < f2:
            hi = x2
            x2 = x1
            f2 = f1
            x1 = lo + inv_phi2 * (hi - lo)
            f1 = f(x1)
        else:
            lo = x1
            x1 = x2
            f1 = f2
            x2 = lo + inv_phi * (hi - lo)
            f2 = f(x2)
    best_ang = 0.5 * (lo + hi)
    return (cx + r * math.cos(best_ang), cy + r * math.sin(best_ang))
