"""Exact combinatorial primitives: Hungarian min-cost assignment and 2D Knapsack DP."""
from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

INF = 1e18


def hungarian_min_cost_duals(
    cost_matrix: Sequence[Sequence[float]],
) -> Tuple[float, List[int], List[float], List[float]]:
    """Exact O(n_rows * n_cols^2) Hungarian algorithm returning primal assignment and dual potentials.

    Assumes n_rows <= n_cols (every row is matched to a distinct column).
    Entries >= INF/2 are treated as forbidden.

    Dual invariants:
        row_u[i] + col_v[j] <= cost_matrix[i][j] for all i, j
        row_u[i] + col_v[col_for_row[i]] == cost_matrix[i][col_for_row[i]]
        col_v[j] == 0.0 for all unassigned columns j, and col_v[j] <= 0.0 for assigned columns j.

    Returns:
        (total_cost, col_for_row, row_u, col_v).
        If infeasible, returns (INF, [], [], []).
    """
    n = len(cost_matrix)
    if n == 0:
        return 0.0, [], [], []
    m = len(cost_matrix[0])
    if n > m:
        return INF, [], [], []

    # 1-indexed potentials u[0..n], v[0..m], matching p[0..m] (row matched to col j)
    u = [0.0] * (n + 1)
    v = [0.0] * (m + 1)
    p = [0] * (m + 1)
    way = [0] * (m + 1)

    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [INF] * (m + 1)
        used = [False] * (m + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = INF
            j1 = 0
            row = cost_matrix[i0 - 1]
            ui0 = u[i0]
            for j in range(1, m + 1):
                if not used[j]:
                    cur = row[j - 1] - ui0 - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j
            if delta >= INF * 0.25:
                return INF, [], [], []
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break

    col_for_row = [-1] * n
    total = 0.0
    for j in range(1, m + 1):
        if p[j] != 0:
            r_idx = p[j] - 1
            c = cost_matrix[r_idx][j - 1]
            if c >= INF * 0.25:
                return INF, [], [], []
            col_for_row[r_idx] = j - 1
            total += c
    return total, col_for_row, u[1:], v[1:]


def hungarian_min_cost(cost_matrix: Sequence[Sequence[float]]) -> Tuple[float, List[int]]:
    """Exact O(n_rows * n_cols^2) Hungarian algorithm for rectangular min-cost assignment."""
    total, col_for_row, _, _ = hungarian_min_cost_duals(cost_matrix)
    return total, col_for_row


def knapsack_2d(
    items: Sequence[Tuple[int, int, float]],
    cap_l: int,
    cap_h: int,
) -> Tuple[float, List[int]]:
    """Exact 2D 0/1 knapsack maximizing sum(value) s.t. sum(nl) <= cap_l, sum(nh) <= cap_h.

    Args:
        items: Sequence of (need_light, need_heavy, net_value).
        cap_l: Available light effectors.
        cap_h: Available heavy effectors.

    Returns:
        (best_value, list_of_selected_indices).
    """
    n = len(items)
    if n == 0:
        return 0.0, []

    w_h = cap_h + 1
    size = (cap_l + 1) * w_h
    dp = [-INF] * size
    dp[0] = 0.0
    # Store bitmask of chosen items (Python ints have arbitrary precision!)
    mask = [0] * size

    for idx, (nl, nh, val) in enumerate(items):
        if val <= 0.0 or nl > cap_l or nh > cap_h:
            continue
        bit = 1 << idx
        for i in range(cap_l, nl - 1, -1):
            row_off = i * w_h
            prev_row_off = (i - nl) * w_h
            for j in range(cap_h, nh - 1, -1):
                pv = dp[prev_row_off + j - nh]
                if pv > -INF * 0.5:
                    nv = pv + val
                    pos = row_off + j
                    if nv > dp[pos]:
                        dp[pos] = nv
                        mask[pos] = mask[prev_row_off + j - nh] | bit

    best_val = 0.0
    best_mask = 0
    for pos in range(size):
        if dp[pos] > best_val:
            best_val = dp[pos]
            best_mask = mask[pos]

    chosen = [i for i in range(n) if (best_mask >> i) & 1]
    return best_val, chosen
