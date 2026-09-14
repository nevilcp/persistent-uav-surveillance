"""Stage 3A: Round-Robin Route Generator

Implements architecture.md §4.1 / thesis Alg. 5: a serpentine (boustrophedon)
sweep of the grid, a seeded round-robin distribution of cells to routes, and
a bounded local-smoothing pass to balance loop times across routes.

Key advantages over KMNN:
- Guaranteed balanced routes (each route gets ≈same number of cells)
- No empty or micro-routes (< 5 cells)
- More predictable performance characteristics
- Better coverage consistency across multiple laps
"""

from typing import Dict, List, Optional, Sequence, Tuple
import numpy as np
from math import hypot

from .core.cell import Cell
from .core.route import Route

__all__ = ["generate_routes_roundrobin"]


def generate_routes_roundrobin(
    cells: Sequence[Cell],
    n_launch: int,
    cruise_speed: float,
    depot: Tuple[float, float] = (0.0, 0.0),
    furthest_first: bool = False,
    buse_s: Optional[float] = None,
) -> Tuple[List[Route], object]:
    """Generate routes via a serpentine sweep + seeded round-robin + smoothing.

    1. Sweep the grid in a boustrophedon order, starting from the row
       nearest the depot (`_serpentine_order`).
    2. Seed the round-robin phase; with furthest_first, the route seeded
       from the sweep position farthest from the depot is labelled route 0,
       so it is scheduled first (`_pick_seeds`).
    3. Distribute cells to routes round-robin along the seeded phase.
    4. Smooth loop-time variance with a bounded number of boundary
       moves/swaps between adjacent routes, respecting `buse_s`.
    5. Order each route's cells with a nearest-neighbour tie-break, kept
       only when it shortens the loop (the serpentine order already yields
       short hops).

    Args:
        cells: List of surveillance cells to cover
        n_launch: Number of routes (UAVs) to generate
        cruise_speed: UAV speed in m/s for time calculations
        depot: (x, y) depot coordinates
        furthest_first: If True, seed route 0 from the depot-farthest cell
        buse_s: Usable battery endurance in seconds. Smoothing moves that
            would push a route's loop time above this are rejected. None
            disables the guard.

    Returns:
        Tuple of (route_list, summary_dict)
    """

    print(
        f"🔄 Round-Robin: Distributing {len(cells)} cells → {n_launch} routes (serpentine)"
    )

    if not cells:
        return [], {"algorithm": "roundrobin", "longest_loop_time": 0.0}

    if n_launch <= 0:
        raise ValueError("n_launch must be positive")

    sweep = _serpentine_order(cells, depot)
    seed_offset = _pick_seeds(sweep, n_launch, depot, furthest_first)

    routes: List[List[Cell]] = [[] for _ in range(n_launch)]
    for i, cell in enumerate(sweep):
        routes[(i + seed_offset) % n_launch].append(cell)

    routes, smoothing_moves = _smooth_routes(routes, depot, cruise_speed, buse_s=buse_s)

    # Convert to Route objects, tie-breaking with nearest-neighbour ordering
    # only when it actually shortens the loop.
    route_objects = []
    for i, cell_list in enumerate(routes):
        if not cell_list:
            route = Route(id=f"rr_{i:02d}", cell_sequence=[], loop_time=0.0)
        else:
            ordered_cells = _best_order(cell_list, depot, cruise_speed)
            route_time = _calculate_route_time(ordered_cells, cruise_speed, depot)
            route = Route(
                id=f"rr_{i:02d}",
                cell_sequence=[cell.id for cell in ordered_cells],
                loop_time=route_time,
            )
        route_objects.append(route)

    # Verify balance and report
    route_lengths = [len(r.cell_sequence) for r in route_objects]
    min_length = min(route_lengths) if route_lengths else 0
    max_length = max(route_lengths) if route_lengths else 0
    avg_length = sum(route_lengths) / len(route_lengths) if route_lengths else 0

    print(
        f"   ✅ Route balance: {min_length}-{max_length} cells/route (avg: {avg_length:.1f})"
    )

    loop_times = [route.loop_time or 0.0 for route in route_objects]
    longest_loop = max(loop_times) if loop_times else 0.0
    loop_time_std = float(np.std(loop_times)) if loop_times else 0.0

    summary = {
        "algorithm": "roundrobin",
        "longest_loop_time": longest_loop,
        "total_routes": len(route_objects),
        "min_route_length": min_length,
        "max_route_length": max_length,
        "avg_route_length": avg_length,
        "balance_variance": np.var(route_lengths) if route_lengths else 0,
        "loop_time_std": loop_time_std,
        "smoothing_moves": smoothing_moves,
    }

    return route_objects, summary


def _serpentine_order(cells: Sequence[Cell], depot: Tuple[float, float]) -> List[Cell]:
    """Boustrophedon sweep: group cells by row (y), alternate column
    traversal direction per row, starting from the row nearest the depot."""
    rows: Dict[float, List[Cell]] = {}
    for cell in cells:
        key = round(cell.y, 6)
        rows.setdefault(key, []).append(cell)

    row_keys = sorted(rows.keys(), key=lambda y: abs(y - depot[1]))

    ordered: List[Cell] = []
    for i, y in enumerate(row_keys):
        row_cells = sorted(rows[y], key=lambda c: c.x)
        if i % 2 == 1:
            row_cells = list(reversed(row_cells))
        ordered.extend(row_cells)
    return ordered


def _pick_seeds(
    sweep: List[Cell], n: int, depot: Tuple[float, float], furthest_first: bool
) -> int:
    """Resolve n evenly spaced seed positions along the sweep to a single
    round-robin phase offset. With furthest_first, the offset is chosen so
    the route seeded from the depot-farthest sweep position becomes route 0
    (and is therefore scheduled first)."""
    if n <= 0 or not sweep or not furthest_first:
        return 0
    total = len(sweep)
    seed_positions = [round(k * total / n) for k in range(n)]
    farthest_seed = max(seed_positions)
    return (-farthest_seed) % n


def _smooth_routes(
    routes: List[List[Cell]],
    depot: Tuple[float, float],
    cruise_speed: float,
    buse_s: Optional[float] = None,
    max_passes: int = 3,
) -> Tuple[List[List[Cell]], int]:
    """Local boundary smoothing between adjacent routes: move or swap 1-2
    boundary cells if it reduces loop-time variance and keeps both routes
    within buse_s."""
    n = len(routes)
    if n < 2:
        return routes, 0

    def loop_time(cell_list: List[Cell]) -> float:
        # Use the same best-of-(sweep, nearest-neighbour) time that the
        # final route will report, so the smoothing objective matches
        # loop_time_std exactly instead of a pre-reorder proxy for it.
        return _route_time(cell_list, depot, cruise_speed)

    def total_deviation(times: List[float]) -> float:
        # Sum of squared deviations from the mean: strictly decreasing this
        # is equivalent to strictly decreasing the population variance (and
        # thus loop_time_std), unlike a sum-of-absolute-deviations proxy
        # which can improve locally while the overall std worsens.
        mean = sum(times) / len(times)
        return sum((t - mean) ** 2 for t in times)

    moves = 0
    times = [loop_time(r) for r in routes]
    for _ in range(max_passes):
        improved = False
        for u in range(n - 1):
            v = u + 1
            base_dev = total_deviation(times)
            best = None
            for new_u, new_v in _boundary_candidates(routes[u], routes[v]):
                if not new_u or not new_v:
                    continue
                lu, lv = loop_time(new_u), loop_time(new_v)
                if buse_s is not None and (lu > buse_s or lv > buse_s):
                    continue
                trial = list(times)
                trial[u], trial[v] = lu, lv
                dev = total_deviation(trial)
                if dev < base_dev - 1e-9 and (best is None or dev < best[0]):
                    best = (dev, new_u, new_v, lu, lv)
            if best is not None:
                _, new_u, new_v, lu, lv = best
                routes[u], routes[v] = new_u, new_v
                times[u], times[v] = lu, lv
                moves += 1
                improved = True
        if not improved:
            break
    return routes, moves


def _boundary_candidates(
    route_u: List[Cell], route_v: List[Cell]
) -> List[Tuple[List[Cell], List[Cell]]]:
    """Candidate (new_u, new_v) pairs from moving/swapping 1-2 boundary
    cells between two adjacent routes."""
    candidates: List[Tuple[List[Cell], List[Cell]]] = []
    for k in (1, 2):
        if len(route_u) > k:
            candidates.append((route_u[:-k], route_u[-k:] + route_v))
        if len(route_v) > k:
            candidates.append((route_u + route_v[:k], route_v[k:]))
    if route_u and route_v:
        candidates.append((route_u[:-1] + [route_v[0]], [route_u[-1]] + route_v[1:]))
    return candidates


def _best_order(
    cells: List[Cell], depot: Tuple[float, float], cruise_speed: float
) -> List[Cell]:
    """Nearest-neighbour tie-break: only replace sweep order if it is
    strictly shorter (the serpentine sweep already yields short hops)."""
    if not cells:
        return cells
    nn_ordered = _order_nearest_neighbour(cells, depot)
    if _calculate_route_time(nn_ordered, cruise_speed, depot) < _calculate_route_time(
        cells, cruise_speed, depot
    ):
        return nn_ordered
    return cells


def _route_time(
    cells: List[Cell], depot: Tuple[float, float], cruise_speed: float
) -> float:
    """Best achievable loop time for a cell set: whichever of the sweep
    order or its nearest-neighbour tie-break is shorter."""
    return _calculate_route_time(
        _best_order(cells, depot, cruise_speed), cruise_speed, depot
    )


def _calculate_route_time(
    cells: List[Cell], cruise_speed: float, depot: Tuple[float, float]
) -> float:
    """Calculate time to complete the route loop."""
    if not cells:
        return 0.0

    total_distance = 0.0

    # Distance from depot to first cell
    if cells:
        total_distance += hypot(cells[0].x - depot[0], cells[0].y - depot[1])

    # Distance between consecutive cells
    for i in range(1, len(cells)):
        prev_cell = cells[i - 1]
        curr_cell = cells[i]
        total_distance += hypot(curr_cell.x - prev_cell.x, curr_cell.y - prev_cell.y)

    # Distance from last cell back to depot
    if cells:
        last_cell = cells[-1]
        total_distance += hypot(depot[0] - last_cell.x, depot[1] - last_cell.y)

    return total_distance / cruise_speed if cruise_speed > 0 else 0.0


def _order_nearest_neighbour(
    cells: List[Cell], depot: Tuple[float, float]
) -> List[Cell]:
    """Order cells within a route for efficient traversal using nearest neighbor heuristic."""
    if len(cells) <= 2:
        return cells

    ordered = []
    remaining = list(cells)  # Use list to avoid hashing issues

    # Start from cell closest to depot
    distances_to_depot = [
        hypot(cell.x - depot[0], cell.y - depot[1]) for cell in remaining
    ]
    closest_idx = min(
        range(len(distances_to_depot)), key=distances_to_depot.__getitem__
    )
    current = remaining.pop(closest_idx)
    ordered.append(current)

    # Greedy nearest neighbor
    while remaining:
        distances = [
            hypot(current.x - cell.x, current.y - cell.y) for cell in remaining
        ]
        nearest_idx = min(range(len(distances)), key=distances.__getitem__)
        current = remaining.pop(nearest_idx)
        ordered.append(current)

    return ordered
