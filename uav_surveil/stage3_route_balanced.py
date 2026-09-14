"""Stage 3A: Balanced route generator.

A simple improvement over KMNN that ensures:
1. More balanced route lengths
2. Better spatial distribution
3. No duplicate starting points
"""

from __future__ import annotations

from collections.abc import Sequence
from math import hypot

import numpy as np

from .core.cell import Cell
from .core.route import Route

__all__ = ["generate_routes_balanced"]


def generate_routes_balanced(
    cells: Sequence[Cell],
    n_launch: int,
    cruise_speed: float,
    depot: tuple[float, float] = (0.0, 0.0),
) -> tuple[list[Route], dict]:
    """
    Generate balanced routes using improved distribution.

    Fixes KMNN issues:
    - Ensures routes have similar lengths (balanced workload)
    - Better spatial distribution to avoid gaps
    - Unique starting points to prevent route overlap
    """

    print(f"🔄 Balanced: Distributing {len(cells)} cells → {n_launch} routes")

    if not cells:
        return [], {"algorithm": "balanced", "longest_loop_time": 0.0}

    depot_coord = np.array([depot[0], depot[1]])

    # Strategy: Distribute cells evenly by distance from depot
    # This ensures more balanced route lengths

    # Calculate distances from depot
    distances = [hypot(cell.x - depot[0], cell.y - depot[1]) for cell in cells]

    # Sort cells by distance (nearest to farthest)
    sorted_indices = np.argsort(distances)

    # Create routes by round-robin assignment
    # This ensures each route gets mix of near/far cells
    routes = [[] for _ in range(n_launch)]

    for i, cell_idx in enumerate(sorted_indices):
        route_idx = i % n_launch
        routes[route_idx].append(cells[cell_idx])

    # Convert to Route objects with proper ordering
    route_objects = []
    for i, cell_list in enumerate(routes):
        if not cell_list:
            # Empty route - shouldn't happen but handle gracefully
            route = Route(
                id=f"balanced_route_{i:02d}",
                cell_sequence=[],  # Empty list of cell IDs
                loop_time=0.0,
            )
        else:
            # Order cells within route for efficient traversal
            ordered_cells = _order_cells_efficiently(cell_list, depot_coord)

            # Calculate loop time
            loop_time = _calculate_loop_time(ordered_cells, cruise_speed, depot_coord)

            route = Route(
                id=f"balanced_route_{i:02d}",
                cell_sequence=[
                    cell.id for cell in ordered_cells
                ],  # Store cell IDs, not Cell objects
                loop_time=loop_time,
            )

        route_objects.append(route)

    # --- Post-process: ensure each route has at least MIN_CELLS_PER_ROUTE cells ---
    MIN_CELLS_PER_ROUTE = 5
    small_routes = [
        r for r in route_objects if len(r.cell_sequence) < MIN_CELLS_PER_ROUTE
    ]
    if small_routes:
        orphan_cells = []
        for r in small_routes:
            orphan_cells.extend(r.cell_sequence)
            route_objects.remove(r)
        # Distribute orphans to existing routes (shortest first)
        route_objects.sort(key=lambda rt: len(rt.cell_sequence))
        for idx, cid in enumerate(orphan_cells):
            route_objects[idx % len(route_objects)].cell_sequence.append(cid)
        # Re-add empty placeholders to keep count consistent
        while len(route_objects) < n_launch:
            route_objects.append(
                Route(
                    id=f"balanced_empty_{len(route_objects):02d}",
                    cell_sequence=[],
                    loop_time=0.0,
                )
            )

    # Summary statistics
    longest_loop = max(route.loop_time or 0 for route in route_objects)
    avg_cells_per_route = len(cells) / n_launch

    summary = {
        "algorithm": "balanced",
        "longest_loop_time": longest_loop,
        "avg_cells_per_route": avg_cells_per_route,
        "route_balance": _calculate_balance_metric(route_objects),
    }

    print(
        f"✅ Balanced routes: max loop {longest_loop:.1f}s, "
        f"avg {avg_cells_per_route:.1f} cells/route"
    )

    return route_objects, summary


def _order_cells_efficiently(cells: list[Cell], depot: np.ndarray) -> list[Cell]:
    """Order cells within a route for efficient traversal (nearest neighbor)."""
    if len(cells) <= 1:
        return cells

    ordered = []
    remaining = cells.copy()

    # Start with cell closest to depot
    current_pos = depot

    while remaining:
        # Find nearest remaining cell
        distances = [
            hypot(cell.x - current_pos[0], cell.y - current_pos[1])
            for cell in remaining
        ]
        nearest_idx = np.argmin(distances)
        nearest_cell = remaining.pop(nearest_idx)

        ordered.append(nearest_cell)
        current_pos = np.array([nearest_cell.x, nearest_cell.y])

    return ordered


def _calculate_loop_time(
    cells: list[Cell], cruise_speed: float, depot: np.ndarray
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


def _calculate_balance_metric(routes: list[Route]) -> float:
    """Calculate how balanced the routes are (lower = more balanced)."""
    lengths = [len(route.cell_sequence) for route in routes]
    if not lengths:
        return 0.0

    avg_length = sum(lengths) / len(lengths)
    variance = sum((length - avg_length) ** 2 for length in lengths) / len(lengths)

    return variance  # Lower variance = better balance
