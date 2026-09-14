"""Stage 3A: Route Generation (baseline implementation).

Provides a very fast deterministic algorithm – *Serpentine Sweep* – that
creates contiguous lawn-mower routes and splits them evenly across the active
UAVs.  Good enough for first-round simulations; can be swapped out later via the
factory method.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

from .config.parameters import SystemParameters
from .core.cell import Cell
from .core.route import Route

__all__ = [
    "RouteSetSummary",
    "generate_routes_greedy",
]


@dataclass
class RouteSetSummary:
    """Lightweight stats object returned alongside the route list."""

    n_routes: int
    cells_per_route: list[int]
    longest_loop_time: float


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------


def _serpentine_order(cells: Sequence[Cell]) -> list[Cell]:
    """Return cells sorted in serpentine lawn-mower order.

    Works by grouping cells by *y* (row) then alternating X ordering on every
    other row.  Robust as long as the grid is axis-aligned.
    """

    # Group cells by their *row* – identify by exact y coordinate
    rows = {}
    for c in cells:
        rows.setdefault(c.y, []).append(c)

    ordered_rows = sorted(rows.items(), key=lambda kv: kv[0])  # ascending y
    result: list[Cell] = []

    for i, (_, row_cells) in enumerate(ordered_rows):
        row_sorted = sorted(row_cells, key=lambda c: c.x, reverse=bool(i % 2))
        result.extend(row_sorted)

    return result


def _chunk(lst: list[Cell], k: int) -> list[list[Cell]]:
    """Split *lst* into *k* nearly-equal consecutive chunks."""

    n = len(lst)
    base = n // k
    remainder = n % k
    chunks = []
    start = 0
    for i in range(k):
        end = start + base + (1 if i < remainder else 0)
        chunks.append(lst[start:end])
        start = end
    return chunks


# -----------------------------------------------------------------------------
# Public API
# -----------------------------------------------------------------------------


def generate_routes_greedy(
    cells: Sequence[Cell],
    n_launch: int,
    cruise_speed: float,
    depot: tuple[float, float] = (0.0, 0.0),
) -> tuple[list[Route], RouteSetSummary]:
    """Generate routes using serpentine sweep + even split.

    Args:
        cells: List of *Cell* objects covering the grid.
        n_launch: Number of active UAVs.
        cruise_speed: UAV speed (m/s) used for simple duration estimate.
        depot: Coordinates of launch depot.

    Returns:
        (routes, summary)
    """

    if n_launch <= 0:
        raise ValueError("n_launch must be positive")

    # 1. Order cells
    ordered = _serpentine_order(list(cells))

    # 2. Chunk into n_launch sub-lists
    cell_chunks = _chunk(ordered, n_launch)

    routes: list[Route] = []
    longest_loop = 0.0
    cell_size_est = abs(ordered[0].x - ordered[1].x) if len(ordered) > 1 else 0.0

    for idx, chunk in enumerate(cell_chunks):
        route_id = f"route_{idx:02d}"
        cell_ids = [c.id for c in chunk]

        # Naive path length estimate: Manhattan distance along chunk
        distance = max(0, (len(chunk) - 1)) * cell_size_est
        duration = distance / cruise_speed if cruise_speed > 0 else 0.0

        # Loop time includes ferry to first cell & back to depot (approx)
        loop_time = (
            duration
            + 2
            * (((chunk[0].x - depot[0]) ** 2 + (chunk[0].y - depot[1]) ** 2) ** 0.5)
            / cruise_speed
        )

        longest_loop = max(longest_loop, loop_time)

        routes.append(
            Route(
                id=route_id,
                cell_sequence=cell_ids,
                estimated_duration=duration,
                loop_time=loop_time,
            )
        )

    summary = RouteSetSummary(
        n_routes=n_launch,
        cells_per_route=[len(c) for c in cell_chunks],
        longest_loop_time=longest_loop,
    )

    return routes, summary


# Convenience wrapper from config


def generate_routes_from_config(
    config: SystemParameters, cells: Sequence[Cell], n_launch: int
):
    """Generate greedy routes directly from a *SystemParameters* object."""

    return generate_routes_greedy(
        cells,
        n_launch,
        cruise_speed=config.uav.cruise_speed,
        depot=(config.mission.depot_x, config.mission.depot_y),
    )
