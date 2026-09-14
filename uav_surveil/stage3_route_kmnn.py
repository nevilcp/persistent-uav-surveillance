"""Stage 3A: K-means + Nearest-Neighbour route generator (baseline v1).

Motivation – provide shorter, more balanced loops than the serpentine sweep
without depending on heavy external solvers.  Uses a lightweight K-means
implementation (numpy only) to cluster grid cells into *n_launch* buckets and
then orders each bucket with a greedy nearest-neighbour heuristic.

The public API is identical to the other route generators so it slots straight
into *stage3_route_factory*.
"""

from __future__ import annotations

from collections.abc import Sequence
from itertools import pairwise
from math import hypot

import numpy as np

from .core.cell import Cell
from .core.route import Route

__all__ = [
    "generate_routes_kmnn",
]


# -----------------------------------------------------------------------------
# Helper: lightweight K-means (Lloyd) – 10 iterations max
# -----------------------------------------------------------------------------


def _kmeans(
    points: np.ndarray,
    k: int,
    max_iter: int = 10,
    rng: np.random.Generator | None = None,
):
    if rng is None:
        rng = np.random.default_rng(42)
    n = points.shape[0]
    if k >= n:
        # One cell per cluster – trivial assignment
        return np.arange(n)

    # Initial centroids – choose k random unique indices
    cent_idx = rng.choice(n, size=k, replace=False)
    centroids = points[cent_idx]

    assignments = np.zeros(n, dtype=int)
    for _ in range(max_iter):
        # Assignment step
        dists = np.linalg.norm(points[:, None, :] - centroids[None, :, :], axis=2)
        new_assign = dists.argmin(axis=1)
        if np.array_equal(new_assign, assignments):
            break  # converged
        assignments = new_assign
        # Update step
        for i in range(k):
            cluster_pts = points[assignments == i]
            if len(cluster_pts) > 0:
                centroids[i] = cluster_pts.mean(axis=0)
            else:
                # Empty cluster – re-seed to a random point
                centroids[i] = points[rng.integers(0, n)]
    return assignments


# -----------------------------------------------------------------------------
# Helper: greedy nearest-neighbour ordering
# -----------------------------------------------------------------------------


def _order_nearest_neighbour(
    cells: list[Cell], depot: tuple[float, float]
) -> list[Cell]:
    if len(cells) <= 2:
        return cells
    # Start from cell with smallest (x+y) to get deterministic order
    start = min(cells, key=lambda c: c.x + c.y)
    visited = [start]
    remaining = list(cells)
    remaining.remove(start)
    current = start
    while remaining:
        nxt = min(
            remaining, key=lambda c: (c.x - current.x) ** 2 + (c.y - current.y) ** 2
        )
        visited.append(nxt)
        remaining.remove(nxt)
        current = nxt
    return visited


# -----------------------------------------------------------------------------
# Helper: rotate route so it starts at cell nearest to depot
# -----------------------------------------------------------------------------


def _rotate_to_nearest_depot(
    cells: list[Cell], depot: tuple[float, float]
) -> list[Cell]:
    if not cells:
        return cells
    nearest_idx = min(
        range(len(cells)),
        key=lambda i: (cells[i].x - depot[0]) ** 2 + (cells[i].y - depot[1]) ** 2,
    )
    # Rotate list so nearest cell becomes first
    return cells[nearest_idx:] + cells[:nearest_idx]


# -----------------------------------------------------------------------------
# Public API
# -----------------------------------------------------------------------------


def generate_routes_kmnn(
    cells: Sequence[Cell],
    n_launch: int,
    cruise_speed: float,
    depot: tuple[float, float] = (0.0, 0.0),
) -> tuple[list[Route], object]:
    """Generate *n_launch* routes using K-means clustering + NN pathing.

    Returns:
        (route_list, summary) where summary has attributes similar to *RouteSetSummary*.
    """
    if n_launch <= 0:
        raise ValueError("n_launch must be positive")
    n_cells = len(cells)
    if n_cells == 0:
        raise ValueError("No cells provided")

    pts = np.array([(c.x, c.y) for c in cells])
    assignments = _kmeans(pts, n_launch)

    # Build routes per cluster
    clusters: list[list[Cell]] = [[] for _ in range(n_launch)]
    for cell, idx in zip(cells, assignments):
        clusters[idx].append(cell)

    routes: list[Route] = []
    longest_loop = 0.0
    for idx, cluster in enumerate(clusters):
        if not cluster:
            continue  # empty cluster (rare) – skip
        ordered = _order_nearest_neighbour(cluster, depot)
        ordered = _rotate_to_nearest_depot(ordered, depot)
        cell_ids = [c.id for c in ordered]

        # Compute path distance
        dist = 0.0
        for a, b in pairwise(ordered):
            dist += hypot(a.x - b.x, a.y - b.y)
        # Ferry legs
        dist += 2 * hypot(ordered[0].x - depot[0], ordered[0].y - depot[1])

        loop_time = dist / cruise_speed if cruise_speed > 0 else 0.0
        longest_loop = max(longest_loop, loop_time)

        routes.append(
            Route(
                id=f"kmnn_{idx:02d}",
                cell_sequence=cell_ids,
                estimated_duration=loop_time,  # crude, equals loop for one-way patrol
                loop_time=loop_time,
            )
        )

    # Fill up to n_launch with empty routes if some clusters were empty
    placeholder_cell_id = cells[0].id if cells else "placeholder"
    while len(routes) < n_launch:
        routes.append(
            Route(
                id=f"kmnn_empty_{len(routes):02d}", cell_sequence=[placeholder_cell_id]
            )
        )

    summary = type("RouteSetSummary", (), {})()
    summary.n_routes = len(routes)
    summary.cells_per_route = [len(r.cell_sequence) for r in routes]
    summary.longest_loop_time = longest_loop

    return routes, summary
