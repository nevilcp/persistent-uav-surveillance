"""Stage 2: route generation."""

from __future__ import annotations

import math
from bisect import bisect_right
from collections.abc import Sequence
from dataclasses import dataclass, replace

from .airspace import Airspace
from .geometry import path_length
from .grid import Cell, Grid, boustrophedon_order
from .params import Point, UAVParams


@dataclass(frozen=True)
class Route:
    """Cell list and departure time of one UAV slot; a replacement UAV inherits it."""

    id: int  # 1-based
    cells: tuple[Cell, ...]  # ordered cell list I_u
    departure: float = 0.0  # scheduled departure d_u (s)
    depot: int = 0  # index of the home base

    @property
    def waypoints(self) -> list[Point]:
        return [c.position for c in self.cells]


def round_robin_routes(
    grid: Grid, n_fly: int, cells: Sequence[Cell] | None = None
) -> list[Route]:
    """Round-robin assignment along the boustrophedon order of ``cells``."""
    ordered = boustrophedon_order(list(grid.cells if cells is None else cells))
    return [Route(id=u + 1, cells=tuple(ordered[u::n_fly])) for u in range(n_fly)]


@dataclass(frozen=True)
class Loop:
    """Closed patrol loop through every free cell, with arc-length bookkeeping."""

    vertices: tuple[Point, ...]  # closed polyline corners, detour points included
    arcs: tuple[float, ...]  # arc length at each vertex, starting at 0
    length: float  # L, including the closing segment
    cells: tuple[Cell, ...]  # free cells in loop order
    cell_arcs: tuple[float, ...]  # arc length at each of those cells

    def point_at(self, s: float) -> Point:
        """Position at arc length ``s`` (wraps around the loop)."""
        s %= self.length
        i = bisect_right(self.arcs, s) - 1
        end = self.arcs[i + 1] if i + 1 < len(self.arcs) else self.length
        x0, y0 = self.vertices[i]
        x1, y1 = self.vertices[(i + 1) % len(self.vertices)]
        t = (s - self.arcs[i]) / (end - self.arcs[i])
        return (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)

    def cells_near(self, s: float, radius: float) -> list[Cell]:
        """Free cells whose loop position is within ``radius`` of arc length ``s``."""
        s %= self.length
        return [
            cell
            for cell, arc in zip(self.cells, self.cell_arcs)
            if min(abs(arc - s), self.length - abs(arc - s)) <= radius
        ]


def shared_loop(grid: Grid, airspace: Airspace) -> Loop:
    """Closed boustrophedon over the free cells; zone cells are flown round."""
    order = [
        grid.cells[row * grid.n_cols + col]
        for row, col in _cycle(grid.n_rows, grid.n_cols)
        if airspace.free_index.get(row * grid.n_cols + col) is not None
    ]
    vertices: list[Point] = []
    cell_vertex: list[int] = []
    for cell, following in zip(order, order[1:] + order[:1]):
        cell_vertex.append(len(vertices))
        vertices.append(cell.position)
        vertices.extend(airspace.path(cell.position, following.position)[1:-1])
    arcs = [0.0]
    for a, b in zip(vertices, vertices[1:]):
        arcs.append(arcs[-1] + math.dist(a, b))
    length = arcs[-1] + math.dist(vertices[-1], vertices[0])
    return Loop(
        tuple(vertices),
        tuple(arcs),
        length,
        tuple(order),
        tuple(arcs[i] for i in cell_vertex),
    )


def _cycle(n_rows: int, n_cols: int) -> list[tuple[int, int]]:
    """Hamiltonian cycle of a grid with an even side: rows, then back along column 0."""
    if n_rows % 2 == 0:
        cycle = []
        for row in range(n_rows):
            cols = range(1, n_cols) if row % 2 == 0 else range(n_cols - 1, 0, -1)
            cycle.extend((row, col) for col in cols)
        cycle.extend((row, 0) for row in range(n_rows - 1, -1, -1))
        return cycle
    if n_cols % 2 == 0:
        return [(row, col) for col, row in _cycle(n_cols, n_rows)]
    raise ValueError("a closed loop needs an even number of rows or columns")


def loop_distance(
    cells: list[Cell] | tuple[Cell, ...], airspace: Airspace | None = None
) -> float:
    """In-region travel distance of an ordered waypoint list."""
    points = [c.position for c in cells]
    if airspace is None:
        return path_length(points)
    return sum(airspace.distance(a, b) for a, b in zip(points, points[1:]))


def airborne_time(
    cells: list[Cell] | tuple[Cell, ...],
    ferry_time: float,
    uav: UAVParams,
    airspace: Airspace | None = None,
) -> float:
    """Total airborne time of one sortie."""
    return 2.0 * ferry_time + loop_distance(cells, airspace) / uav.speed


def ferry_time(grid: Grid, depot: Point, uav: UAVParams) -> float:
    """One-way ferry time from the depot."""
    return grid.ferry_distance(depot) / uav.speed


def base_ferry_times(grid: Grid, airspace: Airspace, uav: UAVParams) -> list[float]:
    """One-way ferry time from each base to the farthest surveyed cell."""
    if not airspace.zones:
        return [ferry_time(grid, depot, uav) for depot in airspace.depots]
    return [
        max(airspace.depot_distance(c.position, m) for c in airspace.free_cells)
        / uav.speed
        for m in range(len(airspace.depots))
    ]


def assign_bases(routes: list[Route], airspace: Airspace) -> list[Route]:
    """Routes homed at the base nearest, on average, to their cells by flight."""
    if len(airspace.depots) == 1:
        return routes
    homed = []
    for route in routes:
        mean = [
            sum(airspace.depot_distance(c.position, m) for c in route.cells)
            for m in range(len(airspace.depots))
        ]
        homed.append(replace(route, depot=mean.index(min(mean))))
    return homed


def check_admissible(
    routes: list[Route],
    ferry: float,
    uav: UAVParams,
    airspace: Airspace | None = None,
) -> None:
    """Raise if any route exceeds the usable endurance."""
    for route in routes:
        t_air = airborne_time(route.cells, ferry, uav, airspace)
        if t_air > uav.usable_endurance:
            raise ValueError(
                f"route {route.id} is not admissible: T_airborne = {t_air:.0f} s "
                f"exceeds T_use = {uav.usable_endurance:.0f} s"
            )
