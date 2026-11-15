"""Stage 2: route generation."""

from __future__ import annotations

import itertools
from collections.abc import Sequence
from dataclasses import dataclass, replace

from .airspace import Airspace
from .fleet import apportion
from .geometry import path_length
from .grid import Cell, Grid, choset_order
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
    """Round-robin assignment along Choset decomposition sweeps of ``cells``."""
    selected = list(grid.cells if cells is None else cells)
    if not 1 <= n_fly <= len(selected):
        raise ValueError("route count must be between one and the number of free cells")
    ordered = choset_order(selected)
    return [Route(id=u + 1, cells=tuple(ordered[u::n_fly])) for u in range(n_fly)]


def depot_round_robin_routes(grid: Grid, n_fly: int, airspace: Airspace) -> list[Route]:
    """Choset sweeps and round-robin within nearest-flyable-depot sectors.

    Flyers follow free-cell counts, with at least one per occupied sector.
    Equal inventories and balanced sectors are not assumed. Connector paths
    may leave a sector to avoid an NFZ.
    """
    sectors: list[list[Cell]] = [[] for _ in airspace.depots]
    for cell in airspace.free_cells:
        sectors[airspace.nearest_depot(cell.position)].append(cell)
    active = [m for m, cells in enumerate(sectors) if cells]
    if not len(active) <= n_fly <= len(airspace.free_cells):
        raise ValueError("need at least one flyer per occupied sector and at most one per free cell")
    extras = (apportion(n_fly - len(active), [len(sectors[m]) - 1 for m in active])
              if n_fly > len(active) else [0] * len(active))
    routes = []
    for m, extra in zip(active, extras):
        for route in round_robin_routes(grid, 1 + extra, sectors[m]):
            routes.append(replace(route, id=len(routes) + 1, depot=m))
    return routes


def loop_distance(
    cells: list[Cell] | tuple[Cell, ...], airspace: Airspace | None = None
) -> float:
    """In-region travel distance of an ordered waypoint list."""
    points = [c.position for c in cells]
    if airspace is None:
        return path_length(points)
    return sum(airspace.distance(a, b) for a, b in itertools.pairwise(points))


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
