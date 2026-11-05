"""Stage 2: route generation."""

from __future__ import annotations

from dataclasses import dataclass

from .geometry import path_length
from .grid import Cell, Grid, boustrophedon_order
from .params import Point, UAVParams


@dataclass(frozen=True)
class Route:
    """Cell list and departure time of one UAV slot; a replacement UAV inherits it."""

    id: int  # 1-based
    cells: tuple[Cell, ...]  # ordered cell list I_u
    departure: float = 0.0  # scheduled departure d_u (s)

    @property
    def waypoints(self) -> list[Point]:
        return [c.position for c in self.cells]


def round_robin_routes(grid: Grid, n_fly: int) -> list[Route]:
    """Round-robin assignment along the boustrophedon order."""
    ordered = boustrophedon_order(grid.cells)
    return [Route(id=u + 1, cells=tuple(ordered[u::n_fly])) for u in range(n_fly)]


def loop_distance(cells: list[Cell] | tuple[Cell, ...]) -> float:
    """In-region travel distance of an ordered waypoint list."""
    return path_length([c.position for c in cells])


def airborne_time(
    cells: list[Cell] | tuple[Cell, ...], ferry_time: float, uav: UAVParams
) -> float:
    """Total airborne time of one sortie."""
    return 2.0 * ferry_time + loop_distance(cells) / uav.speed


def ferry_time(grid: Grid, depot: Point, uav: UAVParams) -> float:
    """One-way ferry time from the depot."""
    return grid.ferry_distance(depot) / uav.speed


def check_admissible(routes: list[Route], ferry: float, uav: UAVParams) -> None:
    """Raise if any route exceeds the usable endurance."""
    for route in routes:
        t_air = airborne_time(route.cells, ferry, uav)
        if t_air > uav.usable_endurance:
            raise ValueError(
                f"route {route.id} is not admissible: T_airborne = {t_air:.0f} s "
                f"exceeds T_use = {uav.usable_endurance:.0f} s"
            )
