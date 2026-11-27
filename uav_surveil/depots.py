"""Bases: turnaround service queue, open/closed flag and reserve pools."""

from __future__ import annotations

from collections import deque
from collections.abc import Sequence

from .airspace import Airspace
from .geometry import distance
from .params import Point
from .uav import UAV, UAVState


class Depot:
    """One base with a FIFO turnaround queue."""

    def __init__(self, index: int, position: Point, slots: int = 0) -> None:
        self.index = index
        self.position = position
        self.slots = slots  # kappa; 0 = unlimited
        self.is_open = True
        self.closed_at: float | None = None
        self.in_service: list[UAV] = []
        self.queue: deque[UAV] = deque()

    @property
    def waiting(self) -> int:
        """Q_m: UAVs waiting for a free slot."""
        return len(self.queue)

    def arrive(self, uav: UAV, turnaround: float) -> None:
        """Start an arriving UAV's turnaround, or queue it when all slots are busy."""
        if self.slots and len(self.in_service) >= self.slots:
            uav.state = UAVState.QUEUED
            self.queue.append(uav)
        else:
            self._start(uav, turnaround)

    def advance(self, dt: float, turnaround: float) -> None:
        """Progress every turnaround by ``dt``, then fill freed slots from the queue."""
        for uav in list(self.in_service):
            uav.swap_remaining -= dt
            if uav.swap_remaining <= 0.0:
                self.in_service.remove(uav)
                uav.battery = 1.0
                uav.state = UAVState.READY
                uav.clear_route()
        while self.queue and (not self.slots or len(self.in_service) < self.slots):
            self._start(self.queue.popleft(), turnaround)

    def close(self, time: float) -> None:
        """Stop accepting returning UAVs."""
        self.is_open = False
        self.closed_at = time

    def _start(self, uav: UAV, turnaround: float) -> None:
        uav.state = UAVState.SWAPPING
        uav.swap_remaining = turnaround
        self.in_service.append(uav)


def reserve_pool(uavs: Sequence[UAV], base: int, contingency: bool) -> list[UAV]:
    """Parked UAVs of a base waiting for a route, rotation or spare."""
    return [
        u
        for u in uavs
        if u.state is UAVState.READY
        and u.route is None
        and u.home == base
        and u.is_contingency == contingency
    ]


def nearest_open_base(
    airspace: Airspace, depots: Sequence[Depot], position: Point, metric: str
) -> tuple[int, float]:
    """Closest open base and the distance the dispatch rules use to it."""
    open_bases = [d.index for d in depots if d.is_open]
    if metric == "straight":
        base = min(open_bases, key=lambda m: distance(position, airspace.depots[m]))
        return base, distance(position, airspace.depots[base])
    base = airspace.nearest_depot(position, open_bases)
    return base, airspace.depot_distance(position, base)
