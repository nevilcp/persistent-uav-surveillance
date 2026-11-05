"""Stage 5: UAV failure management."""

from __future__ import annotations

from .coverage import CoverageMap
from .geometry import distance
from .grid import Cell
from .monitor import EventLog
from .params import Parameters
from .routing import Route, loop_distance
from .uav import UAV, UAVState


class FailureManager:
    """Failure injection and the bridge / take-over response."""

    def __init__(
        self,
        params: Parameters,
        uavs: list[UAV],
        coverage: CoverageMap,
        ferry_time: float,
        events: EventLog,
    ) -> None:
        self._cfg = params.failure
        self._uav = params.uav
        self._theta = params.mission.revisit_threshold
        self._uavs = uavs
        self._coverage = coverage
        self._ferry_time = ferry_time
        self._events = events

        self.failed_uav: UAV | None = None
        self.route: Route | None = None
        self.contingency: UAV | None = None
        self.t_fail: float | None = None
        self.t_handover: float | None = None
        self.n_inserts = 0
        self._first_cover: dict[int, float | None] = {}

    def due(self, time: float) -> bool:
        return self._cfg.enabled and self.failed_uav is None and time >= self._cfg.time

    def inject(self, time: float) -> Route:
        """Fail the UAV flying the configured route and return that route."""
        flying = [
            u
            for u in self._uavs
            if u.state is UAVState.ON_ROUTE
            and u.route
            and u.route.id == self._cfg.route
        ]
        if not flying:
            raise RuntimeError(f"no UAV is flying route {self._cfg.route} at t={time}")
        # prefer the UAV not already being replaced, then the most recently launched
        victim = min(flying, key=lambda u: (u.replaced, -u.launched_at))
        victim.state = UAVState.FAILED
        self.failed_uav, self.route, self.t_fail = victim, victim.route, time
        self._first_cover = {c.index: None for c in victim.route.cells}
        self._events.add(time, "failure", victim.id, f"route {victim.route.id}")
        return victim.route

    def bridge(self, time: float) -> None:
        """Insert urgent orphan cells into surviving routes until hand-over."""
        if self.failed_uav is None or self.t_handover is not None:
            return
        urgent = [
            c
            for c in self.route.cells
            if self._coverage.age(c.index, time) >= self._theta - self._cfg.age_margin
            and not self._is_bridged(c)
        ]
        urgent.sort(key=lambda c: -self._coverage.age(c.index, time))
        for cell in urgent:
            choice = self._best_assignee(cell)
            if choice is None:
                continue  # left to the contingency UAV
            uav, position = choice
            uav.insert_waypoint(position, cell)
            self.n_inserts += 1
            self._events.add(time, "bridge_insert", uav.id, f"cell {cell.index}")

    def _is_bridged(self, cell: Cell) -> bool:
        return any(
            u.state is UAVState.ON_ROUTE and cell.index in u.inserted
            for u in self._uavs
        )

    def _is_survivor(self, uav: UAV) -> bool:
        # the contingency UAV flies the failed route and already carries every orphan cell
        return uav.state is UAVState.ON_ROUTE and uav.route.id != self.route.id

    def _best_assignee(self, cell: Cell) -> tuple[UAV, int] | None:
        """Surviving UAV and insertion slot with the smallest loop-time increase."""
        best: tuple[float, int, UAV, int] | None = None
        for uav in self._uavs:
            if not self._is_survivor(uav):
                continue
            base = loop_distance(uav.waypoints)
            for position in range(len(uav.waypoints) + 1):
                extra = _insertion_cost(uav.waypoints, cell, position)
                new_airborne = 2.0 * self._ferry_time + (base + extra) / self._uav.speed
                if new_airborne > self._uav.usable_endurance:
                    continue
                key = (extra, uav.id)
                if best is None or key < best[:2]:
                    best = (extra, uav.id, uav, position)
        return None if best is None else (best[2], best[3])

    def check_handover(self, time: float) -> None:
        """Undo the bridging once the contingency UAV has reached the route."""
        spare = self.contingency
        if spare is None or self.t_handover is not None or spare.waypoints_reached < 1:
            return
        for uav in self._uavs:
            if uav.inserted:
                uav.remove_inserted_waypoints()
        self.t_handover = time
        self._events.add(time, "handover", spare.id, f"route {self.route.id}")

    def on_observation(self, cell_index: int, time: float) -> None:
        """Record the first observation of an orphan cell after the failure."""
        if cell_index in self._first_cover and self._first_cover[cell_index] is None:
            self._first_cover[cell_index] = time

    @property
    def recovery_time(self) -> float | None:
        """Recovery time; None while an orphan cell is still unobserved."""
        if self.t_fail is None or any(t is None for t in self._first_cover.values()):
            return None
        return max(t - self.t_fail for t in self._first_cover.values())


def _insertion_cost(waypoints: list[Cell], cell: Cell, position: int) -> float:
    """Loop-distance increase of inserting ``cell`` before ``position``."""
    before = waypoints[position - 1].position if position > 0 else None
    after = waypoints[position].position if position < len(waypoints) else None
    if before is None and after is None:
        return 0.0
    if before is None:
        return distance(cell.position, after)
    if after is None:
        return distance(before, cell.position)
    return (
        distance(before, cell.position)
        + distance(cell.position, after)
        - distance(before, after)
    )
