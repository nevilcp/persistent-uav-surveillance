"""State of a single UAV."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from .geometry import step_towards
from .grid import Cell
from .params import Point
from .routing import Route


class UAVState(Enum):
    READY = "ready"  # parked at the depot with a full battery
    ON_ROUTE = "on_route"  # ferrying to / looping on its route
    RETURNING = "returning"  # flying straight home
    RELAYING = "relaying"  # shared loop: flying to, or hovering at, a hand-over point
    QUEUED = "queued"  # at a base whose turnaround slots are all busy
    SWAPPING = "swapping"  # depot turnaround of duration tau
    FAILED = "failed"  # down in place; takes no further part


@dataclass(eq=False)
class UAV:
    """One airframe; after a turnaround it rejoins the rotation pool."""

    id: int
    position: Point
    battery: float = 1.0  # b_u in [0, 1]
    state: UAVState = UAVState.READY
    is_contingency: bool = False  # member of U_spare, reserved for failures

    # waypoints is a working copy of route.cells that bridging may extend
    route: Route | None = None
    waypoints: list[Cell] = field(default_factory=list)
    target: int = 0
    waypoints_reached: int = 0  # waypoints visited since launch
    inserted: set[int] = field(default_factory=set)  # cells bridged for a failed UAV

    home: int = 0  # index of the base the UAV is parked at or heading to
    leg: list[Point] = field(default_factory=list)  # points left on the way to leg_goal
    leg_goal: Point | None = None  # destination the leg was planned for

    launch_at: float | None = None  # scheduled initial departure d_u
    launched_at: float = -math.inf
    replaced: bool = False  # a replacement has been requested this sortie
    swap_remaining: float = 0.0  # turnaround time left (s)

    # shared loop only; a looping UAV is ON_ROUTE with no route
    arc: float = 0.0  # position along the loop (m)
    meet_arc: float = 0.0  # loop position of the hand-over this relief flies to
    meet_ahead: float = 0.0  # loop distance the relieved UAV still has to cover
    relieving: UAV | None = None  # incumbent being relieved; None for a virtual slot
    left_at: float = 0.0  # time the UAV left the loop
    cut_after: float | None = None  # staggered first sortie: relief requested after (s)

    @property
    def airborne(self) -> bool:
        """Whether the UAV is in the air."""
        return self.state in (UAVState.ON_ROUTE, UAVState.RETURNING, UAVState.RELAYING)

    def launch(self, route: Route, time: float) -> None:
        """Start flying ``route`` from its first waypoint."""
        self.route = route
        self.waypoints = list(route.cells)
        self.target = 0
        self.waypoints_reached = 0
        self.inserted = set()
        self.set_leg(None, [])
        self.launch_at = None
        self.launched_at = time
        self.replaced = False
        self.is_contingency = False
        self.state = UAVState.ON_ROUTE

    def clear_route(self) -> None:
        """Drop the current route."""
        self.route = None
        self.waypoints = []
        self.target = 0
        self.inserted = set()
        self.set_leg(None, [])

    def set_leg(self, goal: Point | None, leg: list[Point]) -> None:
        """Plan the remaining flight to ``goal`` along ``leg``."""
        self.leg_goal = goal
        self.leg = leg

    def insert_waypoint(self, position: int, cell: Cell) -> None:
        """Temporarily insert ``cell`` before waypoint ``position``."""
        # insertions are removed by cell index, so a duplicate would take the UAV's own waypoint
        if any(c.index == cell.index for c in self.waypoints):
            raise ValueError(
                f"cell {cell.index} is already a waypoint of UAV {self.id}"
            )
        self.waypoints.insert(position, cell)
        if position < self.target:
            self.target += 1
        self.inserted.add(cell.index)

    def remove_inserted_waypoints(self) -> None:
        """Restore the original waypoint list, keeping progress."""
        kept: list[Cell] = []
        removed_before_target = 0
        for i, cell in enumerate(self.waypoints):
            if cell.index in self.inserted:
                removed_before_target += i < self.target
            else:
                kept.append(cell)
        self.target = (self.target - removed_before_target) % max(len(kept), 1)
        self.waypoints = kept
        self.inserted = set()

    def fly_leg(self, max_step: float) -> tuple[float, bool]:
        """Advance along the leg; returns distance flown and arrival at its end."""
        flown = 0.0
        while self.leg:
            # leftover step carries over detour vertices, but not the final arrival
            self.position, step, arrived = step_towards(
                self.position, self.leg[0], max_step - flown
            )
            flown += step
            if not arrived:
                return flown, False
            self.leg.pop(0)
        return flown, True
