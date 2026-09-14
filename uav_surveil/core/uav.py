"""Core UAV data model for swarm surveillance optimization.

This module defines the fundamental UAV data structure used throughout
the surveillance system.
"""

from dataclasses import dataclass, field
from enum import Enum

from .route import Route

# -----------------------------------------------------------------------------
# UAV operational state used by scheduler and metrics
# -----------------------------------------------------------------------------


class UAVState(Enum):
    """Operational state of a UAV.

    Attributes:
        SPARE: Parked at depot, never yet assigned (rotation pool).
        ON_MISSION: Currently executing a surveillance route.
        RTB: On mission, flying back to depot (returning to base).
        SWAPPING: On pad performing battery hot-swap.
        IDLE: At depot after a swap, waiting for its own phase slot to
            relaunch. Distinct from SPARE: reserved for its own route
            rather than available to cover another UAV's early return.
        FAILED: Hard failure; frozen in place.
    """

    SPARE = "spare"
    ON_MISSION = "on_mission"
    RTB = "rtb"
    SWAPPING = "swapping"
    IDLE = "idle"
    FAILED = "failed"


@dataclass
class UAV:
    """Represents a single UAV in the surveillance swarm.

    Attributes:
        id: Unique identifier for the UAV
        x: Current X coordinate position (m)
        y: Current Y coordinate position (m)
        soc: State of Charge as percentage (0.0-1.0)
        route_list: Ordered list of routes assigned to this UAV
        launch_time: Planned start time (s) assigned by scheduler
        is_active: Whether UAV is currently active in patrol
        last_update: Timestamp of last telemetry update
    """

    id: str
    x: float
    y: float
    soc: float  # State of Charge (0.0 to 1.0)
    route_list: list[Route]
    launch_time: float = 0.0  # Planned start time (s) assigned by scheduler
    is_active: bool = False
    # Extended attributes ---------------------------------------------------
    state: UAVState = UAVState.SPARE  # Initial state; updated by scheduler
    phase_offset: float | None = (
        None  # Route's initial departure phase; kept across swaps
    )
    swap_timer: float = 0.0  # Remaining hot-swap time (s)
    last_update: float | None = None
    is_contingency: bool = False  # True if this UAV is reserved for failure scenarios
    # Stage-5 failure handling ----------------------------------------------
    is_failed: bool = False  # Hard failure flag (removed from active control)
    tail_index_at_failure: int | None = (
        None  # Waypoint index at failure for tail takeover
    )
    last_insert_time: float = 0.0  # Timestamp of last temporary insert (bridge mode)
    temp_assignments: dict[str, int] = field(
        default_factory=dict
    )  # failed_id -> count of temporary inserts

    def __post_init__(self):
        """Validate UAV parameters after initialization."""
        if not 0.0 <= self.soc <= 1.0:
            raise ValueError(f"SoC must be between 0.0 and 1.0, got {self.soc}")
        if self.route_list is None:
            self.route_list = []

        # Ensure state/is_active consistency
        if self.is_active and self.state == UAVState.SPARE:
            self.state = UAVState.ON_MISSION

    @property
    def _fly_home(self) -> bool:
        """Read-only alias kept for one release so external readers (e.g.
        plotting scripts) that still check `_fly_home` keep working; the
        simulation itself now uses `state == UAVState.RTB` directly."""
        return self.state == UAVState.RTB

    def move_towards(
        self, target_x: float, target_y: float, speed: float, dt: float
    ) -> float:
        """Move UAV towards a target position by speed*dt (meters).
        Args:
            target_x: Target X coordinate
            target_y: Target Y coordinate
            speed: UAV speed (m/s)
            dt: Time step (s)
        """
        import math

        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)
        if dist == 0:
            return 0.0
        step = min(speed * dt, dist)
        self.x += dx / dist * step
        self.y += dy / dist * step
        return step

    def at_position(
        self, target_x: float, target_y: float, threshold: float = 2.0
    ) -> bool:
        """Check if UAV is within threshold distance of a target position (e.g., cell center)."""
        import math

        return math.hypot(self.x - target_x, self.y - target_y) <= threshold
