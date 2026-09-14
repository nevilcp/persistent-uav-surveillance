"""Core Route data model for UAV path planning.

This module defines the route structure used for UAV navigation
and surveillance path optimization.
"""

from dataclasses import dataclass


@dataclass
class Route:
    """Represents a surveillance route for a UAV.

    This is a placeholder class that will be expanded with full
    path planning and optimization functionality.

    Attributes:
        id: Unique identifier for the route
        cell_sequence: Ordered list of cell IDs to visit
        estimated_duration: Expected route completion time (seconds)
        departure_time: Scheduled departure timestamp
        loop_time: Total loop duration including return to depot
    """

    id: str
    cell_sequence: list[str]
    estimated_duration: float | None = None
    departure_time: float | None = None
    loop_time: float | None = None

    def __post_init__(self):
        """Validate route parameters after initialization."""
        if not self.cell_sequence:
            raise ValueError("Route must contain at least one cell")

    def get_waypoint_count(self) -> int:
        """Get the number of waypoints in this route.

        Returns:
            Number of cells to visit in this route
        """
        return len(self.cell_sequence)

    def is_valid(self) -> bool:
        """Check if route parameters are valid.

        Returns:
            True if route has valid configuration
        """
        return (
            bool(self.cell_sequence)
            and (self.estimated_duration is None or self.estimated_duration > 0)
            and (self.loop_time is None or self.loop_time > 0)
        )

    def get_waypoints(self, cell_lookup: dict) -> list:
        """Return the list of (x, y) waypoints for this route using a cell
        lookup dict."""
        return [(cell_lookup[cid].x, cell_lookup[cid].y) for cid in self.cell_sequence]
