"""Core Cell data model for grid-based surveillance.

This module defines the surveillance cell structure used in the
discretized grid representation.
"""

from dataclasses import dataclass


@dataclass
class Cell:
    """Represents a single surveillance cell in the discretized grid.

    Attributes:
        id: Unique identifier for the cell
        x: X coordinate of cell center (m)
        y: Y coordinate of cell center (m)
        last_seen_ts: Timestamp of last observation (seconds since start)
        priority: Surveillance priority weight (1.0 = normal)
        is_covered: Whether cell is currently being observed
    """

    id: str
    x: float
    y: float
    last_seen_ts: float | None = None
    priority: float = 1.0
    is_covered: bool = False

    def __post_init__(self):
        """Validate cell parameters after initialization."""
        if self.priority <= 0:
            raise ValueError(f"Priority must be positive, got {self.priority}")

    def age(self, current_time: float) -> float:
        """Calculate age since last observation.

        Args:
            current_time: Current simulation time

        Returns:
            Age in seconds, or infinity if never observed
        """
        if self.last_seen_ts is None:
            return float("inf")
        return max(0, current_time - self.last_seen_ts)

    def update_observation(self, timestamp: float) -> None:
        """Update cell with new observation timestamp.

        Args:
            timestamp: Time of observation
        """
        self.last_seen_ts = timestamp
        self.is_covered = True
