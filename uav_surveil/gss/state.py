"""Ground Station Scheduler (GSS) state management.

This module defines the central state model for the surveillance system
using Pydantic for validation and serialization.
"""

from pydantic import BaseModel, Field, field_validator

from ..core.cell import Cell
from ..core.uav import UAV


class State(BaseModel):
    """Central state model for the Ground Station Scheduler.

    This model maintains the complete system state including UAV fleet,
    surveillance grid, timing, and operational flags.

    Attributes:
        clock: Current simulation time (seconds since start)
        uavs: List of all UAVs in the system
        cells: List of all surveillance cells
        spare_count: Number of spare UAVs available on pad
        need_route_update: Flag indicating routes need recalculation
        need_spare: Flag indicating spare UAV launch required
        total_coverage_gaps: Running count of coverage violations
        max_revisit_gap: Current maximum revisit gap (seconds)
    """

    clock: int = Field(default=0, ge=0, description="Simulation time in seconds")
    uavs: list[UAV] = Field(default_factory=list, description="Fleet of UAVs")
    cells: list[Cell] = Field(
        default_factory=list, description="Surveillance grid cells"
    )
    spare_count: int = Field(default=0, ge=0, description="Available spare UAVs")
    need_route_update: bool = Field(default=False, description="Route update flag")
    need_spare: bool = Field(default=False, description="Spare launch flag")
    total_coverage_gaps: int = Field(
        default=0, ge=0, description="Coverage violations count"
    )
    max_revisit_gap: float = Field(
        default=0.0, ge=0.0, description="Maximum revisit gap"
    )

    model_config = {"arbitrary_types_allowed": True, "validate_assignment": True}

    @field_validator("uavs")
    @classmethod
    def validate_uav_ids_unique(cls, v):
        """Ensure all UAV IDs are unique."""
        if v:
            ids = [uav.id for uav in v]
            if len(ids) != len(set(ids)):
                raise ValueError("UAV IDs must be unique")
        return v

    @field_validator("cells")
    @classmethod
    def validate_cell_ids_unique(cls, v):
        """Ensure all cell IDs are unique."""
        if v:
            ids = [cell.id for cell in v]
            if len(ids) != len(set(ids)):
                raise ValueError("Cell IDs must be unique")
        return v

    def get_active_uavs(self) -> list[UAV]:
        """Get list of currently active UAVs.

        Returns:
            List of UAVs that are currently active in patrol
        """
        return [uav for uav in self.uavs if uav.is_active]

    def get_total_fleet_size(self) -> int:
        """Get total fleet size including spares.

        Returns:
            Total number of UAVs (active + spare)
        """
        return len(self.uavs) + self.spare_count

    def update_clock(self, new_time: int) -> None:
        """Update system clock with validation.

        Args:
            new_time: New simulation time

        Raises:
            ValueError: If new_time is less than current clock
        """
        if new_time < self.clock:
            raise ValueError(f"Time cannot go backwards: {new_time} < {self.clock}")
        self.clock = new_time

    def calculate_max_revisit_gap(self) -> float:
        """Calculate current maximum revisit gap across all cells.

        Returns:
            Maximum time since last observation across all cells
        """
        if not self.cells:
            return 0.0

        max_gap = 0.0
        for cell in self.cells:
            gap = cell.age(float(self.clock))
            if gap != float("inf"):  # Skip never-observed cells
                max_gap = max(max_gap, gap)

        self.max_revisit_gap = max_gap
        return max_gap
