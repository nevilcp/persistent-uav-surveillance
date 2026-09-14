"""Centralized parameter configuration for UAV surveillance system.

This module defines all tunable parameters for the surveillance system,
enabling easy experimentation and scenario management.
"""

from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field, field_validator
from pathlib import Path
import json
from datetime import datetime


class MissionParameters(BaseModel):
    """Mission-level parameters defining the surveillance task."""

    area_width: float = Field(
        default=500.0, ge=50.0, le=5000.0, description="Surveillance area width (m)"
    )
    area_length: float = Field(
        default=500.0, ge=50.0, le=5000.0, description="Surveillance area length (m)"
    )
    depot_x: float = Field(default=0.0, description="Depot X coordinate (m)")
    depot_y: float = Field(default=0.0, description="Depot Y coordinate (m)")
    max_ferry_distance: float = Field(
        default=500.0,
        ge=0.0,
        description="Maximum ferry distance from depot to grid edge (m)",
    )
    mission_duration: float = Field(
        default=3600.0, ge=60.0, description="Total mission duration (seconds)"
    )

    @field_validator("area_width", "area_length")
    @classmethod
    def validate_positive_dimensions(cls, v):
        if v <= 0:
            raise ValueError("Area dimensions must be positive")
        return v


class UAVParameters(BaseModel):
    """UAV platform specifications and performance parameters."""

    cruise_speed: float = Field(
        default=6.0, ge=0.5, le=20.0, description="UAV cruise velocity (m/s)"
    )
    flight_altitude: float = Field(
        default=50.0,
        ge=10.0,
        le=150.0,
        description="Standard flight altitude above ground (m)",
    )


class BatteryParameters(BaseModel):
    """Battery and power management parameters."""

    total_endurance: float = Field(
        default=2100.0,
        ge=300.0,
        le=10800.0,
        description="Total battery endurance (seconds)",
    )
    usable_endurance: float = Field(
        default=1890.0,
        ge=270.0,
        le=9720.0,
        description="Usable endurance after reserves (seconds)",
    )
    soc_floor: float = Field(
        default=0.1,
        ge=0.05,
        le=0.3,
        description="State of charge safety floor (fraction)",
    )
    soc_return_threshold: float = Field(
        default=0.2,
        ge=0.1,
        le=0.5,
        description="SoC threshold to trigger return (fraction)",
    )
    hot_swap_time: float = Field(
        default=60.0,
        ge=10.0,
        le=300.0,
        description="Battery hot-swap time on pad (seconds)",
    )
    charging_time: float = Field(
        default=3600.0,
        ge=1800.0,
        le=14400.0,
        description="Full battery charging time (seconds)",
    )
    prelaunch_margin: float = Field(
        default=0.02,
        ge=0.0,
        le=0.2,
        description="Stage 4 ETA pre-launch SoC margin epsilon (fraction)",
    )

    @field_validator("usable_endurance")
    @classmethod
    def validate_usable_endurance(cls, v, info):
        if "total_endurance" in info.data and v > info.data["total_endurance"]:
            raise ValueError("Usable endurance cannot exceed total endurance")
        return v


class GridParameters(BaseModel):
    """Grid discretization and coverage parameters."""

    cell_size: float = Field(
        default=40.0, ge=10.0, le=200.0, description="Grid cell side length (m)"
    )
    priority_zones: Dict[str, float] = Field(
        default_factory=dict,
        description="Special priority zones {zone_id: priority_weight}",
    )
    exclusion_zones: List[Dict[str, Any]] = Field(
        default_factory=list, description="No-fly zones as list of polygon definitions"
    )
    origin: tuple[float, float] = Field(
        default=(0.0, 0.0),
        description="(x, y) coordinates of the southwest corner of the grid",
    )

    def get_cell_count(self, area_width: float, area_length: float) -> tuple[int, int]:
        """Calculate number of cells in each dimension."""
        cols = int(area_width / self.cell_size)
        rows = int(area_length / self.cell_size)
        return rows, cols


class OptimizationParameters(BaseModel):
    """Optimization algorithm parameters and thresholds."""

    # Stage 0 - Battery
    max_battery_reserve: float = Field(
        default=0.1,
        ge=0.05,
        le=0.3,
        description="Maximum allowed battery reserve (xi_max)",
    )

    # Stage 2 - Fleet
    spare_floor_ratio: float = Field(
        default=0.2, ge=0.0, le=0.5, description="Minimum spare UAV ratio"
    )
    total_inventory: int = Field(
        default=4,
        ge=1,
        le=100,
        description="Total number of UAV airframes available (K_inv)",
    )
    max_fleet_budget: float = Field(
        default=1000000.0, ge=50000.0, description="Maximum fleet acquisition budget"
    )
    uav_unit_cost: float = Field(
        default=50000.0, ge=10000.0, le=200000.0, description="Cost per UAV unit"
    )
    operating_cost_per_hour: float = Field(
        default=100.0, ge=10.0, le=1000.0, description="Operating cost per UAV per hour"
    )
    use_enhanced_fleet: bool = Field(
        default=False,
        description="Use enhanced fleet sizing with rotation/contingency distinction",
    )

    # Stage 3A - Route generation
    route_optimization_time_limit: float = Field(
        default=30.0,
        ge=1.0,
        le=300.0,
        description="Time limit for route optimization (seconds)",
    )
    alns_iterations: int = Field(
        default=1000, ge=100, le=10000, description="ALNS algorithm iteration limit"
    )
    # New: choose route algorithm (factory name)
    route_algorithm: str = Field(
        default="greedy",
        description="Route generation algorithm: greedy, kmnn, rr, alns, milp",
    )
    furthest_first: bool = Field(
        default=False,
        description="Start with furthest cells first (supervisor's enhancement)",
    )

    # Stage 3B - Batch scheduler
    batch_size: int = Field(
        default=4, ge=1, le=10, description="Number of UAVs per departure batch"
    )
    batch_period: float = Field(
        default=30.0,
        ge=10.0,
        le=300.0,
        description="Time between batch departures (seconds)",
    )

    # Stage 5 - Re-routing
    emergency_replan_time_limit: float = Field(
        default=5.0,
        ge=0.5,
        le=30.0,
        description="Time limit for emergency re-planning (seconds)",
    )


class FailureTriggerConfig(BaseModel):
    """Configurable failure trigger (time|position|soc)."""

    kind: str = Field(default="off", description="Trigger type: off|time|position|soc")
    uav_id: Optional[str] = Field(default=None, description="Target UAV id for failure")
    t_s: Optional[float] = Field(
        default=None, description="Failure time (s) for kind=time"
    )
    x: Optional[float] = Field(
        default=None, description="X coordinate for kind=position"
    )
    y: Optional[float] = Field(
        default=None, description="Y coordinate for kind=position"
    )
    soc_threshold: Optional[float] = Field(
        default=None, description="SoC threshold for kind=soc"
    )


class BridgePolicyConfig(BaseModel):
    """Stage-5a bridging policy knobs."""

    realloc_k: int = Field(
        default=2, ge=1, le=8, description="k nearest neighbors to consider"
    )
    realloc_horizon_s: float = Field(
        default=180.0,
        ge=30.0,
        le=900.0,
        description="Stop bridging if contingency ETA within horizon",
    )
    realloc_age_guard_s: float = Field(
        default=60.0,
        ge=0.0,
        le=600.0,
        description="Age guard before max gap to mark urgent",
    )
    bridge_tick_s: float = Field(
        default=10.0, ge=1.0, le=120.0, description="Bridge reassignment tick interval"
    )
    min_hold_time_s: float = Field(
        default=60.0,
        ge=0.0,
        le=600.0,
        description="Min time to hold a temp cell before re-evaluating",
    )
    max_inserts_per_uav: int = Field(
        default=1, ge=1, le=5, description="Cap of temporary cells per UAV"
    )
    max_detour_ratio: float = Field(
        default=0.2,
        ge=0.05,
        le=0.5,
        description="Max detour as fraction of remaining loop",
    )
    handover_grace_s: float = Field(
        default=15.0,
        ge=0.0,
        le=120.0,
        description="Freeze bridge near handover to avoid churn",
    )


class FailureParameters(BaseModel):
    """Stage-5 failure handling master config."""

    enabled: bool = Field(default=False, description="Enable Stage-5 failure handling")
    trigger: FailureTriggerConfig = Field(default_factory=FailureTriggerConfig)
    bridge_policy: BridgePolicyConfig = Field(default_factory=BridgePolicyConfig)


class STLParameters(BaseModel):
    """Signal Temporal Logic contract parameters."""

    # C-1: Coverage contract
    max_revisit_gap: float = Field(
        default=120.0,
        ge=30.0,
        le=600.0,
        description="Maximum allowed revisit gap Θ (seconds)",
    )

    # C-3: Spare launch contract
    spare_launch_deadline: float = Field(
        default=1.0, ge=0.1, le=10.0, description="Spare launch deadline (seconds)"
    )

    # Robustness monitoring
    robustness_warning_threshold: float = Field(
        default=5.0,
        ge=1.0,
        le=30.0,
        description="Robustness warning threshold (seconds)",
    )
    robustness_critical_threshold: float = Field(
        default=1.0,
        ge=0.1,
        le=10.0,
        description="Robustness critical threshold (seconds)",
    )


class SimulationParameters(BaseModel):
    """Simulation and verification parameters."""

    time_step: float = Field(
        default=1.0, ge=0.1, le=5.0, description="Simulation time step (seconds)"
    )
    gss_frequency: float = Field(
        default=1.0,
        ge=0.1,
        le=10.0,
        description="Ground Station Scheduler frequency (Hz)",
    )
    monte_carlo_runs: int = Field(
        default=1000,
        ge=100,
        le=10000,
        description="Monte Carlo simulation runs for V&V",
    )
    random_seed: Optional[int] = Field(
        default=42, description="Random seed for reproducibility"
    )
    failure_rate: float = Field(
        default=0.001, ge=0.0, le=0.1, description="UAV failure rate per second"
    )
    weather_enabled: bool = Field(
        default=False, description="Enable weather effects simulation"
    )
    real_time_factor: float = Field(
        default=1.0,
        ge=0.1,
        le=100.0,
        description="Real-time simulation factor (1.0 = real-time)",
    )


class SystemParameters(BaseModel):
    """Complete system parameter configuration."""

    # Metadata
    config_name: str = Field(
        default="baseline", description="Configuration name/identifier"
    )
    description: str = Field(
        default="Baseline UAV surveillance configuration",
        description="Configuration description",
    )
    created_at: datetime = Field(
        default_factory=datetime.now, description="Configuration creation timestamp"
    )
    version: str = Field(default="1.0", description="Configuration version")

    # Parameter groups
    mission: MissionParameters = Field(default_factory=MissionParameters)
    uav: UAVParameters = Field(default_factory=UAVParameters)
    battery: BatteryParameters = Field(default_factory=BatteryParameters)
    grid: GridParameters = Field(default_factory=GridParameters)
    optimization: OptimizationParameters = Field(default_factory=OptimizationParameters)
    stl: STLParameters = Field(default_factory=STLParameters)
    simulation: SimulationParameters = Field(default_factory=SimulationParameters)
    failure: FailureParameters = Field(default_factory=FailureParameters)

    model_config = {"arbitrary_types_allowed": True}

    def save_to_file(self, filepath: Path) -> None:
        """Save configuration to JSON file."""
        with open(filepath, "w") as f:
            json.dump(self.model_dump(), f, indent=2, default=str)

    @classmethod
    def load_from_file(cls, filepath: Path) -> "SystemParameters":
        """Load configuration from JSON file."""
        with open(filepath, "r") as f:
            data = json.load(f)
        return cls(**data)

    def get_battery_constraint_params(self) -> dict:
        """Get parameters for Stage 0 battery constraint."""
        return {
            "d_ferry": self.mission.max_ferry_distance,
            "v_max": self.uav.cruise_speed,
            "endurance": self.battery.total_endurance,
            "soc_floor": self.battery.soc_floor,
            "xi_max": self.optimization.max_battery_reserve,
        }

    def get_grid_build_params(self) -> dict:
        """Get parameters for Stage 1 grid building."""
        return {
            "area_width": self.mission.area_width,
            "area_length": self.mission.area_length,
            "cell_size": self.grid.cell_size,
            "origin": self.grid.origin,
        }

    def get_fleet_optimization_params(self) -> dict:
        """Get parameters for Stage 2 fleet optimization."""
        return {
            "spare_floor_ratio": self.optimization.spare_floor_ratio,
            "max_budget": self.optimization.max_fleet_budget,
            "uav_cost": self.optimization.uav_unit_cost,
            "operating_cost_per_hour": self.optimization.operating_cost_per_hour,
            "total_inventory": self.optimization.total_inventory,
        }

    def validate_consistency(self) -> List[str]:
        """Validate parameter consistency across modules."""
        warnings = []

        # Check battery vs mission consistency
        max_mission_distance = 2 * self.mission.max_ferry_distance
        max_battery_distance = self.uav.cruise_speed * self.battery.usable_endurance

        if max_mission_distance > max_battery_distance:
            warnings.append(
                f"Mission ferry distance ({max_mission_distance}m) may exceed "
                f"battery capacity ({max_battery_distance}m)"
            )

        # Check grid vs coverage consistency
        total_cells = self.grid.get_cell_count(
            self.mission.area_width, self.mission.area_length
        )
        min_coverage_time = total_cells[0] * total_cells[1] * 10  # Rough estimate

        if min_coverage_time > self.stl.max_revisit_gap:
            warnings.append(
                f"Grid size may require longer coverage time than STL limit "
                f"({min_coverage_time}s > {self.stl.max_revisit_gap}s)"
            )

        return warnings

    def summary(self) -> str:
        """Generate human-readable parameter summary."""
        return f"""
UAV Surveillance System Configuration: {self.config_name}
═══════════════════════════════════════════════════════

Mission Profile:
  • Area: {self.mission.area_width}×{self.mission.area_length}m
  • Max ferry: {self.mission.max_ferry_distance}m
  • Duration: {self.mission.mission_duration/3600:.1f}h

UAV Specifications:
  • Cruise speed: {self.uav.cruise_speed} m/s
  • Battery endurance: {self.battery.total_endurance/60:.1f} min
  • SoC floor: {self.battery.soc_floor:.1%}

Grid Configuration:
  • Cell size: {self.grid.cell_size}m
  • Grid dimensions: {self.grid.get_cell_count(self.mission.area_width, self.mission.area_length)}

STL Contracts:
  • Max revisit gap: {self.stl.max_revisit_gap}s
  • Spare launch deadline: {self.stl.spare_launch_deadline}s

Fleet Budget:
  • Max budget: ${self.optimization.max_fleet_budget:,.0f}
  • UAV cost: ${self.optimization.uav_unit_cost:,.0f}
  • Spare ratio: {self.optimization.spare_floor_ratio:.1%}
        """.strip()
