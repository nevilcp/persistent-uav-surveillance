"""Simulation parameters; design choices are listed in docs/design-notes.md."""

from __future__ import annotations

import json
import types
import typing
from dataclasses import dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any

Point = tuple[float, float]
Rect = tuple[float, float, float, float]  # x0, y0, x1, y1


@dataclass(frozen=True)
class RegionParams:
    """Surveillance region, grid resolution, bases and no-fly zones."""

    width: float = 500.0  # extent of A along x (m)
    length: float = 500.0  # extent of A along y (m)
    cell_size: float = 40.0  # grid-cell side Delta (m)
    depot: Point = (-500.0, 0.0)  # depot D, 500 m west of A
    depots: tuple[Point, ...] = ()  # bases; empty means (depot,)
    zones: tuple[Rect, ...] = ()  # rectangular no-fly zones
    zone_margin: float = 0.0  # clearance beta kept around every zone (m)

    @property
    def bases(self) -> tuple[Point, ...]:
        """Effective base positions."""
        return self.depots or (self.depot,)


@dataclass(frozen=True)
class UAVParams:
    """Airframe speed, endurance and turnaround."""

    speed: float = 6.0  # mission speed v_max (m/s)
    endurance: float = 2100.0  # total airborne endurance T_endure (s)
    sigma_min: float = 0.1  # minimum permissible battery level
    turnaround: float = 60.0  # depot turnaround time tau (s)
    # a cell is covered when its centroid is inside this radius; 0.75 Delta (design choice)
    sensing_radius: float = 30.0
    depot_slots: int = 0  # kappa, UAVs swapped in parallel per base; 0 = unlimited

    @property
    def usable_endurance(self) -> float:
        """Usable airborne window."""
        return (1.0 - self.sigma_min) * self.endurance

    @property
    def range(self) -> float:
        """Distance flown on a full battery (m)."""
        return self.speed * self.endurance


@dataclass(frozen=True)
class FleetParams:
    """Fleet inventory and cost weights for the Stage 1 split."""

    size: int = 21  # inventory N
    batch_size: int = 4  # maximum UAVs per launch batch N_batch
    reserve_fraction: float = 0.03  # rho_min (design choice: one spare for N <= 33)
    fly_cost: float = 1.0  # c_fly (design choice; only c_reserve >= c_fly matters)
    reserve_cost: float = 1.2  # c_reserve >= c_fly (design choice)


@dataclass(frozen=True)
class ScheduleParams:
    """Staggered departure timing."""

    batch_period: float = 150.0  # T_batch (s)
    launch_gap: float = (
        60.0  # delta_launch >= tau (s); smallest valueallows (design choice)
    )


@dataclass(frozen=True)
class PatrolParams:
    """The supported path planner."""

    mode: str = "round_robin"  # Choset decomposition, then round-robin assignment


@dataclass(frozen=True)
class DispatchParams:
    """Stage 4 battery margins and distance metric."""

    return_buffer: float = 0.02  # sigma_buf (design choice)
    prelaunch_margin: float = 0.06  # sigma_pre (design choice)
    metric: str = "flyable"  # "flyable" | "straight" distance to the base


@dataclass(frozen=True)
class FailureParams:
    """Forced UAV failures and base closure."""

    enabled: bool = False
    route: int = 10  # route (1-based) whose UAV fails; mid-sortie at the default t_f
    time: float = 1800.0  # failure time t_f (s)
    age_margin: float = 60.0  # delta_age (design choice)
    recovery_tolerance: float = (
        120.0  # epsilon in Theta_eps = Theta + eps (design choice)
    )
    later_times: tuple[float, ...] = ()  # further breakdowns (s), same victim rule
    near_depot: int | None = None  # victim is the loop UAV nearest this base
    closed_depot: int | None = None  # base closed at closure_time
    closure_time: float = 1800.0  # (s)


@dataclass(frozen=True)
class MissionParams:
    """Simulation horizon and revisit specification."""

    duration: float = 4800.0  # simulated time (s)
    time_step: float = 1.0  # GSM sampling period (s); 1 Hz by default
    revisit_threshold: float = 180.0  # Theta (s)
    steady_state_start: float = 1200.0  # t_ss in(s) (design choice)


@dataclass(frozen=True)
class Parameters:
    """Complete parameter set of one run."""

    region: RegionParams = field(default_factory=RegionParams)
    uav: UAVParams = field(default_factory=UAVParams)
    fleet: FleetParams = field(default_factory=FleetParams)
    schedule: ScheduleParams = field(default_factory=ScheduleParams)
    patrol: PatrolParams = field(default_factory=PatrolParams)
    dispatch: DispatchParams = field(default_factory=DispatchParams)
    failure: FailureParams = field(default_factory=FailureParams)
    mission: MissionParams = field(default_factory=MissionParams)

    def __post_init__(self) -> None:
        if not 0.0 < self.uav.sigma_min < 1.0:
            raise ValueError("sigma_min must lie in (0, 1)")
        if not 0.0 <= self.fleet.reserve_fraction < 1.0:
            raise ValueError("rho_min (reserve_fraction) must lie in [0, 1)")
        if self.schedule.launch_gap < self.uav.turnaround:
            raise ValueError("launch_gap must be at least turnaround")
        if self.dispatch.prelaunch_margin <= self.dispatch.return_buffer:
            raise ValueError("prelaunch_margin must exceed return_buffer")
        if self.fleet.reserve_cost < self.fleet.fly_cost:
            raise ValueError("Stage 1 requires reserve_cost >= fly_cost")
        self._validate_airspace()
        self._validate_patrol()

    def _validate_airspace(self) -> None:
        region, failure = self.region, self.failure
        if region.zone_margin < 0.0:
            raise ValueError("zone_margin must be non-negative")
        if self.uav.depot_slots < 0:
            raise ValueError("depot_slots must be non-negative")
        for x0, y0, x1, y1 in region.zones:
            if not (x0 < x1 and y0 < y1):
                raise ValueError(f"zone {(x0, y0, x1, y1)} needs x0 < x1 and y0 < y1")
        count = len(region.bases)
        for name in ("near_depot", "closed_depot"):
            index = getattr(failure, name)
            if index is not None and not 0 <= index < count:
                raise ValueError(f"{name} must index one of the {count} bases")
        if failure.closed_depot is not None and count < 2:
            raise ValueError("closing a base needs at least two bases")
        if failure.closure_time < 0.0:
            raise ValueError("closure_time must be non-negative")
        if any(t < 0.0 for t in failure.later_times):
            raise ValueError("later_times must be non-negative")
        if self.dispatch.metric not in ("flyable", "straight"):
            raise ValueError("dispatch metric must be 'flyable' or 'straight'")

    def _validate_patrol(self) -> None:
        patrol = self.patrol
        if patrol.mode != "round_robin":
            raise ValueError("patrol mode must be 'round_robin' (Choset decomposition); shared_loop was removed")

    @classmethod
    def from_json(cls, path: str | Path) -> Parameters:
        """Load a parameter set; omitted entries keep their default value."""
        with open(path) as handle:
            return _build(cls, json.load(handle))


def _build(cls: type, data: dict[str, Any]) -> Any:
    """Instantiate (nested) dataclass ``cls`` from a plain dict."""
    hints = typing.get_type_hints(cls)
    unknown = set(data) - {f.name for f in fields(cls)}
    if unknown:
        raise ValueError(f"unknown {cls.__name__} entries: {sorted(unknown)}")
    return cls(**{name: _coerce(hints[name], value) for name, value in data.items()})


def _coerce(hint: Any, value: Any) -> Any:
    """Convert a JSON value to ``hint``: dataclasses from dicts, tuples from lists."""
    if is_dataclass(hint):
        return _build(hint, value)
    if value is None:
        return None
    if typing.get_origin(hint) in (typing.Union, types.UnionType):
        inner = [a for a in typing.get_args(hint) if a is not type(None)]
        return _coerce(inner[0], value)
    if typing.get_origin(hint) is tuple:
        args = typing.get_args(hint)
        if len(args) == 2 and args[1] is Ellipsis:
            return tuple(_coerce(args[0], v) for v in value)
        return tuple(_coerce(a, v) for a, v in zip(args, value, strict=True))
    return value
