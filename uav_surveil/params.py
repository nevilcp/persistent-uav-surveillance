"""Simulation parameters; design choices are listed in docs/design_notes.md."""

from __future__ import annotations

import json
import typing
from dataclasses import dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any

Point = tuple[float, float]


@dataclass(frozen=True)
class RegionParams:
    """Surveillance region, grid resolution and depot."""

    width: float = 500.0  # extent of A along x (m)
    length: float = 500.0  # extent of A along y (m)
    cell_size: float = 40.0  # grid-cell side Delta (m)
    depot: Point = (-500.0, 0.0)  # depot D, 500 m west of A


@dataclass(frozen=True)
class UAVParams:
    """Airframe speed, endurance and turnaround."""

    speed: float = 6.0  # mission speed v_max (m/s)
    endurance: float = 2100.0  # total airborne endurance T_endure (s)
    sigma_min: float = 0.1  # minimum permissible battery level
    turnaround: float = 60.0  # depot turnaround time tau (s)
    # a cell is covered when its centroid is inside this radius; 0.75 Delta (design choice)
    sensing_radius: float = 30.0

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
class DispatchParams:
    """Stage 4 battery margins."""

    return_buffer: float = 0.02  # sigma_buf (design choice)
    prelaunch_margin: float = 0.06  # sigma_pre (design choice)


@dataclass(frozen=True)
class FailureParams:
    """A single forced UAV failure."""

    enabled: bool = False
    route: int = 10  # route (1-based) whose UAV fails; mid-sortie at the default t_f
    time: float = 1800.0  # failure time t_f (s)
    age_margin: float = 60.0  # delta_age (design choice)
    recovery_tolerance: float = (
        120.0  # epsilon in Theta_eps = Theta + eps (design choice)
    )


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
    kwargs = {}
    for name, value in data.items():
        if is_dataclass(hints[name]):
            value = _build(hints[name], value)
        elif hints[name] == Point:
            value = tuple(value)
        kwargs[name] = value
    return cls(**kwargs)
