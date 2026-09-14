"""
GSS (Guidance, Surveillance, and Safety) Simulation Engine.

This module implements the main simulation loop that integrates all stages:
- Stage 0: Battery optimization
- Stage 1: Grid generation
- Stage 2: Fleet sizing
- Stage 3: Route generation and scheduling

The simulation runs in discrete time steps and tracks UAV states, cell coverage,
and system performance metrics.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Tuple
from enum import Enum
import csv
import os
from datetime import datetime

import simpy

from ..core.cell import Cell
from ..core.route import Route
from ..core.uav import UAV, UAVState
from ..config.config_manager import ConfigManager
from ..config.parameters import SystemParameters
from ..stage0_battery import optimize_battery_reserve
from ..stage1_grid import build_grid_from_config
from ..stage2_fleet import optimize_fleet_from_config
from ..stage3_route_factory import get_route_generator
from ..stage3_schedule import schedule_from_config
from ..stage4_policy import apply_policy
from ..stage5_failure import FailureManager
from ..analysis_tools import (
    log_route_analysis,
    log_cell_coverage_gaps,
    track_uav_routes,
)
import numpy as np


class SimulationState(Enum):
    """Current state of the simulation."""

    INITIALIZING = "initializing"
    RUNNING = "running"
    PAUSED = "paused"
    STOPPED = "stopped"
    ERROR = "error"


@dataclass
class SimulationMetrics:
    """Real-time simulation performance metrics."""

    # Time tracking
    current_time: float = 0.0
    total_runtime: float = 0.0
    time_step: float = 1.0  # Default 1 second time steps

    # Coverage metrics
    cells_observed_last_cycle: int = 0
    cells_overdue: int = 0
    coverage_percentage: float = 0.0
    worst_cell_age: float = 0.0

    # Rolling coverage tracking (240s window)
    rolling_coverage_window: List[float] = field(default_factory=list)
    rolling_coverage_avg: float = 0.0
    coverage_window_size: int = 240  # 4 minutes rolling window

    # Fleet metrics
    active_uavs: int = 0
    spare_uavs: int = 0
    uavs_on_mission: int = 0
    uavs_swapping_battery: int = 0
    # Additional fleet metrics
    contingency_spares: int = 0  # spares marked is_contingency and on pad
    failed_uavs: int = 0  # UAVs in FAILED state
    deployed_uavs: int = 0  # convenience: active + swapping
    # Battery metrics
    avg_soc: float = 1.0
    min_soc: float = 1.0

    # STL C3 alarm counts
    c3_alarms: int = 0  # total alarms raised (low-spare or battery-return events)
    c3_missed: int = 0  # alarms not satisfied within 1 s

    # STL contract compliance
    stl_c1_violations: int = 0  # Coverage violations
    stl_c2_violations: int = 0  # Battery violations
    stl_c3_violations: int = 0  # Spare floor violations
    stl_c4_violations: int = 0  # Orphan recovery violations

    # Orphan/bridge telemetry (populated during failure runs)
    orphan_overdue_count: int = 0
    orphan_coverage_pct: float = 0.0
    bridge_inserts_count: int = 0


@dataclass
class GSSSimulation:
    """
    Main GSS simulation engine.

    Integrates all optimization stages and runs the time-stepped simulation
    with real-time UAV tracking, cell coverage monitoring, and STL compliance.
    """

    config: SystemParameters
    state: SimulationState = SimulationState.INITIALIZING
    metrics: SimulationMetrics = field(default_factory=SimulationMetrics)

    # Core simulation components
    cells: List[Cell] = field(default_factory=list)
    uavs: List[UAV] = field(default_factory=list)
    routes: List[Route] = field(default_factory=list)

    # STL contract tracking
    stl_c1_violations: int = 0
    stl_c2_violations: int = 0
    stl_c3_violations: int = 0
    c3_alarms: List[Dict] = field(default_factory=list)  # Track C3 alarms

    # Internal tracking
    _start_time: Optional[float] = None
    _last_update: float = 0.0
    cell_lookup: Dict[str, Cell] = field(default_factory=dict)  # Added for cell lookup
    _failure_manager: Optional[FailureManager] = None
    _failure_markers: Dict[str, Any] = field(default_factory=dict)
    _soc_log_enabled: bool = False
    _soc_log: Dict[str, list] = field(default_factory=dict)
    _quiet_logging: bool = False
    _last_global_no_spare_warning_time: float = 0.0
    _failed_id: Optional[str] = None
    _contingency_id: Optional[str] = None
    _t_cyc: float = (
        0.0  # T_cyc = longest route loop time; used for phase-preserving relaunch
    )

    # ---------------------------------------------------------
    # Constants / tunables (could move to config later)
    # ---------------------------------------------------------
    _MIN_PROGRESS_FRAC = (
        0.10  # parent must finish 10% of its route before a spare can take over
    )

    # ---------------------------------------------------------
    # During __post_init__ create registry for claimed cells
    # ---------------------------------------------------------
    def __post_init__(self):
        # dataclass post-init hook: initialise claimed cell registry
        self._claimed_cells = set()
        # Track when cells were claimed to allow expiration
        self._cell_claim_times = {}  # cell_id -> claim_timestamp
        # CSV logger setup
        self._csv_file = None
        self._csv_writer = None

    # Helper --------------------------------------------------
    def _enough_progress(self, uav: UAV) -> bool:
        """Return True if the UAV has flown enough of its route to justify a spare."""
        if not uav.route_list:
            return False
        route_len = len(uav.route_list[0].cell_sequence)
        if route_len == 0:
            return False
        progress = getattr(uav, "_waypoint_idx", 0)
        min_cells = max(2, int(self._MIN_PROGRESS_FRAC * route_len))
        return progress >= min_cells

    @staticmethod
    def _next_phase_time(phase: float, t_cyc: float, now: float) -> float:
        """Smallest time >= now congruent to `phase` modulo `t_cyc`."""
        if t_cyc <= 0:
            return now
        return now + ((phase - now) % t_cyc)

    # -----------------------------------------------------------------
    # Modify low-battery branch in _update_uavs to respect min progress
    # -----------------------------------------------------------------
    # (below we add an early-return if not enough progress)

    def initialize(self) -> bool:
        """
        Initialize the simulation using the integrated optimization pipeline.

        Returns:
            True if initialization successful, False otherwise.
        """
        try:
            self.state = SimulationState.INITIALIZING

            # Stage 1: Generate surveillance grid
            print("🟢 Stage 1: Generating surveillance grid...")
            self.cells = build_grid_from_config(self.config)
            print(f"   ✅ Generated {len(self.cells)} cells")
            # Build cell lookup for waypoints
            self.cell_lookup = {cell.id: cell for cell in self.cells}

            # Stage 2: Optimize fleet size (initial pass with default β)
            print("🟢 Stage 2: Optimizing fleet size (initial)...")

            # 🆕 ENHANCEMENT: Use enhanced fleet sizing with rotation/contingency distinction
            use_enhanced = getattr(
                self.config.optimization, "use_enhanced_fleet", False
            )
            fleet_result = optimize_fleet_from_config(
                self.config, use_enhanced=use_enhanced
            )

            if use_enhanced and hasattr(fleet_result, "n_rotation"):
                print(
                    f"   ✅ Enhanced Fleet: {fleet_result.n_launch} active + {fleet_result.n_rotation} rotation + {fleet_result.n_contingency} contingency = {fleet_result.n_spare} spare"
                )
            else:
                print(
                    f"   ✅ Fleet: {fleet_result.n_launch} active + {fleet_result.n_spare} spare"
                )

            # Stage 3A: Generate routes
            print("🟢 Stage 3A: Generating routes...")
            algo = self.config.optimization.route_algorithm
            try:
                route_fn = get_route_generator(algo)
            except ValueError:
                print(f"⚠️ Unknown route algorithm '{algo}', falling back to 'greedy'.")
                route_fn = get_route_generator("greedy")

            # Pass furthest_first parameter only to supported algorithms
            if algo == "roundrobin":
                furthest_first = getattr(
                    self.config.optimization, "furthest_first", False
                )
                self.routes, route_summary = route_fn(
                    self.cells,
                    fleet_result.n_launch,
                    self.config.uav.cruise_speed,
                    (self.config.mission.depot_x, self.config.mission.depot_y),
                    furthest_first,
                    buse_s=self.config.battery.usable_endurance,
                )
            else:
                self.routes, route_summary = route_fn(
                    self.cells,
                    fleet_result.n_launch,
                    self.config.uav.cruise_speed,
                )
            print(f"   ✅ Generated {len(self.routes)} routes")

            # Stage 0: Validate per-route battery feasibility (Alg. 5 line 4)
            print("🟢 Stage 0: Validating battery feasibility per route...")
            depot_x, depot_y = self.config.mission.depot_x, self.config.mission.depot_y
            offending_routes = []
            route_margins = []
            for route in self.routes:
                if not route.cell_sequence:
                    continue
                first_cell = self.cell_lookup[route.cell_sequence[0]]
                d_ferry = math.hypot(first_cell.x - depot_x, first_cell.y - depot_y)
                l_grid = 0.0
                prev_cell = first_cell
                for cid in route.cell_sequence[1:]:
                    cell = self.cell_lookup[cid]
                    l_grid += math.hypot(cell.x - prev_cell.x, cell.y - prev_cell.y)
                    prev_cell = cell
                battery_result = optimize_battery_reserve(
                    d_ferry=d_ferry,
                    l_grid=l_grid,
                    v_max=self.config.uav.cruise_speed,
                    endurance=self.config.battery.total_endurance,
                    soc_floor=self.config.battery.soc_floor,
                )
                route_margins.append(battery_result.margin_seconds)
                if not battery_result.is_feasible:
                    offending_routes.append(route.id)
            if offending_routes:
                raise ValueError(
                    f"Stage 0: {len(offending_routes)} route(s) exceed battery "
                    f"endurance (2*d_ferry + l_grid > usable battery range): "
                    f"{offending_routes}"
                )
            if route_margins:
                print(
                    f"   ✅ All routes battery-feasible, min margin: {min(route_margins):.1f}s"
                )

            # Stage 3B: Schedule departures
            print("🟢 Stage 3B: Scheduling departures...")
            schedule_summary = schedule_from_config(self.config, self.routes)
            self._t_cyc = schedule_summary.longest_loop_time
            print(f"   ✅ Max loop time: {schedule_summary.longest_loop_time:.1f}s")
            print(
                f"   📊 n_surge: {schedule_summary.n_surge}, β_adapt: {schedule_summary.β_adapt:.3f}"
            )

            # Stage 2 (adaptive): Re-optimize with n_surge feedback
            print("🟢 Stage 2: Re-optimizing with adaptive β...")
            # Create temporary config with adaptive β
            import copy

            adaptive_config = copy.deepcopy(self.config)
            adaptive_config.optimization.spare_floor_ratio = schedule_summary.β_adapt

            fleet_result_adaptive = optimize_fleet_from_config(
                adaptive_config, use_enhanced=use_enhanced
            )
            if use_enhanced and hasattr(fleet_result_adaptive, "n_rotation"):
                print(
                    f"   ✅ Adaptive Fleet: {fleet_result_adaptive.n_launch} active + {fleet_result_adaptive.n_rotation} rotation + {fleet_result_adaptive.n_contingency} contingency = {fleet_result_adaptive.n_spare} spare"
                )
            else:
                print(
                    f"   ✅ Adaptive Fleet: {fleet_result_adaptive.n_launch} active + {fleet_result_adaptive.n_spare} spare"
                )
            print(
                f"   📈 Spare ratio: {fleet_result.spare_ratio:.3f} → {fleet_result_adaptive.spare_ratio:.3f}"
            )

            # Keep the *route count* as the launch target, moving any
            # difference from adaptive β into the spare count instead. This
            # keeps every generated route assigned to a UAV: too many active
            # slots left routes-less "active" UAVs (misleading HUD); too few
            # left generated routes with no UAV ever assigned to fly them.

            # NOTE: len(self.routes) is fixed by the route generator call earlier.
            launch_target = len(self.routes)
            if fleet_result_adaptive.n_launch != launch_target:
                spare_adjust = fleet_result_adaptive.n_launch - launch_target
                fleet_result_adaptive = fleet_result_adaptive.__class__(
                    n_launch=launch_target,
                    n_spare=max(0, fleet_result_adaptive.n_spare + spare_adjust),
                    n_rotation=getattr(fleet_result_adaptive, "n_rotation", 0),
                    n_contingency=getattr(fleet_result_adaptive, "n_contingency", 0),
                    total_cost=fleet_result_adaptive.total_cost,
                    spare_ratio=fleet_result_adaptive.spare_ratio,
                    is_feasible=fleet_result_adaptive.is_feasible,
                    solver_status=fleet_result_adaptive.solver_status,
                )

            fleet_result = fleet_result_adaptive

            # Create UAV fleet
            print("🟢 Creating UAV fleet...")
            self._create_uav_fleet(
                fleet_result.n_launch,
                fleet_result.n_spare,
                getattr(fleet_result, "n_contingency", 1),
            )
            # Target = number of routes (which equals initial launch size)
            self._n_launch_target = len(self.routes)

            # Assign waypoints to each UAV
            for uav in self.uavs:
                if uav.route_list:
                    route = uav.route_list[0]
                    uav._waypoints = route.get_waypoints(self.cell_lookup)
            # Predefine the failed UAV's route cell ids for orphan tracking from t0
            try:
                if getattr(self.config.failure, "enabled", False):
                    trig = getattr(self.config.failure, "trigger", None)
                    if trig and getattr(trig, "uav_id", None):
                        target_id = str(trig.uav_id)
                        for uav in self.uavs:
                            if uav.id == target_id and uav.route_list:
                                self._failed_route_ids = list(
                                    uav.route_list[0].cell_sequence
                                )
                                break
            except Exception:
                self._failed_route_ids = []

            # Debug: Log initial route assignments
            print("📋 Initial route assignments:")
            for i, uav in enumerate(self.uavs):
                if uav.route_list and i < len(self.routes):
                    route = uav.route_list[0]
                    print(f"   {uav.id}: {route.id} ({len(route.cell_sequence)} cells)")

            print(f"   ✅ Created {len(self.uavs)} UAVs")

            # Initialize metrics
            self.metrics.time_step = 1.0  # 1 second time steps
            self.metrics.active_uavs = 0  # will increase as UAVs launch
            self.metrics.spare_uavs = (
                fleet_result.n_spare + fleet_result.n_launch
            )  # all yet to fly

            # Initialize CSV logging
            self._init_csv_logging()
            print("📊 CSV logging enabled:", self._csv_file)

            # Export route membership for failure/orphan analysis
            try:
                import os, csv as _csv

                os.makedirs("results", exist_ok=True)
                if hasattr(self, "_simulation_info") and self._simulation_info:
                    info = self._simulation_info
                    route_path = f"results/{info['base_name']}_route_membership.csv"
                    with open(route_path, "w", newline="") as f:
                        w = _csv.writer(f)
                        w.writerow(["route_id", "cell_id"])
                        for r in self.routes:
                            for cid in r.cell_sequence:
                                w.writerow([r.id, cid])
                    print(f"🗂️  Route membership exported: {route_path}")
            except Exception as _e:
                print(f"⚠️  Route membership export failed: {_e}")

            # 🔍 ANALYSIS: Route analysis after generation
            # Determine actual algorithm used (check route IDs)
            actual_algorithm = "unknown"
            if self.routes and self.routes[0].id:
                route_id = self.routes[0].id
                if "kmnn" in route_id:
                    actual_algorithm = "KMNN (fallback)"
                elif "alns" in route_id:
                    actual_algorithm = "ALNS"
                elif "roundrobin" in route_id or "rr" in route_id:
                    actual_algorithm = "Round-Robin"
                elif "balanced" in route_id:
                    actual_algorithm = "Balanced"
                else:
                    actual_algorithm = self.config.optimization.route_algorithm

            log_route_analysis(
                self.routes,
                self.cells,
                actual_algorithm,
                self.config.uav.cruise_speed,
                (self.config.mission.depot_x, self.config.mission.depot_y),
            )

            self.state = SimulationState.RUNNING
            print("🚀 GSS Simulation initialized successfully!")
            # Initialize failure manager (no-op when disabled)
            try:
                self._failure_manager = FailureManager(self)
            except Exception as _e:
                # Keep simulation running even if Stage-5 config incomplete
                self._failure_manager = None
            # Enable per-UAV SoC logging only for failure scenarios
            self._soc_log_enabled = bool(getattr(self.config.failure, "enabled", False))
            # Quiet logging automatically when failure scenarios are enabled
            self._quiet_logging = bool(getattr(self.config.failure, "enabled", False))
            return True

        except ValueError:
            # Deliberate feasibility errors (e.g. Stage 0) must propagate to the caller.
            self.state = SimulationState.ERROR
            raise
        except Exception as e:
            print(f"❌ Simulation initialization failed: {e}")
            self.state = SimulationState.ERROR
            return False

    def step(self) -> bool:
        """
        Execute one simulation time step.

        Returns:
            True if step successful, False if simulation should stop.
        """
        if self.state != SimulationState.RUNNING:
            return False

        try:
            # Update simulation time
            self.metrics.current_time += self.metrics.time_step

            # Update UAV positions and states
            self._update_uavs()

            # Stage-5: failure/bridge tick (if enabled)
            if self._failure_manager is not None:
                self._failure_manager.tick()

            # Update cell coverage status
            self._update_cells()

            # Check STL contract compliance
            self._check_stl_compliance()

            # Update performance metrics
            self._update_metrics()

            # Log metrics to CSV
            self._log_csv_metrics()

            # Optional per-UAV SoC logging for failure analysis
            if self._soc_log_enabled:
                # Log SoC every 5s only for contingency + bridging neighbors
                if int(self.metrics.current_time) % 5 == 0:
                    t_now = self.metrics.current_time
                    selected_ids = set()
                    if self._contingency_id:
                        selected_ids.add(self._contingency_id)
                    if self._failed_id:
                        for u in self.uavs:
                            if u.temp_assignments.get(self._failed_id, 0) > 0:
                                selected_ids.add(u.id)
                    for u in self.uavs:
                        if u.id in selected_ids:
                            if u.id not in self._soc_log:
                                self._soc_log[u.id] = []
                            self._soc_log[u.id].append((t_now, u.soc))

            # 🔍 ANALYSIS: Coverage gap snapshots at configurable cadence
            snapshot_period = 600.0
            try:
                if getattr(self.config.failure, "enabled", False):
                    snapshot_period = max(
                        60.0,
                        float(getattr(self.config.stl, "snapshot_period_s", 120.0)),
                    )
                else:
                    snapshot_period = float(
                        getattr(self.config.stl, "snapshot_period_s", 600.0)
                    )
            except Exception:
                snapshot_period = 600.0
            if (
                self.metrics.current_time > 0
                and self.metrics.current_time % snapshot_period == 0
            ):
                print(f"\n⏱️  === Analysis at t={self.metrics.current_time:.0f}s ===")
                # Pass simulation info for consistent file naming
                filename = None
                if hasattr(self, "_simulation_info") and self._simulation_info:
                    info = self._simulation_info
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = (
                        f"results/{info['base_name']}_coverage_gaps_{timestamp}.csv"
                    )

                log_cell_coverage_gaps(
                    self.cells,
                    self.metrics.current_time,
                    threshold=self.config.stl.max_revisit_gap,
                    filename=filename,
                )
                # Pass simulation info for consistent UAV route file naming
                uav_filename = None
                if hasattr(self, "_simulation_info") and self._simulation_info:
                    info = self._simulation_info
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    uav_filename = (
                        f"results/{info['base_name']}_uav_routes_{timestamp}.csv"
                    )

                track_uav_routes(
                    self.uavs, self.metrics.current_time, filename=uav_filename
                )

            return True

        except Exception as e:
            print(f"❌ Simulation step failed: {e}")
            self.state = SimulationState.ERROR
            return False

    def run(self, duration: float = 3600.0) -> bool:
        """
        Run the simulation for the specified duration.

        Args:
            duration: Simulation duration in seconds (default 1 hour).

        Returns:
            True if simulation completed successfully.
        """
        if not self.initialize():
            return False

        self._start_time = time.time()
        target_time = self.metrics.current_time + duration

        print(f"🎯 Running simulation for {duration:.0f} seconds...")

        while (
            self.metrics.current_time < target_time
            and self.state == SimulationState.RUNNING
        ):

            if not self.step():
                break

            # Print progress every 60 simulation seconds
            if int(self.metrics.current_time) % 60 == 0:
                self._print_status()

        self.state = SimulationState.STOPPED
        print("🏁 Simulation completed!")
        return True

    def _create_uav_fleet(
        self, n_launch: int, n_spare: int, n_contingency: int = 1
    ) -> None:
        """Create UAV fleet with proper initialization."""
        depot_x = self.config.mission.depot_x
        depot_y = self.config.mission.depot_y
        # Create active UAVs (initially inactive, launch via schedule)
        for i in range(n_launch):
            route_list = [self.routes[i]] if i < len(self.routes) else []
            departure = self.routes[i].departure_time if i < len(self.routes) else 0.0
            uav = UAV(
                id=f"{i+1:02d}",  # Simple numbering: 01, 02, 03, ... 16
                x=depot_x,
                y=depot_y,  # Start at depot
                soc=1.0,  # Full battery
                route_list=route_list,
                launch_time=departure,
                is_active=False,  # will be activated when launch_time reached
                phase_offset=departure if i < len(self.routes) else None,
            )
            self.uavs.append(uav)

        # Create spare UAVs (rotation + contingency)
        for i in range(n_spare):
            is_contingency_uav = i >= (
                n_spare - n_contingency
            )  # Last n_contingency UAVs are contingency
            uav = UAV(
                id=f"{n_launch+i+1:02d}",  # Continue numbering: 17, 18, 19, 20, 21
                x=depot_x,
                y=depot_y,  # At depot
                soc=1.0,  # Full battery
                route_list=[],
                launch_time=float("inf"),
                is_active=False,  # On standby
                is_contingency=is_contingency_uav,  # Mark contingency UAVs
            )
            self.uavs.append(uav)

            if is_contingency_uav:
                print(
                    f"   🛡️  UAV {uav.id} marked as CONTINGENCY (reserved for failures)"
                )
            else:
                print(f"   🔄 UAV {uav.id} marked as ROTATION spare")

    def _update_uavs(self) -> None:
        """Update UAV positions, handle swapping, and manage spare launches."""
        # NEW – Stage-4 policy decisions (battery return rule, spare checks)
        apply_policy(self.uavs, self.metrics.current_time, self.config)

        speed = self.config.uav.cruise_speed  # UAV speed (m/s)
        dt = self.metrics.time_step
        depot = (self.config.mission.depot_x, self.config.mission.depot_y)
        soc_threshold = self.config.battery.soc_return_threshold

        # ETA-based spare pre-launch (SIMPLIFIED - just triggers earlier returns)
        self._check_eta_prelaunch(speed, depot)

        for uav in self.uavs:
            # --------------------------------------------------------------
            # Handle battery swap countdown
            # --------------------------------------------------------------
            if uav.state == UAVState.FAILED:
                # Failed UAV does not move; keep position fixed and skip logic
                continue
            if uav.state == UAVState.SWAPPING:
                uav.swap_timer -= dt
                if uav.swap_timer <= 0:
                    # Swap complete – ready for relaunch
                    uav.soc = 1.0

                    # Clear any old cell claims from this UAV's previous routes
                    if hasattr(uav, "_previous_route_cells"):
                        for cell_id in uav._previous_route_cells:
                            self._claimed_cells.discard(cell_id)
                            self._cell_claim_times.pop(cell_id, None)
                        delattr(uav, "_previous_route_cells")

                    # IDLE at depot until its own phase slot comes back
                    # around (a spare is already covering this route at the
                    # current phase; relaunching immediately would double-
                    # cover it).
                    uav.state = UAVState.IDLE
                    uav.is_active = False
                    uav._waypoint_idx = 0

                    active_now = sum(
                        1
                        for vv in self.uavs
                        if vv.state in (UAVState.ON_MISSION, UAVState.RTB)
                    )
                    if active_now < self._n_launch_target:
                        if uav.phase_offset is not None:
                            uav.launch_time = self._next_phase_time(
                                uav.phase_offset, self._t_cyc, self.metrics.current_time
                            )
                        else:
                            uav.launch_time = self.metrics.current_time
                    else:
                        uav.launch_time = float("inf")
                # No further movement while swapping
                continue

            # --------------------------------------------------------------
            # RTB: on mission, navigating straight back to depot
            # --------------------------------------------------------------
            if uav.state == UAVState.RTB:
                dist = uav.move_towards(depot[0], depot[1], speed, dt)
                self._consume_battery(uav, dist)
                if uav.at_position(depot[0], depot[1]):
                    # Arrived – start hot-swap
                    uav.state = UAVState.SWAPPING
                    uav.is_active = False
                    uav.swap_timer = self.config.battery.hot_swap_time

                    # Launch spare immediately if available
                    self._launch_spare_for(uav)

                    # Reset fields for when this UAV becomes spare again
                    uav._waypoint_idx = 0
                continue

            # --------------------------------------------------------------
            # Scheduled launch / activation
            # --------------------------------------------------------------
            if (
                uav.state in (UAVState.SPARE, UAVState.IDLE)
                and not uav.is_active
                and self.metrics.current_time >= uav.launch_time
                and uav.launch_time != float("inf")
            ):
                uav.is_active = True
                uav.state = UAVState.ON_MISSION

            # Skip any UAVs that are not currently on mission
            if uav.state != UAVState.ON_MISSION:
                continue

            # --------------------------------------------------------------
            # Normal waypoint following logic
            # --------------------------------------------------------------
            if uav.route_list:
                if not hasattr(uav, "_waypoints"):
                    route = uav.route_list[0]
                    uav._waypoints = route.get_waypoints(self.cell_lookup)
                if not hasattr(uav, "_waypoint_idx"):
                    uav._waypoint_idx = 0
                waypoints = uav._waypoints

                # Completed route lap
                if uav._waypoint_idx >= len(waypoints):
                    if uav.soc > soc_threshold:
                        # Start next lap – simply reset index
                        uav._waypoint_idx = 0
                        # no battery consumption this tick beyond normal
                    else:
                        # Low battery – head home
                        uav._pending_tail = uav._waypoint_idx  # cells after current idx
                        uav.state = UAVState.RTB
                        # Raise C3 alarm and attempt spare launch immediately
                        if not getattr(uav, "_alarm_raised", False):
                            uav._alarm_raised = True
                            uav._alarm_time = self.metrics.current_time
                            self.metrics.c3_alarms += 1
                            # Only launch spare if enough progress made
                            if self._enough_progress(uav):
                                self._launch_spare_for(uav)
                            else:
                                print(
                                    f"⏸️  Spare for {uav.id} deferred (progress {uav._waypoint_idx}/{len(uav.route_list[0].cell_sequence)})"
                                )
                    continue

                # If already on top of current waypoint (happens right after lap reset)
                while uav._waypoint_idx < len(waypoints) and uav.at_position(
                    waypoints[uav._waypoint_idx][0],
                    waypoints[uav._waypoint_idx][1],
                    threshold=2.0,
                ):
                    uav._waypoint_idx += 1

                if uav._waypoint_idx >= len(waypoints):
                    # Handle lap complete in next iteration
                    continue

                target = waypoints[uav._waypoint_idx]
                dist = uav.move_towards(target[0], target[1], speed, dt)
                self._consume_battery(uav, dist)
                if uav.at_position(target[0], target[1]):
                    uav._waypoint_idx += 1

        # After all movement, second-pass check: any UAV dropped below threshold mid-route
        soc_threshold = self.config.battery.soc_return_threshold
        for uav in self.uavs:
            if uav.state == UAVState.ON_MISSION and uav.soc <= soc_threshold:
                # Force return to depot (state==ON_MISSION already excludes RTB)
                uav._pending_tail = getattr(uav, "_waypoint_idx", 0)
                uav.state = UAVState.RTB
                if not getattr(uav, "_alarm_raised", False):
                    uav._alarm_raised = True
                    uav._alarm_time = self.metrics.current_time
                    self.metrics.c3_alarms += 1
                    if self._enough_progress(uav):
                        self._launch_spare_for(uav)
                    else:
                        print(
                            f"⏸️  Spare for {uav.id} deferred (progress {uav._waypoint_idx}/{len(uav.route_list[0].cell_sequence)})"
                        )

        # Evaluate outstanding alarms
        deadline = self.config.stl.spare_launch_deadline
        for uav in self.uavs:
            if getattr(uav, "_alarm_raised", False):
                # considered satisfied if a spare launched within the deadline
                if self.metrics.current_time - uav._alarm_time > deadline:
                    # find any spare launched corresponding? If not already marked missed
                    if not getattr(uav, "_alarm_checked", False):
                        self.metrics.c3_missed += 1
                        uav._alarm_checked = True
                        uav._alarm_raised = False

    # ---------------------------------------------------------------------
    # Battery helper
    # ---------------------------------------------------------------------

    def _usable_range_m(self) -> float:
        """Usable flight range (m) = cruise_speed * usable_endurance."""
        return self.config.uav.cruise_speed * self.config.battery.usable_endurance

    def _consume_battery(self, uav: UAV, distance_m: float) -> None:
        """Reduce UAV SoC based on distance travelled.

        A simple linear model: SoC fraction consumed = distance / usable_range.
        """
        if distance_m == 0:
            return
        usable_range = self._usable_range_m()
        if usable_range <= 0:
            return
        uav.soc = max(0.0, uav.soc - distance_m / usable_range)

    def _update_cells(self) -> None:
        """Update cell coverage status and ages."""
        current_time = self.metrics.current_time
        # Sensor range: UAV can observe cells within this distance
        # Set to cell_size/2 + margin to ensure adjacent cells are covered during diagonal flight
        sensor_range = self.config.grid.cell_size * 0.75  # 30m for 40m cells

        # Expire old cell claims as safety fallback (most claims cleared on observation)
        expired_cells = []
        for cell_id, claim_time in self._cell_claim_times.items():
            if current_time - claim_time > 1200.0:  # Safety fallback: 20 minutes
                expired_cells.append(cell_id)

        for cell_id in expired_cells:
            print(f"⚠️  Cell {cell_id} claim expired after 1200s (safety fallback)")
            del self._cell_claim_times[cell_id]
            self._claimed_cells.discard(cell_id)

        for cell in self.cells:
            # Check if any UAV is observing this cell
            cell.is_covered = False
            for uav in self.uavs:
                if uav.state in (UAVState.ON_MISSION, UAVState.RTB):
                    # Use distance-based coverage instead of exact position
                    import math

                    distance = math.hypot(uav.x - cell.x, uav.y - cell.y)
                    if distance <= sensor_range:
                        cell.update_observation(current_time)
                        cell.is_covered = True
                        # Clear claim when cell is actually observed (instead of fixed timeout)
                        if cell.id in self._claimed_cells:
                            self._claimed_cells.discard(cell.id)
                            self._cell_claim_times.pop(cell.id, None)
                        break  # Only need one UAV to observe

    def _check_stl_compliance(self) -> None:
        """Check STL contract compliance and update violation counts."""
        # C-1: Coverage violations (cells not observed within Θ)
        # C-2: Battery violations (SoC below floor)
        # C-3: Spare floor – ensure spare ratio ≥ β_min
        beta_min = self.config.optimization.spare_floor_ratio
        if (
            self.metrics.spare_uavs
            + self.metrics.uavs_swapping_battery
            + self.metrics.active_uavs
            == 0
        ):
            return  # Avoid division by zero when no UAVs present
        current_spare_ratio = self.metrics.spare_uavs / (len(self.uavs))
        if current_spare_ratio < beta_min:
            self.metrics.stl_c3_violations += 1
        # C-2: Battery floor violations
        soc_floor = self.config.battery.soc_floor
        for uav in self.uavs:
            if uav.soc < soc_floor:
                self.metrics.stl_c2_violations += 1

        # C-4: Orphan recovery violations
        pass

    def _update_metrics(self) -> None:
        """Update real-time performance metrics."""
        if self._start_time:
            self.metrics.total_runtime = time.time() - self._start_time

        # Count UAV states. RTB counts as on-mission (still deployed, still
        # accruing coverage on its way home); IDLE counts with spares (at
        # depot, just not yet available for ad-hoc reassignment).
        self.metrics.active_uavs = sum(
            1 for uav in self.uavs if uav.state in (UAVState.ON_MISSION, UAVState.RTB)
        )
        self.metrics.uavs_on_mission = self.metrics.active_uavs  # alias for now
        # Count only rotation spares in metrics (contingency spares are separate)
        self.metrics.spare_uavs = sum(
            1
            for uav in self.uavs
            if uav.state in (UAVState.SPARE, UAVState.IDLE) and not uav.is_contingency
        )
        self.metrics.uavs_swapping_battery = sum(
            1 for uav in self.uavs if uav.state == UAVState.SWAPPING
        )
        self.metrics.contingency_spares = sum(
            1
            for uav in self.uavs
            if uav.state == UAVState.SPARE and getattr(uav, "is_contingency", False)
        )
        self.metrics.failed_uavs = sum(
            1 for uav in self.uavs if uav.state == UAVState.FAILED
        )
        self.metrics.deployed_uavs = (
            self.metrics.active_uavs + self.metrics.uavs_swapping_battery
        )

        # Battery metrics
        soc_values = [uav.soc for uav in self.uavs]
        if soc_values:
            self.metrics.avg_soc = sum(soc_values) / len(soc_values)
            self.metrics.min_soc = min(soc_values)
        # Coverage metrics
        overdue_cells = [
            cell for cell in self.cells if cell.age(self.metrics.current_time) > 120.0
        ]
        self.metrics.cells_overdue = len(overdue_cells)
        self.metrics.coverage_percentage = (
            (len(self.cells) - len(overdue_cells)) / len(self.cells) * 100
            if self.cells
            else 0
        )

        # Rolling coverage tracking (240s window)
        self.metrics.rolling_coverage_window.append(self.metrics.coverage_percentage)

        # Keep only last 240 samples (240s at 1s time step)
        if (
            len(self.metrics.rolling_coverage_window)
            > self.metrics.coverage_window_size
        ):
            self.metrics.rolling_coverage_window.pop(0)

        # Compute rolling average
        if self.metrics.rolling_coverage_window:
            self.metrics.rolling_coverage_avg = sum(
                self.metrics.rolling_coverage_window
            ) / len(self.metrics.rolling_coverage_window)

        # --- Orphan telemetry (if failure scenario and failed route known)
        try:
            orphan_ids = getattr(self, "_failed_route_ids", None)
            if orphan_ids:
                theta = float(getattr(self.config.stl, "max_revisit_gap", 180.0))
                ages = []
                overdue_flags = []
                for cid in orphan_ids:
                    c = self.cell_lookup.get(cid)
                    if not c:
                        continue
                    a = c.age(self.metrics.current_time)
                    ages.append(a)
                    overdue_flags.append(1 if a > theta else 0)
                if ages:
                    total = float(len(ages))
                    self.metrics.orphan_overdue_count = int(sum(overdue_flags))
                    self.metrics.orphan_coverage_pct = float(
                        (total - self.metrics.orphan_overdue_count) / total * 100.0
                    )
        except Exception:
            pass

    def _print_status(self) -> None:
        """Print current simulation status."""
        print(
            f"⏱️  Time: {self.metrics.current_time:.0f}s | "
            f"Coverage: {self.metrics.coverage_percentage:.1f}% | "
            f"Active UAVs: {self.metrics.uavs_on_mission}/{len(self.uavs)} | "
            f"Overdue cells: {self.metrics.cells_overdue} | "
            f"SoC avg: {self.metrics.avg_soc*100:4.0f}% min: {self.metrics.min_soc*100:3.0f}%"
        )

    def _get_next_spare(self) -> Optional[UAV]:
        """Retrieve the next spare UAV ready for launch.

        Returns:
            A spare UAV object if available, otherwise None.
        """
        for uav in self.uavs:
            if uav.state == UAVState.SPARE and uav.launch_time == float("inf"):
                return uav
        return None

    # -----------------------------------------------------------------
    # Helper: launch spare with remaining segment of route
    # -----------------------------------------------------------------

    def _launch_spare_for(self, origin_uav: UAV):
        """Launch a spare to continue the route of a returning UAV."""
        # Only select rotation spares, exclude contingency spares (reserved for failures)
        available_spares = [
            uav
            for uav in self.uavs
            if uav.state == UAVState.SPARE and not uav.is_contingency
        ]

        if not available_spares:
            # Reduce verbose logging – global throttle to 60s when quiet mode is active
            current_time = self.metrics.current_time
            if (
                not self._quiet_logging
                or current_time - self._last_global_no_spare_warning_time >= 60.0
            ):
                self._last_global_no_spare_warning_time = current_time
                rotation_spares = len(
                    [
                        uav
                        for uav in self.uavs
                        if uav.state == UAVState.SPARE and not uav.is_contingency
                    ]
                )
                contingency_spares = len(
                    [
                        uav
                        for uav in self.uavs
                        if uav.state == UAVState.SPARE and uav.is_contingency
                    ]
                )
                print(
                    f"⚠️  No rotation spares available for {origin_uav.id} ({rotation_spares}/4 rotation, {contingency_spares}/1 contingency)"
                )
            return None

        spare = available_spares[0]

        # Get the base route for this UAV; the spare takes over the SAME
        # Route (not a freshly built partial one) so it continues the
        # parent's loop at the parent's own point, preserving phase.
        base_route = origin_uav.route_list[0]
        total_cells = len(base_route.cell_sequence)
        current_waypoint_idx = getattr(origin_uav, "_waypoint_idx", 0)

        # Keep the >=3-cell tail-extension rule (docs §4.3) by starting
        # earlier within the SAME route rather than building a new one.
        MIN_SPARE_ROUTE_CELLS = 3
        start_idx = min(current_waypoint_idx, total_cells)
        if total_cells - start_idx < MIN_SPARE_ROUTE_CELLS:
            start_idx = max(0, total_cells - MIN_SPARE_ROUTE_CELLS)

        # Claim bookkeeping is keyed on the cells ahead of the spare only.
        ahead_cells = [
            cid
            for cid in base_route.cell_sequence[start_idx:]
            if cid not in self._claimed_cells
        ]

        if not ahead_cells:
            print(f"⚠️  No unclaimed cells left for spare {spare.id}; skipping launch")
            return None

        self._claimed_cells.update(ahead_cells)
        self._cell_claim_times.update(
            {cid: self.metrics.current_time for cid in ahead_cells}
        )

        print(f"🚀 Launching spare {spare.id} for {origin_uav.id}")
        print(
            f"   Origin progress: waypoint {current_waypoint_idx}/{total_cells}; "
            f"spare resumes route {base_route.id} at index {start_idx}"
        )

        spare.route_list = [base_route]
        spare._waypoints = base_route.get_waypoints(self.cell_lookup)
        spare._waypoint_idx = start_idx
        spare.state = UAVState.ON_MISSION
        spare.mission_start_time = self.metrics.current_time
        spare.phase_offset = origin_uav.phase_offset

        # Track cells claimed for this launch, for later claim cleanup.
        spare._previous_route_cells = ahead_cells.copy()

        # Mark C3 alarm as satisfied if we launched within deadline
        for alarm in self.c3_alarms:
            if (
                not alarm["satisfied"]
                and self.metrics.current_time <= alarm["deadline"]
            ):
                alarm["satisfied"] = True
                break

        # Alarm satisfied
        origin_uav._alarm_raised = False
        origin_uav._alarm_checked = True

        return spare

    # ETA pre-launch method - will be reimplemented in Week 1
    def _check_eta_prelaunch(self, speed: float, depot: tuple[float, float]) -> None:
        """
        Enhanced ETA pre-launch: Launch spares when the parent will hit θ_return
        before reaching depot, preventing coverage gaps.
        """
        import math
        from ..stage4_policy import get_distance_aware_threshold

        for uav in self.uavs:
            if uav.state == UAVState.ON_MISSION:
                # Calculate distance to depot
                distance_to_depot = math.hypot(uav.x - depot[0], uav.y - depot[1])

                # Estimate time to reach depot at current speed
                eta_depot = distance_to_depot / speed if speed > 0 else float("inf")

                # Get distance-aware threshold for this UAV
                threshold = get_distance_aware_threshold(
                    uav.x,
                    uav.y,
                    depot[0],
                    depot[1],
                    self.config.battery.soc_return_threshold,
                )

                # Estimate SoC when reaching depot (rough approximation)
                # Assume linear SoC consumption based on distance
                usable_range = self._usable_range_m()
                soc_consumption_per_meter = (
                    1.0 / usable_range if usable_range > 0 else 0.0
                )
                estimated_soc_at_depot = uav.soc - (
                    distance_to_depot * soc_consumption_per_meter
                )

                # Pre-launch if UAV will hit threshold before depot or just after
                if (
                    estimated_soc_at_depot
                    <= threshold + self.config.battery.prelaunch_margin
                    and distance_to_depot > 200.0  # Don't pre-launch if very close
                    and not getattr(uav, "_prelaunch_triggered", False)
                ):

                    # Launch spare immediately
                    if self._enough_progress(uav):
                        launched_spare = self._launch_spare_for(uav)
                        if launched_spare:
                            uav._prelaunch_triggered = True
                            print(
                                f"   🚀 Pre-launched spare {launched_spare.id} for {uav.id}"
                            )
                            print(
                                f"      ETA to depot: {eta_depot:.0f}s, SoC at depot: {estimated_soc_at_depot:.2f}"
                            )
                            return  # Only launch one spare per tick

    def _init_csv_logging(self, filename: str = None) -> None:
        """Initialize CSV logging for simulation metrics."""
        import csv
        import datetime

        if filename is None:
            # Use shared simulation info if available
            if hasattr(self, "_simulation_info") and self._simulation_info:
                info = self._simulation_info
                filename = f"results/{info['base_name']}_metrics.csv"
            else:
                # Fallback to old behavior
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                route_algo = self.config.optimization.route_algorithm
                filename = f"results/sim_{route_algo}_{timestamp}_metrics.csv"

        try:
            self._csv_file = open(filename, "w", newline="")
            self._csv_writer = csv.writer(self._csv_file)

            # Write header (includes new fleet and orphan telemetry fields)
            self._csv_writer.writerow(
                [
                    "time",
                    "active_uavs",
                    "spare_uavs",
                    "swapping_uavs",
                    "contingency_spares",
                    "failed_uavs",
                    "deployed_uavs",
                    "coverage_%",
                    "rolling_avg_%",
                    "cells_overdue",
                    "c2_violations",
                    "c3_alarms",
                    "c3_missed",
                    "avg_soc",
                    "min_soc",
                    "orphan_overdue",
                    "orphan_coverage_%",
                    "bridge_inserts",
                ]
            )
            print(f"📊 CSV logging enabled: {filename}")
        except Exception as e:
            print(f"⚠️  CSV logging failed: {e}")

    def _log_csv_metrics(self) -> None:
        """Log current metrics to CSV file."""
        if self._csv_writer:
            self._csv_writer.writerow(
                [
                    self.metrics.current_time,
                    self.metrics.active_uavs,
                    self.metrics.spare_uavs,
                    self.metrics.uavs_swapping_battery,
                    self.metrics.contingency_spares,
                    self.metrics.failed_uavs,
                    self.metrics.deployed_uavs,
                    round(self.metrics.coverage_percentage, 1),
                    round(self.metrics.rolling_coverage_avg, 1),
                    self.metrics.cells_overdue,
                    self.metrics.stl_c2_violations,
                    self.metrics.c3_alarms,
                    self.metrics.c3_missed,
                    round(self.metrics.avg_soc * 100, 1),
                    round(self.metrics.min_soc * 100, 1),
                    self.metrics.orphan_overdue_count,
                    round(self.metrics.orphan_coverage_pct, 1),
                    self.metrics.bridge_inserts_count,
                ]
            )
            # Flush every 60 seconds to ensure data is written
            if int(self.metrics.current_time) % 60 == 0:
                self._csv_file.flush()
            # Reset per-tick counters
            try:
                self.metrics.bridge_inserts_count = 0
            except Exception:
                pass

        # When failure is enabled and we reached end, emit recovery metrics CSVs
        if (
            getattr(self.config.failure, "enabled", False)
            and self.state != SimulationState.RUNNING
            and hasattr(self, "_simulation_info")
            and self._simulation_info
        ):
            try:
                base = self._simulation_info["base_name"]
                # Recovery metrics: min coverage after failure and time under 90%
                import os
                import csv as _csv

                metrics_path = f"results/{base}_recovery_metrics.csv"
                # Attempt to compute from existing rolling window/history
                # Note: we have only current rolling window in memory; rely on metrics CSV for full series
                # Parse our metrics CSV to compute recovery stats
                src = f"results/{base}_metrics.csv"
                if os.path.exists(src):
                    times, coverages = [], []
                    with open(src, "r") as f:
                        rdr = _csv.reader(f)
                        next(rdr, None)
                        for row in rdr:
                            if len(row) >= 5:
                                try:
                                    times.append(float(row[0]))
                                    coverages.append(float(row[4]))
                                except ValueError:
                                    pass
                    t_fail = (
                        self._failure_markers.get("t_fail", None)
                        if hasattr(self, "_failure_markers")
                        else None
                    )
                    min_cov = None
                    time_under_90 = 0.0
                    rec_time_90 = None
                    if t_fail is not None and times:
                        # Compute from t_fail onward
                        for t, c in zip(times, coverages):
                            if t >= t_fail:
                                min_cov = c if min_cov is None else min(min_cov, c)
                                if c < 90.0:
                                    time_under_90 += 1.0
                                elif rec_time_90 is None:
                                    rec_time_90 = t - t_fail
                        if rec_time_90 is None and times:
                            rec_time_90 = times[-1] - t_fail
                    with open(metrics_path, "w", newline="") as f:
                        w = _csv.writer(f)
                        w.writerow(
                            [
                                "t_fail",
                                "min_coverage_post_fail",
                                "time_under_90_s",
                                "recovery_time_to_90_s",
                            ]
                        )
                        w.writerow(
                            [
                                t_fail if t_fail is not None else "",
                                f"{min_cov:.1f}" if min_cov is not None else "",
                                (
                                    f"{time_under_90:.0f}"
                                    if time_under_90 is not None
                                    else ""
                                ),
                                f"{rec_time_90:.0f}" if rec_time_90 is not None else "",
                            ]
                        )
                # Per-UAV SoC series
                if getattr(self, "_soc_log", None):
                    soc_path = f"results/{base}_soc_timeseries.csv"
                    with open(soc_path, "w", newline="") as f:
                        w = _csv.writer(f)
                        # header: time plus uav ids
                        ids = sorted(self._soc_log.keys())
                        # gather union of times (we logged per-step so consistent)
                        w.writerow(["time"] + ids)
                        # assume equal length
                        length = max(
                            (len(v) for v in self._soc_log.values()), default=0
                        )
                        for i in range(length):
                            t_val = None
                            row = []
                            for uid in ids:
                                if i < len(self._soc_log[uid]):
                                    t, s = self._soc_log[uid][i]
                                    if t_val is None:
                                        t_val = t
                                    row.append(f"{s*100:.1f}")
                                else:
                                    row.append("")
                            if t_val is not None:
                                w.writerow([f"{t_val:.0f}"] + row)
            except Exception as _e:
                pass


# Export main classes
__all__ = ["GSSSimulation", "SimulationState", "SimulationMetrics"]
