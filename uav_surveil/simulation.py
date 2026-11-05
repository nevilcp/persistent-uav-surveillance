"""Ground Station Monitor: the 1 Hz simulation loop."""

from __future__ import annotations

import json
from pathlib import Path

from .coverage import CoverageMap
from .dispatch import must_return, needs_replacement
from .failure import FailureManager
from .fleet import FleetSplit, split_fleet
from .geometry import distance
from .grid import Grid
from .monitor import EventLog, Monitor
from .params import Parameters
from .routing import Route, check_admissible, ferry_time, round_robin_routes
from .schedule import assign_departures
from .uav import UAV, UAVState


class Simulation:
    """One run of the surveillance architecture."""

    def __init__(self, params: Parameters) -> None:
        self.params = params
        self.time = 0.0
        self.events = EventLog()

        self.grid = Grid(params.region)
        self.coverage = CoverageMap(len(self.grid.cells))
        self.split: FleetSplit = split_fleet(params.fleet)
        self.ferry_time = ferry_time(self.grid, params.region.depot, params.uav)
        routes = round_robin_routes(self.grid, self.split.n_fly)
        check_admissible(routes, self.ferry_time, params.uav)
        self.routes: list[Route] = assign_departures(
            routes, params.fleet, params.schedule
        )
        self.monitor = Monitor(params, self.routes)

        self.uavs = self._create_fleet()
        self.failure = (
            FailureManager(
                params, self.uavs, self.coverage, self.ferry_time, self.events
            )
            if params.failure.enabled
            else None
        )

    def _create_fleet(self) -> list[UAV]:
        """All UAVs start at the depot: flyers, then rotation, then spare."""
        depot, split = self.params.region.depot, self.split
        uavs = []
        for i, route in enumerate(self.routes):
            uavs.append(
                UAV(id=i + 1, position=depot, route=route, launch_at=route.departure)
            )
        for i in range(split.n_rotate):
            uavs.append(UAV(id=split.n_fly + i + 1, position=depot))
        for i in range(split.n_spare):
            uavs.append(
                UAV(
                    id=split.n_fly + split.n_rotate + i + 1,
                    position=depot,
                    is_contingency=True,
                )
            )
        return uavs

    def run(self) -> dict:
        """Simulate the whole mission and return the summary."""
        steps = round(self.params.mission.duration / self.params.mission.time_step)
        for _ in range(steps):
            self.step()
        return self.monitor.summary(self.failure)

    def step(self) -> None:
        """Advance the Ground Station Monitor by one time step."""
        dt = self.params.mission.time_step
        if self.failure is not None and self.failure.due(self.time):
            route = self.failure.inject(self.time)
            self.monitor.raise_alarm(self.time, route.id, "failure")
        self._launch_scheduled()
        self._dispatch()
        self._serve_alarms()
        if self.failure is not None:
            self.failure.bridge(self.time)
            self.failure.check_handover(self.time)
        self._move(dt)
        self.time += dt
        self._observe()
        self.monitor.record(self.time, self.uavs, self.coverage)

    def _launch_scheduled(self) -> None:
        for uav in self.uavs:
            due = uav.launch_at is not None and self.time >= uav.launch_at
            if uav.state is UAVState.READY and due:
                uav.launch(uav.route, self.time)
                self.events.add(
                    self.time, "launch", uav.id, f"route {uav.route.id} (scheduled)"
                )

    def _dispatch(self) -> None:
        """Apply the replacement and return-to-base rules."""
        cfg, depot = self.params, self.params.region.depot
        for uav in self.uavs:
            if uav.state is not UAVState.ON_ROUTE:
                continue
            d = distance(uav.position, depot)
            if not uav.replaced and needs_replacement(
                uav.battery, d, cfg.uav, cfg.dispatch
            ):
                uav.replaced = True
                self.monitor.raise_alarm(self.time, uav.route.id, "rotation")
            if must_return(uav.battery, d, cfg.uav, cfg.dispatch):
                uav.state = UAVState.RETURNING
                self.events.add(
                    self.time, "return", uav.id, f"battery {uav.battery:.3f}"
                )

    def _serve_alarms(self) -> None:
        """Launch a reserve UAV for each open alarm, oldest first."""
        for alarm in self.monitor.open_alarms():
            reserve = self._take_reserve(contingency=alarm.kind == "failure")
            if reserve is None:
                continue  # alarm stays open and its latency keeps growing
            reserve.launch(self.routes[alarm.route - 1], self.time)
            alarm.launch_time = self.time
            self.events.add(
                self.time, "launch", reserve.id, f"route {alarm.route} ({alarm.kind})"
            )
            if alarm.kind == "failure":
                self.failure.contingency = reserve

    def _take_reserve(self, contingency: bool) -> UAV | None:
        """Lowest-numbered waiting spare or rotation UAV."""
        for uav in self.uavs:
            waiting = uav.state is UAVState.READY and uav.route is None
            if waiting and uav.is_contingency == contingency:
                return uav
        return None

    def _move(self, dt: float) -> None:
        params = self.params
        depot, speed = params.region.depot, params.uav.speed
        for uav in self.uavs:
            if uav.state is UAVState.SWAPPING:
                uav.swap_remaining -= dt
                if uav.swap_remaining <= 0.0:
                    uav.battery = 1.0
                    uav.state = UAVState.READY
                    uav.clear_route()
            elif uav.state is UAVState.ON_ROUTE:
                waypoint = uav.waypoints[uav.target]
                flown, arrived = uav.fly_towards(waypoint.position, speed * dt)
                uav.battery -= flown / params.uav.range
                if arrived:
                    uav.target = (uav.target + 1) % len(uav.waypoints)
                    uav.waypoints_reached += 1
            elif uav.state is UAVState.RETURNING:
                flown, arrived = uav.fly_towards(depot, speed * dt)
                uav.battery -= flown / params.uav.range
                if arrived:
                    uav.state = UAVState.SWAPPING
                    uav.swap_remaining = params.uav.turnaround
                    self.events.add(
                        self.time + dt, "arrive", uav.id, f"battery {uav.battery:.3f}"
                    )

    def _observe(self) -> None:
        # any airborne UAV refreshes the cells in its footprint, including while ferrying
        radius = self.params.uav.sensing_radius
        for uav in self.uavs:
            if not uav.airborne:
                continue
            for cell in self.grid.cells_within(uav.position, radius):
                self.coverage.observe(cell.index, self.time)
                if self.failure is not None:
                    self.failure.on_observation(cell.index, self.time)

    def save(self, out_dir: str | Path, summary: dict) -> None:
        """Write metrics, events, routes and summary to ``out_dir``."""
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        self.monitor.write_csv(out / "metrics.csv")
        self.events.write_csv(out / "events.csv")
        with open(out / "routes.csv", "w") as handle:
            handle.write("route,order,cell,row,col,departure_s\n")
            for route in self.routes:
                for k, c in enumerate(route.cells):
                    handle.write(
                        f"{route.id},{k},{c.index},{c.row},{c.col},{route.departure}\n"
                    )
        with open(out / "summary.json", "w") as handle:
            json.dump(summary, handle, indent=2)
