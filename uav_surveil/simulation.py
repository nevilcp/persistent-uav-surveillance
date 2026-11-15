"""Ground Station Monitor: the 1 Hz simulation loop."""

from __future__ import annotations

import json
from pathlib import Path

from .airspace import Airspace
from .coverage import CoverageMap
from .depots import Depot, nearest_open_base, reserve_pool
from .dispatch import must_return, needs_replacement
from .failure import FailureManager
from .fleet import FleetSplit, apportion, split_fleet
from .grid import Grid
from .monitor import EventLog, Monitor
from .params import Parameters, Point
from .routing import (
    Route,
    assign_bases,
    base_ferry_times,
    check_admissible,
    round_robin_routes,
)
from .schedule import assign_departures
from .uav import UAV, UAVState


class Simulation:
    """One run of the surveillance architecture."""

    def __init__(self, params: Parameters) -> None:
        self.params = params
        self.time = 0.0
        self.events = EventLog()

        self.grid = Grid(params.region)
        self.airspace = Airspace.from_region(self.grid, params.region)
        self.depots = [
            Depot(m, position, params.uav.depot_slots)
            for m, position in enumerate(self.airspace.depots)
        ]
        self.coverage = CoverageMap.of(self.airspace)
        self.split: FleetSplit = split_fleet(params.fleet)
        self.ferry_times = base_ferry_times(self.grid, self.airspace, params.uav)
        routes = assign_bases(
            round_robin_routes(self.grid, self.split.n_fly, self.airspace.free_cells),
            self.airspace,
        )
        zoned = self.airspace if params.region.zones else None
        for route in routes:
            check_admissible([route], self.ferry_times[route.depot], params.uav, zoned)
        self.routes: list[Route] = assign_departures(
            routes, params.fleet, params.schedule
        )
        self.monitor = Monitor(params, self.routes, self.airspace, self.depots)

        self.uavs = self._create_fleet()
        self.failure = (
            FailureManager(
                params,
                self.uavs,
                self.coverage,
                self.ferry_times,
                self.events,
                zoned,
            )
            if params.failure.enabled
            else None
        )

    @property
    def ferry_time(self) -> float:
        """Longest one-way ferry time over the bases."""
        return max(self.ferry_times)

    def _create_fleet(self) -> list[UAV]:
        """Flyers at their route's base; reserves shared out by route count."""
        depots, split = self.airspace.depots, self.split
        uavs = [
            UAV(
                id=i + 1,
                position=depots[route.depot],
                home=route.depot,
                route=route,
                launch_at=route.departure,
            )
            for i, route in enumerate(self.routes)
        ]
        per_base = [sum(r.depot == m for r in self.routes) for m in range(len(depots))]
        for count, contingency in ((split.n_rotate, False), (split.n_spare, True)):
            for base, n in enumerate(apportion(count, per_base)):
                for _ in range(n):
                    uavs.append(
                        UAV(
                            id=len(uavs) + 1,
                            position=depots[base],
                            home=base,
                            is_contingency=contingency,
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
        cfg = self.params
        for uav in self.uavs:
            if uav.state is not UAVState.ON_ROUTE:
                continue
            base, d = self._nearest_base(uav.position)
            if not uav.replaced and needs_replacement(
                uav.battery, d, cfg.uav, cfg.dispatch
            ):
                uav.replaced = True
                self.monitor.raise_alarm(self.time, uav.route.id, "rotation")
            if must_return(uav.battery, d, cfg.uav, cfg.dispatch):
                uav.state = UAVState.RETURNING
                uav.home = base
                uav.set_leg(
                    self.airspace.depots[base],
                    self.airspace.depot_path(uav.position, base)[1:],
                )
                self.events.add(
                    self.time, "return", uav.id, f"battery {uav.battery:.3f}"
                )

    def _nearest_base(self, position: Point) -> tuple[int, float]:
        """Closest base and the distance the dispatch rules use to it."""
        return nearest_open_base(
            self.airspace, self.depots, position, self.params.dispatch.metric
        )

    def _serve_alarms(self) -> None:
        """Launch a reserve UAV for each open alarm, oldest first."""
        for alarm in self.monitor.open_alarms():
            base = self.routes[alarm.route - 1].depot
            reserve = self._take_reserve(alarm.kind == "failure", base)
            if reserve is None:
                continue  # alarm stays open and its latency keeps growing
            reserve.launch(self.routes[alarm.route - 1], self.time)
            alarm.launch_time = self.time
            self.events.add(
                self.time, "launch", reserve.id, f"route {alarm.route} ({alarm.kind})"
            )
            if alarm.kind == "failure":
                self.failure.contingency = reserve

    def _take_reserve(self, contingency: bool, base: int) -> UAV | None:
        """Parked rotation UAV of ``base``, or a spare from the nearest base with one."""
        order = [base]
        if contingency:
            others = [d.index for d in self.depots if d.index != base]
            depots = self.airspace.depots
            order += sorted(
                others, key=lambda m: self.airspace.distance(depots[base], depots[m])
            )
        for index in order:
            pool = reserve_pool(self.uavs, index, contingency)
            if pool:
                return pool[0]
        return None

    def _move(self, dt: float) -> None:
        params = self.params
        speed = params.uav.speed
        for depot in self.depots:
            depot.advance(dt, params.uav.turnaround)
        for uav in self.uavs:
            if uav.state is UAVState.ON_ROUTE:
                goal = uav.waypoints[uav.target].position
                if uav.leg_goal != goal:
                    uav.set_leg(goal, self.airspace.path(uav.position, goal)[1:])
                flown, arrived = uav.fly_leg(speed * dt)
                uav.battery -= flown / params.uav.range
                if arrived:
                    uav.set_leg(None, [])
                    uav.target = (uav.target + 1) % len(uav.waypoints)
                    uav.waypoints_reached += 1
            elif uav.state is UAVState.RETURNING:
                flown, arrived = uav.fly_leg(speed * dt)
                uav.battery -= flown / params.uav.range
                if arrived:
                    self.depots[uav.home].arrive(uav, params.uav.turnaround)
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
