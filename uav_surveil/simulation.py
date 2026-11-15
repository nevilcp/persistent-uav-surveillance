"""Ground Station Monitor: the 1 Hz simulation loop."""

from __future__ import annotations

import json
from dataclasses import asdict, replace
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
    base_ferry_times,
    check_admissible,
    depot_round_robin_routes,
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
        routes = depot_round_robin_routes(self.grid, self.split.n_fly, self.airspace)
        zoned = self.airspace if params.region.zones else None
        for route in routes:
            check_admissible([route], self.ferry_times[route.depot], params.uav, zoned)
        self.routes: list[Route] = []
        # Independent departure schedules at each depot; preserve the single-base schedule.
        local_fleet = replace(params.fleet, batch_size=max(1, params.fleet.batch_size // len(self.depots)))
        for depot in self.depots:
            self.routes.extend(assign_departures(
                [r for r in routes if r.depot == depot.index], local_fleet, params.schedule
            ))
        self.monitor = Monitor(params, self.routes, self.airspace, self.depots)

        self.uavs = self._create_fleet()
        self.failures: list[FailureManager] = []
        self.breakdowns: list[tuple[float, int]] = []
        self._failure_times = sorted((params.failure.time, *params.failure.later_times)) if params.failure.enabled else []
        self._next_failure = 0
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
        result = self.monitor.summary(self.failure, self.breakdowns)
        result["planner"] = "choset_round_robin"
        result["fleet"] = {"flying": self.split.n_fly, "rotation": self.split.n_rotate, "spare": self.split.n_spare}
        return result

    def step(self) -> None:
        """Advance the Ground Station Monitor by one time step."""
        dt = self.params.mission.time_step
        self._close_depot()
        self._inject_failures()
        self._launch_scheduled()
        self._dispatch()
        self._serve_alarms()
        for failure in self.failures:
            failure.bridge(self.time)
            failure.check_handover(self.time)
        self._move(dt)
        self.time += dt
        self._observe()
        self.monitor.record(self.time, self.uavs, self.coverage)

    def _inject_failures(self) -> None:
        while self._next_failure < len(self._failure_times) and self.time >= self._failure_times[self._next_failure]:
            cfg = self.params.failure
            params = self.params
            if cfg.near_depot is not None:
                candidates = [u for u in self.uavs if u.state is UAVState.ON_ROUTE and u.route.depot == cfg.near_depot]
                if not candidates:
                    raise ValueError(f"no flying UAV served by depot {cfg.near_depot} at {self.time}")
                victim = min(candidates, key=lambda u: (u.replaced, self.airspace.depot_distance(u.position, cfg.near_depot), u.id))
                params = replace(params, failure=replace(cfg, route=victim.route.id))
            manager = self.failure if not self.failures and cfg.near_depot is None else FailureManager(
                params, self.uavs, self.coverage, self.ferry_times, self.events,
                self.airspace if self.params.region.zones else None,
            )
            route = manager.inject(self.time)
            self.failures.append(manager)
            self.failure = manager
            self.breakdowns.append((self.time, manager.failed_uav.id))
            self.monitor.raise_alarm(self.time, route.id, "failure")
            self._next_failure += 1

    def _close_depot(self) -> None:
        cfg = self.params.failure
        if cfg.closed_depot is None or self.time < cfg.closure_time:
            return
        depot = self.depots[cfg.closed_depot]
        if not depot.is_open:
            return
        depot.close(self.time)
        self.events.add(self.time, "closure", "-", f"base {depot.index}")
        open_bases = [d.index for d in self.depots if d.is_open]
        changed = {}
        for i, route in enumerate(self.routes):
            if route.depot == depot.index:
                base = min(open_bases, key=lambda m: sum(self.airspace.depot_distance(c.position, m) for c in route.cells))
                changed[route.id] = replace(route, depot=base)
                self.routes[i] = changed[route.id]
        for uav in self.uavs:
            if uav.route and uav.route.id in changed:
                uav.route = changed[uav.route.id]
                if uav.state is UAVState.READY and uav.home == depot.index:
                    self.monitor.raise_alarm(self.time, uav.route.id, "rotation")
                    uav.launch_at = None
                    uav.clear_route()
            if uav.state is UAVState.RETURNING and uav.home == depot.index:
                uav.home, _ = self._nearest_base(uav.position)
                uav.set_leg(self.airspace.depots[uav.home], self.airspace.depot_path(uav.position, uav.home)[1:])
        self.events.add(self.time, "reassign", "-", f"{len(changed)} routes moved from base {depot.index}")

    def _launch_scheduled(self) -> None:
        for uav in self.uavs:
            due = uav.launch_at is not None and self.time >= uav.launch_at
            if uav.state is UAVState.READY and due and self.depots[uav.home].is_open:
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
                manager = next((f for f in self.failures if f.route.id == alarm.route and f.contingency is None), None)
                if manager is not None:
                    manager.contingency = reserve

    def _take_reserve(self, contingency: bool, base: int) -> UAV | None:
        """Parked rotation UAV of ``base``, or a spare from the nearest base with one."""
        order = [base]
        if contingency or not self.depots[base].is_open:
            others = [d.index for d in self.depots if d.index != base]
            depots = self.airspace.depots
            order += sorted(
                others, key=lambda m: self.airspace.distance(depots[base], depots[m])
            )
        for index in order:
            if not self.depots[index].is_open:
                continue
            pool = reserve_pool(self.uavs, index, contingency)
            if pool:
                return pool[0]
        return None

    def _move(self, dt: float) -> None:
        params = self.params
        speed = params.uav.speed
        for depot in self.depots:
            if not depot.is_open:
                continue
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
                for failure in self.failures:
                    failure.on_observation(cell.index, self.time)

    def save(self, out_dir: str | Path, summary: dict) -> None:
        """Write metrics, events, routes, parameters and summary to ``out_dir``."""
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        self.monitor.write_csv(out / "metrics.csv")
        self.events.write_csv(out / "events.csv")
        with open(out / "routes.csv", "w") as handle:
            handle.write("route,order,cell,row,col,departure_s\n")
            for route in self.routes:
                handle.writelines(
                    f"{route.id},{k},{c.index},{c.row},{c.col},{route.departure}\n"
                    for k, c in enumerate(route.cells)
                )
        with open(out / "summary.json", "w") as handle:
            json.dump(summary, handle, indent=2)
        with open(out / "parameters.json", "w") as handle:
            json.dump(asdict(self.params), handle, indent=2)
