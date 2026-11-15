"""Shared-loop patrol: evenly spaced UAVs, relay hand-overs and re-spacing."""

from __future__ import annotations

import json
from bisect import bisect_left
from collections.abc import Sequence
from pathlib import Path

from .airspace import Airspace
from .coverage import CoverageMap
from .depots import Depot, nearest_open_base, reserve_pool
from .dispatch import must_return, needs_replacement
from .fleet import FleetSplit, size_loop_fleet
from .grid import Grid
from .monitor import Alarm, EventLog, Monitor
from .params import Parameters
from .routing import shared_loop
from .uav import UAV, UAVState

_TOLERANCE = 1e-6  # numerical slack when testing whether a UAV is behind its target (m)


class LoopSimulation:
    """One run of the shared-loop architecture."""

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
        self.loop = shared_loop(self.grid, self.airspace)
        self.sizing = size_loop_fleet(self.loop, self.airspace, params)
        self.split = FleetSplit(
            self.sizing.n_fly, self.sizing.n_rotate, self.sizing.n_spare
        )
        self.monitor = Monitor(params, [], self.airspace, self.depots)
        self.failure = None  # the loop has no route-based failure manager
        self.breakdowns: list[tuple[float, int]] = []  # (time, UAV id)

        # d_G from every base to every loop vertex, so intercept scans stay cheap
        self._dist = [
            [self.airspace.depot_distance(v, m) for v in self.loop.vertices]
            for m in range(len(self.depots))
        ]
        self._near = self._nearest_table()
        self._pending = (
            sorted([params.failure.time, *params.failure.later_times])
            if params.failure.enabled
            else []
        )
        self._closed = params.failure.closed_depot is None
        self._speed: dict[int, float] = {}
        self._moved: dict[int, float] = {}

        self.uavs: list[UAV] = []
        self._by_id: dict[int, UAV] = {}
        self._deploy()

    def run(self) -> dict:
        """Simulate the whole mission and return the summary."""
        steps = round(self.params.mission.duration / self.params.mission.time_step)
        for _ in range(steps):
            self.step()
        result = self.monitor.summary(None, self.breakdowns)
        result["loop"] = {
            "length_m": self.loop.length,
            "n_fly": self.split.n_fly,
            "n_rotate": self.split.n_rotate,
            "n_spare": self.split.n_spare,
            "breakdowns": [{"time_s": t, "uav": u} for t, u in self.breakdowns],
        }
        return result

    def step(self) -> None:
        """Advance the Ground Station Monitor by one time step."""
        dt = self.params.mission.time_step
        self._inject()
        self._dispatch()
        self._serve_alarms()
        self._respace()
        self._move(dt)
        self.time += dt
        self._hand_over(dt)
        self._observe()
        self.monitor.record(self.time, self.uavs, self.coverage)

    def _deploy(self) -> None:
        """Create the fleet and launch the flyers as reliefs of evenly spaced slots."""
        uav, patrol = self.params.uav, self.params.patrol
        flyers: dict[int, list[UAV]] = {}
        for base in self.sizing.bases:
            flyers[base.index] = [
                self._new_uav(base.index, False) for _ in range(base.n_fly)
            ]
        for base in self.sizing.bases:
            for _ in range(base.n_rotate):
                self._new_uav(base.index, False)
            if base.spare:
                self._new_uav(base.index, True)

        size = {b.index: b for b in self.sizing.bases}
        launched = dict.fromkeys(flyers, 0)
        n = self.sizing.n_fly
        for slot in range(n):
            arc = slot * self.loop.length / n
            point = self.loop.point_at(arc)
            open_bases = [m for m in flyers if len(flyers[m]) > launched[m]]
            m = min(open_bases, key=lambda m: self.airspace.depot_distance(point, m))
            flyer = flyers[m][launched[m]]
            launched[m] += 1
            # the i-th of n_m flyers is relieved after i/n_m of a full sortie, so the
            # first relief requests do not arrive together
            window = uav.usable_endurance - 2.0 * size[m].mean_transit
            flyer.cut_after = (
                launched[m] / size[m].n_fly * (window - patrol.handover_overlap)
            )
            _, vertex, ahead = self._intercept(arc, [m], patrol.handover_overlap, None)
            self._send(flyer, vertex, ahead, None, "initial slot")

    def _new_uav(self, base: int, spare: bool) -> UAV:
        uav = UAV(
            id=len(self.uavs) + 1,
            position=self.airspace.depots[base],
            home=base,
            is_contingency=spare,
        )
        self.uavs.append(uav)
        self._by_id[uav.id] = uav
        return uav

    def _nearest_table(self) -> list[float]:
        """Distance from every loop vertex to the nearest open base."""
        open_bases = [d.index for d in self.depots if d.is_open]
        return [
            min(self._dist[m][j] for m in open_bases)
            for j in range(len(self.loop.vertices))
        ]

    def _looping(self) -> list[UAV]:
        return [u for u in self.uavs if u.state is UAVState.ON_ROUTE]

    def _nearest_base(self, uav: UAV) -> tuple[int, float]:
        return nearest_open_base(
            self.airspace, self.depots, uav.position, self.params.dispatch.metric
        )

    def _inject(self) -> None:
        """Breakdowns at their times, and the closure of a base."""
        cfg = self.params.failure
        while self._pending and self.time >= self._pending[0]:
            self._pending.pop(0)
            victim = self._pick_victim()
            if victim is None:
                continue
            victim.state = UAVState.FAILED
            self.breakdowns.append((self.time, victim.id))
            self.events.add(self.time, "failure", victim.id, f"arc {victim.arc:.0f}")
            # its own relief request is moot; the failure alarm replaces the UAV
            self.monitor.alarms = [
                a
                for a in self.monitor.alarms
                if not (
                    a.kind == "rotation"
                    and a.route == victim.id
                    and a.launch_time is None
                )
            ]
            self.monitor.raise_alarm(self.time, victim.id, "failure")
        if not self._closed and self.time >= cfg.closure_time:
            self._close(cfg.closed_depot)

    def _pick_victim(self) -> UAV | None:
        flying = sorted(self._looping(), key=lambda u: u.arc)
        if not flying:
            return None
        base = self.params.failure.near_depot
        if base is None:
            return flying[(self.params.failure.route - 1) % len(flying)]
        return min(flying, key=lambda u: self.airspace.depot_distance(u.position, base))

    def _close(self, base: int) -> None:
        """Close a base: its stretch passes to the others and returns are re-routed."""
        self._closed = True
        self.depots[base].close(self.time)
        self.events.add(self.time, "closure", "-", f"base {base}")
        self._near = self._nearest_table()
        open_bases = [d.index for d in self.depots if d.is_open]
        try:
            fleet = size_loop_fleet(self.loop, self.airspace, self.params, open_bases)
        except ValueError as error:
            self.events.add(self.time, "resize", "-", f"infeasible: {error}")
        else:
            for b in fleet.bases:
                held = sum(
                    1
                    for u in self.uavs
                    if u.home == b.index and not u.is_contingency and u.route is None
                )
                self.events.add(
                    self.time,
                    "resize",
                    "-",
                    f"base {b.index}: n_fly {b.n_fly}, n_rot {b.n_rotate} (holds {held}), "
                    f"rho {b.utilisation:.2f}",
                )
        for uav in self.uavs:
            if uav.state is UAVState.RETURNING and uav.home == base:
                self._send_home(uav, "base closed")

    def _dispatch(self) -> None:
        """Relief requests and the return-to-base rule for the looping UAVs."""
        cfg = self.params
        for uav in self._looping():
            _, d = self._nearest_base(uav)
            staggered = (
                uav.cut_after is not None
                and self.time - uav.launched_at >= uav.cut_after
            )
            if not uav.replaced and (
                staggered or needs_replacement(uav.battery, d, cfg.uav, cfg.dispatch)
            ):
                uav.replaced = True
                self.monitor.raise_alarm(self.time, uav.id, "rotation")
            if must_return(uav.battery, d, cfg.uav, cfg.dispatch):
                self.events.add(
                    self.time, "return", uav.id, f"battery {uav.battery:.3f}"
                )
                self._send_home(uav, "")

    def _send_home(self, uav: UAV, reason: str) -> None:
        base, _ = self._nearest_base(uav)
        if uav.state is UAVState.ON_ROUTE:
            uav.left_at = self.time
        uav.state = UAVState.RETURNING
        uav.home = base
        uav.set_leg(
            self.airspace.depots[base], self.airspace.depot_path(uav.position, base)[1:]
        )
        if reason:
            self.events.add(self.time, "return", uav.id, reason)

    def _serve_alarms(self) -> None:
        """Launch a relief or spare for each open alarm, oldest first."""
        for alarm in self.monitor.open_alarms():
            if alarm.kind == "failure":
                self._fill_gap(alarm)
            else:
                self._relieve(alarm)

    def _relieve(self, alarm: Alarm) -> None:
        incumbent = self._by_id[alarm.route]
        uav = self.params.uav
        if incumbent.state is UAVState.ON_ROUTE:
            arc, battery, relieved = incumbent.arc, incumbent.battery, incumbent
        else:
            # gone before a relief was free: its slot keeps moving at v
            gone = uav.speed * (self.time - incumbent.left_at)
            arc, battery, relieved = (
                (incumbent.arc + gone) % self.loop.length,
                None,
                None,
            )
        bases = [d.index for d in self.depots if d.is_open]
        base, vertex, ahead = self._intercept(
            arc, bases, self.params.patrol.handover_overlap, battery
        )
        pool = reserve_pool(self.uavs, base, False)
        if not pool:
            return  # alarm stays open and its latency keeps growing
        self._send(pool[0], vertex, ahead, relieved, f"relief of {incumbent.id}")
        alarm.launch_time = self.time

    def _fill_gap(self, alarm: Alarm) -> None:
        pools = {m: reserve_pool(self.uavs, m, True) for m in range(len(self.depots))}
        bases = [m for m, pool in pools.items() if pool]
        if not bases:
            return
        arcs = sorted(self._slot_arcs())
        if arcs:
            gaps = [
                ((b - a) % self.loop.length or self.loop.length, a)
                for a, b in zip(arcs, arcs[1:] + arcs[:1])
            ]
            width, start = max(gaps)
            middle = (start + width / 2.0) % self.loop.length
        else:
            middle = 0.0
        base, vertex, ahead = self._intercept(middle, bases, 0.0, None)
        self._send(pools[base][0], vertex, ahead, None, f"spare for {alarm.route}")
        alarm.launch_time = self.time

    def _slot_arcs(self) -> list[float]:
        """Loop positions of the occupied slots, including reliefs yet to arrive."""
        arcs = [u.arc for u in self._looping()]
        for u in self.uavs:
            if u.state is not UAVState.RELAYING:
                continue
            if u.relieving is None:
                arcs.append((u.meet_arc - u.meet_ahead) % self.loop.length)
            elif u.relieving.state is not UAVState.ON_ROUTE:
                arcs.append(u.meet_arc)
        return arcs

    def _intercept(
        self,
        arc: float,
        bases: Sequence[int],
        overlap: float,
        battery: float | None,
    ) -> tuple[int, int, float]:
        """Base, loop vertex and arc distance of the first hand-over a relief makes in time."""
        loop, uav, dispatch = self.loop, self.params.uav, self.params.dispatch
        slack = uav.speed * overlap
        n = len(loop.vertices)
        start = bisect_left(loop.arcs, arc) % n
        best: tuple[float, int, int, float] | None = None
        for k in range(n):
            j = (start + k) % n
            ahead = (loop.arcs[j] - arc) % loop.length
            if battery is not None:
                left = battery - ahead / uav.range
                if left <= uav.sigma_min + dispatch.return_buffer:
                    break
                if (
                    left
                    <= uav.sigma_min
                    + dispatch.return_buffer
                    + self._near[j] / uav.range
                ):
                    continue
            base = min(bases, key=lambda m: (self._dist[m][j], m))
            late = self._dist[base][j] + slack - ahead
            if late <= 0.0:
                return base, j, ahead
            if best is None or late < best[0]:
                best = (late, base, j, ahead)
        if best is None:
            # the incumbent has no spare battery: meet it where it is
            ahead = (loop.arcs[start] - arc) % loop.length
            return min(bases, key=lambda m: (self._dist[m][start], m)), start, ahead
        return best[1], best[2], best[3]

    def _send(
        self, uav: UAV, vertex: int, ahead: float, incumbent: UAV | None, detail: str
    ) -> None:
        """Launch ``uav`` towards a loop vertex where it takes over a slot."""
        meeting = self.loop.vertices[vertex]
        uav.state = UAVState.RELAYING
        uav.relieving = incumbent
        uav.meet_arc = self.loop.arcs[vertex]
        uav.meet_ahead = ahead
        uav.is_contingency = False
        uav.set_leg(meeting, self.airspace.path(uav.position, meeting)[1:])
        self.events.add(self.time, "launch", uav.id, detail)

    def _respace(self) -> None:
        """Speeds that bring the looping UAVs to even spacing by slowing the leaders."""
        patrol, v = self.params.patrol, self.params.uav.speed
        flying = sorted(self._looping(), key=lambda u: u.arc)
        self._speed = {u.id: v for u in flying}
        n = len(flying)
        # a slot about to be filled would look like a hole, so the leaders are not slowed
        filling = any(
            u.state is UAVState.RELAYING
            and (u.relieving is None or u.relieving.state is not UAVState.ON_ROUTE)
            for u in self.uavs
        )
        if n < 2 or filling:
            return
        length = self.loop.length
        gap = length / n
        arcs = [u.arc for u in flying]
        # UAVs can only slow down, so the reference must leave nobody behind its target
        best: tuple[float, float, list[float]] | None = None
        for r in range(n):
            lead = [
                ((arcs[i] - arcs[r] - ((i - r) % n) * gap + length / 2.0) % length)
                - length / 2.0
                for i in range(n)
            ]
            key = (max(-min(lead) - _TOLERANCE, 0.0), max(lead))
            if best is None or key < best[:2]:
                best = (*key, lead)
        for u, e in zip(flying, best[2]):
            slow = min(
                max(e, 0.0) / patrol.respace_gain_time,
                (1.0 - patrol.min_speed_ratio) * v,
            )
            self._speed[u.id] = v - slow

    def _move(self, dt: float) -> None:
        params = self.params
        speed = params.uav.speed
        burn = speed * dt / params.uav.range  # hovering burns as fast as cruising
        for depot in self.depots:
            depot.advance(dt, params.uav.turnaround)
        self._moved = {}
        for uav in self.uavs:
            if uav.state is UAVState.ON_ROUTE:
                step = self._speed.get(uav.id, speed) * dt
                uav.arc = (uav.arc + step) % self.loop.length
                uav.position = self.loop.point_at(uav.arc)
                uav.battery -= burn
                self._moved[uav.id] = step
            elif uav.state is UAVState.RELAYING:
                uav.fly_leg(speed * dt)
                uav.battery -= burn
            elif uav.state is UAVState.RETURNING:
                flown, arrived = uav.fly_leg(speed * dt)
                uav.battery -= flown / params.uav.range
                if arrived:
                    self.depots[uav.home].arrive(uav, params.uav.turnaround)
                    self.events.add(
                        self.time + dt, "arrive", uav.id, f"battery {uav.battery:.3f}"
                    )

    def _hand_over(self, dt: float) -> None:
        """Reliefs that have arrived take over their slot; the incumbent flies home."""
        v = self.params.uav.speed
        for uav in self.uavs:
            if uav.state is not UAVState.RELAYING:
                continue
            incumbent = uav.relieving
            if incumbent is None:
                uav.meet_ahead -= v * dt
            elif incumbent.state is UAVState.ON_ROUTE:
                uav.meet_ahead -= self._moved[incumbent.id]
            gone = incumbent is not None and incumbent.state is not UAVState.ON_ROUTE
            if uav.leg or not (gone or uav.meet_ahead <= 0.0):
                continue
            overshoot = min(max(-uav.meet_ahead, 0.0), v * dt) if not gone else 0.0
            uav.arc = (uav.meet_arc + overshoot) % self.loop.length
            uav.position = self.loop.point_at(uav.arc)
            uav.state = UAVState.ON_ROUTE
            uav.relieving = None
            uav.launched_at = self.time
            uav.replaced = False
            self.events.add(self.time, "handover", uav.id, f"arc {uav.arc:.0f}")
            if incumbent is not None and incumbent.state is UAVState.ON_ROUTE:
                self._send_home(incumbent, "relieved")

    def _observe(self) -> None:
        radius = self.params.uav.sensing_radius
        for uav in self.uavs:
            if not uav.airborne:
                continue
            for cell in self.grid.cells_within(uav.position, radius):
                self.coverage.observe(cell.index, self.time)

    def save(self, out_dir: str | Path, summary: dict) -> None:
        """Write metrics, events, the loop and summary to ``out_dir``."""
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        self.monitor.write_csv(out / "metrics.csv")
        self.events.write_csv(out / "events.csv")
        with open(out / "loop.csv", "w") as handle:
            handle.write("order,x,y,arc_m\n")
            vertices = zip(self.loop.vertices, self.loop.arcs)
            handle.writelines(
                f"{k},{x},{y},{arc}\n" for k, ((x, y), arc) in enumerate(vertices)
            )
        with open(out / "summary.json", "w") as handle:
            json.dump(summary, handle, indent=2)
