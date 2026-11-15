"""Stage 6: verification and validation of clauses C1-C7."""

from __future__ import annotations

import csv
import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

from .airspace import Airspace
from .coverage import CoverageMap
from .depots import Depot, nearest_open_base
from .params import Parameters
from .routing import Route
from .uav import UAV, UAVState

if TYPE_CHECKING:
    from .failure import FailureManager

COLUMNS = (
    "time",
    "coverage_pct",
    "n_overdue",
    "max_age",
    "battery_margin",
    "n_on_route",
    "n_returning",
    "n_swapping",
    "n_rotation_ready",
    "n_spare_ready",
    "n_failed",
    "n_routes_uncovered",
)
# written only for runs with zones, several bases or a slot limit, so the paper's CSV stays as is
EXTENDED_COLUMNS = (*COLUMNS, "way_home_margin", "zone_clearance", "max_base_queue")


@dataclass
class Alarm:
    """Request for a reserve UAV, either ``rotation`` or ``failure``."""

    time: float
    route: int
    kind: str
    launch_time: float | None = None

    def latency(self, now: float) -> float:
        """Launch latency; the waiting time so far if unanswered."""
        return (now if self.launch_time is None else self.launch_time) - self.time


class EventLog:
    """Chronological record of discrete GSM decisions."""

    def __init__(self) -> None:
        self.rows: list[tuple[float, str, int | str, str]] = []

    def add(self, time: float, kind: str, uav_id: int | str, detail: str = "") -> None:
        self.rows.append((time, kind, uav_id, detail))

    def write_csv(self, path: Path) -> None:
        with open(path, "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(("time", "event", "uav", "detail"))
            writer.writerows(self.rows)


class Monitor:
    """Per-step time series and the verdict on clauses C1-C7."""

    def __init__(
        self,
        params: Parameters,
        routes: list[Route],
        airspace: Airspace | None = None,
        depots: Sequence[Depot] = (),
    ) -> None:
        self._params = params
        self._routes = routes
        self._airspace = airspace
        self._depots = depots
        self.extended = airspace is not None and bool(
            airspace.zones or len(airspace.depots) > 1 or params.uav.depot_slots
        )
        self.rows: list[dict[str, float]] = []
        self.alarms: list[Alarm] = []
        self._airborne_steps = 0
        self._transit_steps = 0
        self._queued_since: dict[int, float] = {}
        self._longest_wait = 0.0
        # per base: (peak UAVs present, time of the peak, peak number waiting)
        self._busiest = [(0, 0.0, 0) for _ in depots]

    def raise_alarm(self, time: float, route: int, kind: str) -> Alarm:
        alarm = Alarm(time, route, kind)
        self.alarms.append(alarm)
        return alarm

    def open_alarms(self) -> list[Alarm]:
        return [a for a in self.alarms if a.launch_time is None]

    def record(self, time: float, uavs: list[UAV], coverage: CoverageMap) -> None:
        fresh = coverage.n_fresh(time, self._params.mission.revisit_threshold)
        sigma_min = self._params.uav.sigma_min

        def count(state: UAVState) -> int:
            return sum(1 for u in uavs if u.state is state)

        waiting = [u for u in uavs if u.state is UAVState.READY and u.route is None]
        flown = {u.route.id for u in uavs if u.state is UAVState.ON_ROUTE and u.route}
        uncovered = sum(r.departure < time and r.id not in flown for r in self._routes)

        row = {
            "time": time,
            "coverage_pct": 100.0 * fresh / len(coverage),
            "n_overdue": len(coverage) - fresh,
            "max_age": coverage.max_age(time),
            "battery_margin": min(
                (u.battery - sigma_min for u in uavs if u.airborne),
                default=math.nan,
            ),
            "n_on_route": count(UAVState.ON_ROUTE),
            "n_returning": count(UAVState.RETURNING),
            "n_swapping": count(UAVState.SWAPPING),
            "n_rotation_ready": sum(not u.is_contingency for u in waiting),
            "n_spare_ready": sum(u.is_contingency for u in waiting),
            "n_failed": count(UAVState.FAILED),
            "n_routes_uncovered": uncovered,
        }
        if self.extended:
            row.update(self._extended_row(time, uavs))
            self._track_load(time, uavs)
        self.rows.append(row)

    def _extended_row(self, time: float, uavs: list[UAV]) -> dict[str, float]:
        """C5, C7 and C6 values of one step; nan where the clause has no subject."""
        air, uav = self._airspace, self._params.uav
        flying = [u for u in uavs if u.airborne]
        way_home = []
        for u in flying:
            try:
                d = nearest_open_base(air, self._depots, u.position, "flyable")[1]
            except ValueError:
                # inside a zone there is no flyable way; the straight line is a lower bound
                d = nearest_open_base(air, self._depots, u.position, "straight")[1]
            way_home.append(u.battery - d / uav.range - uav.sigma_min)
        beta = self._params.region.zone_margin
        clearance = [air.signed_distance(u.position) - beta for u in flying]
        queue = [d.waiting - d.slots for d in self._depots if d.slots]
        return {
            "way_home_margin": min(way_home, default=math.nan),
            "zone_clearance": min(
                (c for c in clearance if math.isfinite(c)), default=math.nan
            ),
            "max_base_queue": max(queue, default=math.nan),
        }

    def _track_load(self, time: float, uavs: list[UAV]) -> None:
        """Transit share, longest base queue wait and each base's busiest moment."""
        for u in uavs:
            if not u.airborne:
                continue
            self._airborne_steps += 1
            # a round-robin UAV is ferrying until its first waypoint
            ferrying = u.route is not None and u.waypoints_reached == 0
            if u.state is not UAVState.ON_ROUTE or ferrying:
                self._transit_steps += 1
        queued = {u.id for u in uavs if u.state is UAVState.QUEUED}
        for uid in queued - self._queued_since.keys():
            self._queued_since[uid] = time
        for uid in self._queued_since.keys() - queued:
            self._longest_wait = max(
                self._longest_wait, time - self._queued_since.pop(uid)
            )
        for d in self._depots:
            present = len(d.in_service) + d.waiting
            peak = self._busiest[d.index]
            if present > peak[0]:
                self._busiest[d.index] = (present, time, max(peak[2], d.waiting))
            else:
                self._busiest[d.index] = (*peak[:2], max(peak[2], d.waiting))

    def write_csv(self, path: Path) -> None:
        with open(path, "w", newline="") as handle:
            columns = EXTENDED_COLUMNS if self.extended else COLUMNS
            writer = csv.DictWriter(handle, fieldnames=columns)
            writer.writeheader()
            writer.writerows(self.rows)

    def summary(
        self,
        failure: FailureManager | None,
        breakdowns: Sequence[tuple[float, int]] = (),
    ) -> dict:
        """Headline statistics and the verdict on every clause.

        ``breakdowns`` are the injected (time, UAV id) pairs.
        """
        p, dt = self._params, self._params.mission.time_step
        now = self.rows[-1]["time"]
        coverage = [r["coverage_pct"] for r in self.rows]
        steady = [
            r["coverage_pct"]
            for r in self.rows
            if r["time"] > p.mission.steady_state_start
        ]
        margins = [
            r["battery_margin"]
            for r in self.rows
            if not math.isnan(r["battery_margin"])
        ]
        latencies = [a.latency(now) for a in self.alarms]
        theta = p.mission.revisit_threshold

        result = {
            "duration_s": now,
            "coverage": {
                "mean_pct": _mean(coverage),
                "peak_pct": max(coverage),
                "time_at_or_above_90pct_s": dt * sum(c >= 90.0 for c in coverage),
                "steady_state_mean_pct": _mean(steady),
                "mean_overdue_cells": _mean([r["n_overdue"] for r in self.rows]),
                "final_overdue_cells": int(self.rows[-1]["n_overdue"]),
            },
            "c1_all_cells_fresh_s": dt * sum(r["max_age"] <= theta for r in self.rows),
            "c2_battery": {
                "min_margin": min(margins, default=None),
                "violating_steps": sum(m < 0.0 for m in margins),
            },
            "c3_spare_launch": {
                "alarms": len(self.alarms),
                "max_latency_s": max(latencies, default=0.0),
                "violations": sum(lat > p.uav.turnaround for lat in latencies),
            },
        }
        if failure is not None and failure.t_fail is not None:
            t_rec = failure.recovery_time
            result["c4_recovery"] = {
                "t_fail_s": failure.t_fail,
                "t_handover_s": failure.t_handover,
                "recovery_time_s": t_rec,
                "limit_s": theta + p.failure.recovery_tolerance,
                "satisfied": t_rec is not None
                and t_rec <= theta + p.failure.recovery_tolerance,
                "bridge_inserts": failure.n_inserts,
            }
        if self.extended:
            if breakdowns:
                result["c4_breakdowns"] = self._recoveries(breakdowns)
            result.update(self._extended_summary(now))
            result["passed"] = all(
                result[key]["violating_steps"] == 0
                for key in (
                    "c2_battery",
                    "c5_way_home",
                    "c6_base_queue",
                    "c7_zone_clearance",
                )
            )
        return result

    def _recoveries(self, breakdowns: Sequence[tuple[float, int]]) -> list[dict]:
        """C4 per breakdown: time until no cell is older than Theta again."""
        p = self._params
        theta = p.mission.revisit_threshold
        limit = theta + p.failure.recovery_tolerance
        ends = [t for t, _ in breakdowns[1:]] + [math.inf]
        found = []
        for (t_fail, uav_id), end in zip(breakdowns, ends):
            window = [r for r in self.rows if t_fail <= r["time"] < end]
            stale = [r["time"] for r in window if r["max_age"] > theta]
            if not stale:
                recovery = 0.0
            else:
                restored = next(
                    (r["time"] for r in window
                     if r["time"] > stale[0] and r["max_age"] <= theta),
                    None,
                )
                recovery = None if restored is None else restored - t_fail
            found.append(
                {
                    "t_fail_s": t_fail,
                    "uav": uav_id,
                    "recovery_time_s": recovery,
                    "limit_s": limit,
                    "satisfied": recovery is not None and recovery <= limit,
                }
            )
        return found

    def _extended_summary(self, now: float) -> dict:
        """C5-C7 verdicts, robustness and the load figures."""

        def series(name: str) -> list[float]:
            return [r[name] for r in self.rows if not math.isnan(r[name])]

        way_home = series("way_home_margin")
        clearance = series("zone_clearance")
        queue = series("max_base_queue")
        # a UAV still queued at the end has waited at least this long
        pending = max((now - t for t in self._queued_since.values()), default=0.0)
        return {
            "c5_way_home": {
                "min_margin": min(way_home, default=None),
                "violating_steps": sum(m < 0.0 for m in way_home),
            },
            "c6_base_queue": {
                "max_excess": max(queue, default=None),
                "min_margin": -max(queue) if queue else None,
                "violating_steps": sum(q > 0.0 for q in queue),
            },
            "c7_zone_clearance": {
                "min_clearance_m": min(clearance, default=None),
                "violating_steps": sum(c < 0.0 for c in clearance),
            },
            "longest_queue_wait_s": max(self._longest_wait, pending),
            "transit_share": self._transit_steps / self._airborne_steps
            if self._airborne_steps
            else None,
            "busiest_moment": [
                {"base": m, "present": n, "at_s": t, "max_waiting": w}
                for m, (n, t, w) in enumerate(self._busiest)
            ],
        }


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None
