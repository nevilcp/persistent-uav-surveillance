"""Stage 6: verification and validation of clauses C1-C4."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

from .coverage import CoverageMap
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
    """Per-step time series and the verdict on clauses C1-C4."""

    def __init__(self, params: Parameters, routes: list[Route]) -> None:
        self._params = params
        self._routes = routes
        self.rows: list[dict[str, float]] = []
        self.alarms: list[Alarm] = []

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
        flown = {u.route.id for u in uavs if u.state is UAVState.ON_ROUTE}
        uncovered = sum(r.departure < time and r.id not in flown for r in self._routes)

        self.rows.append(
            {
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
        )

    def write_csv(self, path: Path) -> None:
        with open(path, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=COLUMNS)
            writer.writeheader()
            writer.writerows(self.rows)

    def summary(self, failure: FailureManager | None) -> dict:
        """Headline statistics and the verdict on every clause."""
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
        return result


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None
