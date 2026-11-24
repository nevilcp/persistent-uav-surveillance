"""Per-frame state of a scenario as JSON, the input of render_animation.py."""

from __future__ import annotations

import argparse
import json
import math
from bisect import bisect_left
from collections import deque
from pathlib import Path

from run_port_study import BASE_NAMES, runs, single_base
from run_scenarios import SCENARIOS

from uav_surveil import Parameters, make_simulation
from uav_surveil.dispatch import return_threshold
from uav_surveil.routing import loop_distance
from uav_surveil.uav import UAVState

ROLLING_WINDOW = 240  # s


def scenario(name: str) -> Parameters:
    """Parameters from the baseline or port study."""
    if name in SCENARIOS:
        return SCENARIOS[name]
    if name.startswith("R1_"):
        return single_base(BASE_NAMES.index(name[3:]))
    return runs()[name]


def dump(name: str, params: Parameters | None = None) -> dict:
    """Every time step of a scenario run."""
    params = scenario(name) if params is None else params
    sim = make_simulation(params)
    extended = bool(params.region.depots or params.region.zones)
    theta = params.mission.revisit_threshold
    turnaround = params.uav.turnaround
    cells = sim.airspace.free_cells
    mgr = sim.failure
    t_launch = None
    c2 = 0
    frames = []
    departures = sorted(route.departure for route in sim.routes)
    recent = deque(maxlen=max(1, round(ROLLING_WINDOW / params.mission.time_step)))
    steps = round(params.mission.duration / params.mission.time_step)
    for _ in range(steps):
        sim.step()
        mgr = sim.failure
        t, row = sim.time, sim.monitor.rows[-1]
        if row["battery_margin"] < 0:  # nan compares false
            c2 += 1
        if t_launch is None and mgr is not None and mgr.contingency is not None:
            t_launch = t
        recent.append(row["coverage_pct"])
        late = sum(a.latency(t) > turnaround for a in sim.monitor.alarms)
        margins = row if sim.monitor.extended else sim.monitor.extended_sample(sim.uavs)
        frames.append(
            {
                "t": t,
                "fresh": [round(1 - min(sim.coverage.age(c.index, t) / theta, 1.0), 3) for c in cells],
                "uavs": [[f"{u.id:02d}", round(u.position[0], 1), round(u.position[1], 1), round(u.battery, 3)] for u in sim.uavs],
                "active": int(row["n_on_route"]),
                "spare": int(row["n_spare_ready"]),
                "routes_due": bisect_left(departures, t),
                "routes_unstaffed": int(row["n_routes_uncovered"]),
                "failed": int(row["n_failed"]),
                "c2": c2,
                "c3_missed": late,
                "c3_alarms": len(sim.monitor.alarms),
                "cov": row["coverage_pct"],
                "roll": sum(recent) / len(recent),
                "battery_margin": finite(row["battery_margin"]),
                "way_home_margin": finite(margins["way_home_margin"]),
                "queue_margin": finite(-margins["max_base_queue"]),
                "zone_clearance": finite(margins["zone_clearance"]),
            }
        )
        if extended:
            frames[-1]["closed_depots"] = [d.index for d in sim.depots if not d.is_open]
        if extended or params.failure.enabled:
            frames[-1]["failed_uavs"] = [f"{u.id:02d}" for u in sim.uavs if u.state is UAVState.FAILED]

    failure = None
    if mgr is not None and mgr.t_fail is not None:
        rec = mgr.recovery_time
        failure = {
            "t_fail": mgr.t_fail,
            "uav": mgr.failed_uav.id,
            "route": mgr.route.id,
            "spare": mgr.contingency.id if mgr.contingency else None,
            "t_launch": t_launch,
            "t_handover": mgr.t_handover,
            "t_restored": None if rec is None else mgr.t_fail + rec,
        }

    loop = max(loop_distance(r.cells, sim.airspace) + sim.airspace.distance(r.cells[-1].position, r.cells[0].position)
               for r in sim.routes) / params.uav.speed
    # Use the region-edge threshold as a reference for the scenario caption.
    edge = return_threshold(sim.grid.ferry_distance(params.region.depot), params.uav, params.dispatch)
    data = {
        "name": name,
        "title": "Continuous patrol",
        "meta": f"Baseline  |  Algorithm: Round-Robin  |  Fleet: {params.fleet.size} UAVs",
        "cell_size": params.region.cell_size,
        "cells": [list(c.position) for c in cells],
        "depot": list(params.region.depot),
        "return_pct": round(edge * 100),
        "loop": round(loop),
        "rolling_window": ROLLING_WINDOW,
        "monitor_limits": {
            "freshness": theta,
            "launch": turnaround,
            "recovery": theta + params.failure.recovery_tolerance,
        },
        "recoveries": sim.monitor.recoveries(sim.breakdowns),
        "failure_depots": {f"{f.failed_uav.id:02d}": f.failed_uav.home for f in sim.failures},
        "failure": failure,
        "frames": frames,
    }
    if extended or params.failure.enabled:
        data["events"] = [{"t": t, "kind": kind, "uav": uid, "detail": detail}
                          for t, kind, uid, detail in sim.events.rows if kind in ("failure", "closure")]
    if extended:
        bases = params.region.bases
        edge = return_threshold(max(min(sim.airspace.depot_distance(c.position, m)
                                        for m in range(len(bases))) for c in cells),
                                params.uav, params.dispatch)
        pad = params.region.cell_size * 2
        data.update(
            title="Port surveillance",
            meta=f"{name}  |  {params.patrol.mode.replace('_', ' ').title()}  |  {len(sim.uavs)} UAVs",
            depots=[list(p) for p in bases],
            return_pct=round(edge * 100),
            zones=[list(z) for z in params.region.zones],
            loop_polyline=[],
            planner="choset_round_robin",
            routes=[{"id": r.id, "depot": r.depot, "waypoints": [list(p) for p in r.waypoints]} for r in sim.routes],
            extent=[min(0, *(p[0] for p in bases)) - pad,
                    max(params.region.width, *(p[0] for p in bases)) + pad,
                    min(0, *(p[1] for p in bases)) - pad,
                    max(params.region.length, *(p[1] for p in bases)) + pad],
        )
    return data


def finite(value: float) -> float | None:
    return value if math.isfinite(value) else None


def main() -> None:
    """Command-line entry point."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=[*SCENARIOS, *runs(), *(f"R1_{b}" for b in BASE_NAMES), "all"])
    parser.add_argument("--out", type=Path, default=Path("results/frames"))
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    for name in SCENARIOS if args.scenario == "all" else [args.scenario]:
        data = dump(name)
        path = args.out / f"{name}.json"
        path.write_text(json.dumps(data, separators=(",", ":")))
        print(f"{name}: {len(data['frames'])} frames, {path.stat().st_size // 1024} KB")


if __name__ == "__main__":
    main()
