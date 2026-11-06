"""Per-frame state of a scenario as JSON, the input of render_animation.py."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from run_scenarios import SCENARIOS

from uav_surveil import Simulation
from uav_surveil.dispatch import return_threshold
from uav_surveil.routing import loop_distance

ROLLING_WINDOW = 240  # s


def dump(name: str) -> dict:
    """Every time step of a scenario run."""
    params = SCENARIOS[name]
    sim = Simulation(params)
    theta = params.mission.revisit_threshold
    turnaround = params.uav.turnaround
    cells = sim.grid.cells
    mgr = sim.failure
    t_launch = None
    c2 = 0
    frames = []
    steps = round(params.mission.duration / params.mission.time_step)
    for _ in range(steps):
        sim.step()
        t, row = sim.time, sim.monitor.rows[-1]
        if row["battery_margin"] < 0:  # nan compares false
            c2 += 1
        if t_launch is None and mgr is not None and mgr.contingency is not None:
            t_launch = t
        recent = [r["coverage_pct"] for r in sim.monitor.rows[-ROLLING_WINDOW:]]
        late = sum(a.latency(t) > turnaround for a in sim.monitor.alarms)
        frames.append(
            {
                "t": t,
                "fresh": [round(1 - min(sim.coverage.age(i, t) / theta, 1.0), 3) for i in range(len(cells))],
                "uavs": [[f"{u.id:02d}", round(u.position[0], 1), round(u.position[1], 1), round(u.battery, 3)] for u in sim.uavs],
                "active": int(row["n_on_route"]),
                "spare": int(row["n_spare_ready"]),
                "c2": c2,
                "c3_missed": late,
                "c3_alarms": len(sim.monitor.alarms),
                "cov": row["coverage_pct"],
                "roll": sum(recent) / len(recent),
            }
        )

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

    loop = max(loop_distance(r.cells) for r in sim.routes) / params.uav.speed
    # the threshold grows with distance; the one at the region edge is the one the hud quotes
    edge = return_threshold(sim.grid.ferry_distance(params.region.depot), params.uav, params.dispatch)
    return {
        "name": name,
        "title": "Continuous Patrol",
        "meta": f"Baseline  |  Algorithm: Round-Robin  |  Fleet: {params.fleet.size} UAVs",
        "cell_size": params.region.cell_size,
        "cells": [list(c.position) for c in cells],
        "depot": list(params.region.depot),
        "return_pct": round(edge * 100),
        "loop": round(loop),
        "rolling_window": ROLLING_WINDOW,
        "failure": failure,
        "frames": frames,
    }


def main() -> None:
    """Command-line entry point."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=[*SCENARIOS, "all"])
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
