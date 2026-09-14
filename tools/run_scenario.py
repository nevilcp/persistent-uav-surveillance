#!/usr/bin/env python3
"""Headless batch scenario runner (Phase 1 WP7).

Runs a GSSSimulation end-to-end without the animation/plotting overhead of
examples/visualize_simulation.py, producing the same result CSVs under
results/ (metrics, periodic coverage-gap snapshots, bridge/failure events,
per-UAV SoC timeseries, and recovery metrics for failure runs).

Usage:
    python tools/run_scenario.py --config baseline --sim-number 27 \
        [--inventory 11] [--duration 9600] [--failure] \
        [--snapshot-period 600] [--quiet]
"""

import argparse
import contextlib
import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir)))

from uav_surveil.analysis_tools import make_simulation_info
from uav_surveil.config import load_scenario
from uav_surveil.gss.simulation import GSSSimulation


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run a GSS simulation headlessly and write its result CSVs."
    )
    parser.add_argument(
        "--config", required=True, help="Scenario name (e.g. baseline, test, urban)"
    )
    parser.add_argument(
        "--sim-number", type=int, required=True, help="Sim tag number (sim_NNN)"
    )
    parser.add_argument(
        "--inventory",
        type=int,
        default=None,
        help="Override optimization.total_inventory",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Override mission.mission_duration (seconds)",
    )
    parser.add_argument(
        "--failure",
        action="store_true",
        help="Enable the scenario's configured failure trigger",
    )
    parser.add_argument(
        "--snapshot-period",
        type=float,
        default=None,
        help="Override stl.snapshot_period_s (seconds)",
    )
    parser.add_argument(
        "--quiet", action="store_true", help="Suppress per-tick simulation logging"
    )
    args = parser.parse_args()

    config = load_scenario(args.config)

    if args.inventory is not None:
        config.optimization.total_inventory = args.inventory
    if args.snapshot_period is not None:
        config.stl.snapshot_period_s = args.snapshot_period
    if args.failure:
        config.failure.enabled = True

    duration = (
        args.duration if args.duration is not None else config.mission.mission_duration
    )

    os.makedirs("results", exist_ok=True)
    simulation_info = make_simulation_info(config, args.sim_number)

    sim = GSSSimulation(config=config)
    # Pass simulation info for consistent file naming, matching
    # visualize_simulation.py (metrics/coverage-gap/recovery CSVs) and wire
    # the bridge/failure event log (only ever read via getattr on sim).
    sim._simulation_info = simulation_info
    sim._events_path = f"results/{simulation_info['base_name']}_events.csv"
    sim._quiet_logging = args.quiet

    try:
        if args.quiet:
            with open(os.devnull, "w") as devnull, contextlib.redirect_stdout(devnull):
                success = sim.run(duration=duration)
        else:
            success = sim.run(duration=duration)
    except ValueError as e:
        print(f"❌ {simulation_info['base_name']}: {e}")
        sys.exit(1)

    if not success:
        print(f"❌ {simulation_info['base_name']}: simulation failed to complete")
        sys.exit(1)

    # run()'s last _log_csv_metrics() call happens while state is still
    # RUNNING, so the recovery/SoC-timeseries export (gated on state !=
    # RUNNING) needs an explicit final call now that the run has stopped.
    sim._export_failure_recovery_csvs()

    print(simulation_info["base_name"])


if __name__ == "__main__":
    main()
