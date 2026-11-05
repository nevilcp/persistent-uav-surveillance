"""Command-line entry point."""

from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path

from .params import Parameters
from .simulation import Simulation


def override(params: Parameters, assignment: str) -> Parameters:
    """Parameters with one ``section.name=value`` override applied."""
    path, _, text = assignment.partition("=")
    section, _, name = path.partition(".")
    if not (section and name and text):
        raise SystemExit(f"--set expects section.name=value, got {assignment!r}")
    current = getattr(getattr(params, section), name)
    value = json.loads(text)
    value = tuple(value) if isinstance(current, tuple) else type(current)(value)
    return replace(
        params, **{section: replace(getattr(params, section), **{name: value})}
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="python -m uav_surveil",
        description="Simulate persistent UAV-swarm surveillance.",
    )
    parser.add_argument(
        "--config", type=Path, help="JSON parameter file (default: nominal parameters)"
    )
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="SECTION.NAME=VALUE",
        help="override one parameter, e.g. fleet.size=31 (repeatable)",
    )
    parser.add_argument(
        "--failure",
        nargs=2,
        type=float,
        metavar=("ROUTE", "TIME"),
        help="fail the UAV flying route ROUTE at TIME seconds",
    )
    parser.add_argument(
        "--out", type=Path, help="directory for metrics.csv, events.csv, ..."
    )
    args = parser.parse_args()

    params = Parameters.from_json(args.config) if args.config else Parameters()
    for assignment in args.set:
        params = override(params, assignment)
    if args.failure is not None:
        route, time = args.failure
        params = replace(
            params,
            failure=replace(params.failure, enabled=True, route=int(route), time=time),
        )

    sim = Simulation(params)
    split = sim.split
    print(
        f"fleet split: {split.n_fly} flying + {split.n_rotate} rotation + {split.n_spare} spare"
    )
    summary = sim.run()
    print(json.dumps(summary, indent=2))
    if args.out:
        sim.save(args.out, summary)
        print(f"results written to {args.out}/")


if __name__ == "__main__":
    main()
