"""The port study (runs R0-R6 and 20 random maps), written to ``results/port/``."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import random
from dataclasses import dataclass, replace
from pathlib import Path

from uav_surveil import Parameters, make_simulation
from uav_surveil.fleet import split_fleet
from uav_surveil.grid import Grid
from uav_surveil.params import Rect
from uav_surveil.routing import closed_loop_distance

ROOT = Path(__file__).resolve().parent.parent
PORT = Parameters.from_json(ROOT / "configs" / "port.json")
BASE_NAMES = ("west", "east", "north")


@dataclass
class Outcome:
    """One finished run, or the reason it could not be set up."""

    name: str
    params: Parameters
    sim: object | None = None
    summary: dict | None = None
    error: str | None = None

    @property
    def figures(self) -> dict:
        """Headline numbers of the run."""
        s, sim = self.summary, self.sim
        rows = sim.monitor.rows
        # the wait is measured once settled and once every cell has been seen
        first_full = next(
            (r["time"] for r in rows if math.isfinite(r["max_age"])), math.inf
        )
        start = max(self.params.mission.steady_state_start, first_full)
        longest = max(
            (r["max_age"] for r in rows if r["time"] > start), default=math.nan
        )
        return {
            "uavs": sim.split.total,
            "split": f"{sim.split.n_fly}+{sim.split.n_rotate}+{sim.split.n_spare}",
            "max_route_km": max(
                closed_loop_distance(route, sim.airspace) for route in sim.routes
            ) / 1000.0,
            "coverage": s["coverage"]["steady_state_mean_pct"],
            "longest_wait": longest,
            "c2_margin": s["c2_battery"]["min_margin"],
            "c5": s["c5_way_home"]["violating_steps"],
            "c6": s["c6_base_queue"]["violating_steps"],
            "c7": s["c7_zone_clearance"]["violating_steps"],
            "c3": s["c3_spare_launch"]["violations"],
            "alarms": s["c3_spare_launch"]["alarms"],
            "transit": s["transit_share"],
            "passed": s["passed"],
        }


def run(name: str, params: Parameters, out: Path | None = None) -> Outcome:
    """Simulate ``params``; infeasible set-ups are reported instead of raised."""
    try:
        sim = make_simulation(params)
        summary = sim.run()
    except ValueError as error:
        print(f"{name:12s} infeasible: {error}")
        return Outcome(name, params, error=str(error))
    if out is not None:
        sim.save(out / name, summary)
    print(f"{name:12s} done")
    return Outcome(name, params, sim, summary)


def single_base(index: int) -> Parameters:
    """The port with one base that has all the service slots of the three."""
    region = replace(PORT.region, depots=(PORT.region.bases[index],))
    slots = PORT.uav.depot_slots * len(PORT.region.bases)
    return replace(
        PORT,
        region=region,
        uav=replace(PORT.uav, depot_slots=slots),
    )


def runs() -> dict[str, Parameters]:
    """Every run except the single-base ones."""
    failure = PORT.failure
    return {
        "R0": replace(PORT, dispatch=replace(PORT.dispatch, metric="straight")),
        "R2": replace(PORT, region=replace(PORT.region, zones=())),
        "R3": PORT,
        "R4": replace(PORT, failure=replace(failure, enabled=True, time=1800.0, near_depot=1)),
        "R5": replace(
            PORT,
            failure=replace(
                failure,
                enabled=True,
                time=1800.0,
                later_times=(2100.0, 2400.0, 2700.0),
                near_depot=1,
            ),
        ),
        "R6": replace(
            PORT, failure=replace(failure, closed_depot=2, closure_time=1800.0)
        ),
    }


def random_zones(
    seed: int, region=PORT.region, share: float = 0.08
) -> tuple[Rect, ...]:
    """Cell-aligned rectangles over about ``share`` of the cells, free space connected."""
    rng = random.Random(seed)
    grid = Grid(region)
    rows, cols, size = grid.n_rows, grid.n_cols, grid.cell_size
    keep_out = {
        c.index
        for c in grid.cells
        if any(math.dist(c.position, b) <= 3.0 * size for b in region.bases)
    }
    goal = round(share * rows * cols)
    blocked: set[int] = set()
    zones: list[Rect] = []
    for _ in range(5000):
        if len(blocked) >= goal:
            return tuple(zones)
        long, short = rng.randint(4, 12), rng.randint(2, 4)
        w, h = (long, short) if rng.random() < 0.5 else (short, long)
        c0, r0 = rng.randint(0, cols - w), rng.randint(0, rows - h)
        cells = {(r * cols + c) for r in range(r0, r0 + h) for c in range(c0, c0 + w)}
        union = blocked | cells
        if cells & (blocked | keep_out) or len(union) > 1.15 * goal:
            continue
        if not _connected(union, rows, cols):
            continue
        blocked = union
        zones.append((c0 * size, r0 * size, (c0 + w) * size, (r0 + h) * size))
    raise RuntimeError(f"no zone layout found for seed {seed}")


def _connected(blocked: set[int], rows: int, cols: int) -> bool:
    """Whether the free cells form one 4-connected block."""
    free = {i for i in range(rows * cols) if i not in blocked}
    stack, seen = [next(iter(free))], set()
    while stack:
        i = stack.pop()
        if i in seen:
            continue
        seen.add(i)
        r, c = divmod(i, cols)
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            if 0 <= r + dr < rows and 0 <= c + dc < cols:
                j = (r + dr) * cols + c + dc
                if j in free and j not in seen:
                    stack.append(j)
    return len(seen) == len(free)


def run_random_maps(count: int, out: Path) -> list[dict]:
    """The primary Choset/round-robin planner on seeded obstacle maps."""
    rows = []
    for seed in range(count):
        zones = random_zones(seed)
        region = replace(PORT.region, zones=zones)
        row = {"seed": seed, "zones": len(zones)}
        outcome = run(f"R3_map{seed:02d}", replace(PORT, region=region))
        if outcome.error:
            row["R3_error"] = outcome.error
        else:
            f = outcome.figures
            row |= {
                "R3_uavs": f["uavs"],
                "R3_coverage": round(f["coverage"], 2),
                "R3_longest_wait": f["longest_wait"],
                "R3_passed": f["passed"],
                "free_cells": len(outcome.sim.airspace.free_cells),
            }
        rows.append(row)
    keys = sorted({k for r in rows for k in r}, key=lambda k: (k != "seed", k))
    with open(out / "random_maps.csv", "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
    return rows


def fmt(value, spec: str = ".1f") -> str:
    """Table cell: the value, or a dash when there is none."""
    return "-" if value is None else format(value, spec)


def run_table(outcomes: list[Outcome]) -> str:
    """Markdown table of the main runs."""
    header = (
        "| Run | UAVs (fly+rot+spare) | Max route (km) | Steady cov. (%) | Longest wait (s) "
        "| C2 min margin | C3 late / alarms | C5 | C6 | C7 | Transit share | C2, C5-C7 met |\n"
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|\n"
    )
    lines = []
    for o in outcomes:
        if o.error:
            lines.append(f"| {o.name} | infeasible: {o.error} |" + " |" * 10)
            continue
        f = o.figures
        lines.append(
            f"| {o.name} | {f['uavs']} ({f['split']}) | {fmt(f['max_route_km'])} "
            f"| {f['coverage']:.1f} | {f['longest_wait']:.0f} | {f['c2_margin']:.3f} "
            f"| {f['c3']} / {f['alarms']} | {f['c5']} | {f['c6']} | {f['c7']} "
            f"| {fmt(f['transit'], '.2f')} | {'yes' if f['passed'] else 'no'} |"
        )
    note = (
        "\nLongest wait: largest cell age after the first 1200 s and after every cell "
        "has been seen once. Coverage: mean over the steady state.\n"
    )
    return header + "\n".join(lines) + "\n" + note


def recovery_table(outcomes: list[Outcome]) -> str:
    """C4 per breakdown and spare-launch latency of R5 and R6."""
    lines = []
    for o in outcomes:
        if o.error or o.name not in ("R4", "R5", "R6"):
            continue
        s = o.summary
        c3 = s["c3_spare_launch"]
        lines.append(
            f"**{o.name}**: longest launch latency {c3['max_latency_s']:.0f} s."
        )
        for b in s.get("c4_breakdowns", []):
            rec = b["recovery_time_s"]
            lines.append(
                f"- breakdown at {b['t_fail_s']:.0f} s (UAV {b['uav']}): recovery "
                f"{'not reached' if rec is None else f'{rec:.0f} s'} "
                f"(limit {b['limit_s']:.0f} s, {'met' if b['satisfied'] else 'missed'})"
            )
        closure = [e for e in o.sim.events.rows if e[1] in ("closure", "reassign")]
        lines.extend(f"- t = {e[0]:.0f} s {e[1]}: {e[3]}" for e in closure)
    return "\n".join(lines) + "\n"


def random_table(rows: list[dict]) -> str:
    """Summary of the random-map campaign."""

    def stats(key: str) -> str:
        values = [r[key] for r in rows if key in r]
        if not values:
            return "-"
        return (
            f"{min(values):.1f} / {sum(values) / len(values):.1f} / {max(values):.1f}"
        )

    lines = [
        f"{len(rows)} maps, movement deterministic; map seeds 0–{len(rows)-1}.\n",
        "| Metric | Choset + round-robin |",
        "|---|---:|",
        f"| UAVs (min / mean / max) | {stats('R3_uavs')} |",
        f"| Steady coverage % (min / mean / max) | {stats('R3_coverage')} |",
        f"| Longest wait s (min / mean / max) | {stats('R3_longest_wait')} |",
        f"| Maps meeting C2, C5-C7 | {sum(bool(r.get('R3_passed')) for r in rows)} |",
        f"| Maps infeasible | {sum('R3_error' in r for r in rows)} |",
    ]
    return "\n".join(lines) + "\n"


def plot(outcomes: dict[str, Outcome], random_rows: list[dict], out: Path) -> None:
    """Figures of the study, written to ``out``."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    def done(name: str) -> Outcome | None:
        o = outcomes.get(name)
        return o if o is not None and not o.error else None

    def series(o: Outcome, key: str) -> tuple[list[float], list[float]]:
        rows = o.sim.monitor.rows
        return [r["time"] for r in rows], [r[key] for r in rows]

    r4 = done("R4")
    if r4 is not None:
        fig, ax = plt.subplots(figsize=(6, 6))
        for x0, y0, x1, y1 in r4.params.region.zones:
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color="0.6"))
        for route in r4.sim.routes:
            points = []
            for a, b in zip(route.cells, (*route.cells[1:], route.cells[0])):
                points.extend(r4.sim.airspace.path(a.position, b.position))
            xs, ys = zip(*points)
            ax.plot(xs, ys, lw=0.35, alpha=0.5, color=f"C{route.depot}")
        for k, (x, y) in enumerate(r4.params.region.bases):
            ax.plot(x, y, "k^", ms=9)
            ax.annotate(
                BASE_NAMES[k], (x, y), textcoords="offset points", xytext=(6, 6)
            )
        ax.set_aspect("equal")
        ax.set_xlim(-600, 1800)
        ax.set_ylim(-100, 1800)
        fig.tight_layout()
        fig.savefig(out / "port-map.png", dpi=150)
        plt.close(fig)

    compared = [(n, o) for n in ("R0", "R2", "R3", "R4") if (o := done(n))]
    if compared:
        fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        for name, o in compared:
            axes[0].plot(*series(o, "coverage_pct"), lw=0.8, label=name)
            t, age = series(o, "max_age")
            axes[1].plot(
                t,
                [a if math.isfinite(a) else math.nan for a in age],
                lw=0.8,
                label=name,
            )
        theta = PORT.mission.revisit_threshold
        axes[1].axhline(theta, color="grey", ls=":", lw=0.8)
        axes[1].set_ylim(bottom=0)
        axes[0].set_ylabel("coverage (%)")
        axes[1].set_ylabel("longest wait (s)")
        axes[1].set_xlabel("time (s)")
        axes[0].legend()
        fig.tight_layout()
        fig.savefig(out / "coverage-by-run.png", dpi=150)
        plt.close(fig)

    events = [(n, o) for n in ("R5", "R6") if (o := done(n))]
    if events:
        fig, axes = plt.subplots(
            len(events), 1, figsize=(8, 3 * len(events)), sharex=True, squeeze=False
        )
        for ax, (name, o) in zip(axes[:, 0], events):
            t, age = series(o, "max_age")
            ax.plot(t, [a if math.isfinite(a) else math.nan for a in age], lw=0.8)
            ax.axhline(PORT.mission.revisit_threshold, color="grey", ls=":", lw=0.8)
            failure = o.params.failure
            marks = [failure.time, *failure.later_times] if failure.enabled else []
            if failure.closed_depot is not None:
                marks.append(failure.closure_time)
            for m in marks:
                ax.axvline(m, color="k", ls="--", lw=0.8)
            ax.set_ylim(bottom=0)
            ax.set_ylabel(f"{name} longest wait (s)")
        axes[-1, 0].set_xlabel("time (s)")
        fig.tight_layout()
        fig.savefig(out / "breakdown-and-closure.png", dpi=150)
        plt.close(fig)

    if random_rows:
        seeds = [r["seed"] for r in random_rows]
        fig, axes = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
        axes[0].plot(
            seeds, [r.get("R3_coverage", math.nan) for r in random_rows],
            "o-", ms=3, lw=0.6, label="R3",
        )
        axes[1].plot(
            seeds, [r.get("R3_uavs", math.nan) for r in random_rows],
            "o-", ms=3, lw=0.6, label="R3",
        )
        axes[0].set_ylabel("steady coverage (%)")
        axes[1].set_ylabel("UAVs")
        axes[1].set_xlabel("map seed")
        axes[0].legend()
        fig.tight_layout()
        fig.savefig(out / "random-maps.png", dpi=150)
        plt.close(fig)


def main() -> None:
    """Command-line entry point."""
    parser = argparse.ArgumentParser(
        description="Run the port study R0-R6 and the random maps."
    )
    parser.add_argument("--out", type=Path, default=Path("results/port"))
    parser.add_argument(
        "--maps", type=int, default=20, help="number of random maps (0 skips them)"
    )
    parser.add_argument(
        "--only", help="comma-separated runs, e.g. R4,R5 (R1 = the single-base runs)"
    )
    parser.add_argument(
        "--no-plots", action="store_true", help="skip the matplotlib figures"
    )
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    sources = [
        *(ROOT / "uav_surveil").glob("*.py"),
        Path(__file__),
        ROOT / "configs" / "port.json",
    ]
    manifest = {
        "python": platform.python_version(),
        "planner": "choset_round_robin",
        "depot_setback_m": 500,
        "source_sha256": {
            str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(sources)
        },
    }
    (args.out / "reproducibility.json").write_text(json.dumps(manifest, indent=2) + "\n")
    wanted = set(args.only.split(",")) if args.only else None
    chosen = {n: p for n, p in runs().items() if wanted is None or n in wanted}
    outcomes: dict[str, Outcome] = {}
    if wanted is None or "R1" in wanted:
        for index, label in enumerate(BASE_NAMES):
            name = f"R1_{label}"
            outcomes[name] = run(name, single_base(index), args.out)
    for name, params in chosen.items():
        if name == "R3":
            print("R3 split:", split_fleet(params.fleet))
        outcomes[name] = run(name, params, args.out)

    # the best single base is the one that keeps the most cells fresh, then waits least
    singles = [o for n, o in outcomes.items() if n.startswith("R1_") and not o.error]
    best = max(
        singles,
        key=lambda o: (o.figures["coverage"], -o.figures["longest_wait"]),
        default=None,
    )
    ordered = sorted(outcomes.values(), key=lambda o: o.name)
    table = run_table(ordered)
    if best is not None:
        table += f"\nBest single base (R1): {best.name.removeprefix('R1_')}.\n"
    sections = [
        "# Port study: Choset decomposition + round-robin\n",
        (
            "All runs use the new planner, 120 UAVs (87 flying, 27 rotation, 6 spare), "
            "500 m depot setbacks, two service slots per depot (six for single-base runs), "
            "4800 s duration and a 1 s step. Steady state is t > 1200 s. "
            "R0 changes the return metric to straight distance; R1 uses each depot alone; "
            "R2 removes NFZs; R3 is nominal; R4 adds one failure; R5 adds four failures; "
            "R6 closes the north depot. The reported pass flag covers sampled C2/C5/C6/C7 "
            "only; freshness, launch latency and recovery are reported separately.\n"
        ),
        table,
    ]
    recoveries = recovery_table(ordered)
    if recoveries.strip():
        sections += ["\n## Breakdown and closure\n", recoveries]
    random_rows: list[dict] = []
    if args.maps and (wanted is None or "random" in wanted):
        random_rows = run_random_maps(args.maps, args.out)
        sections += ["\n## Random maps\n", random_table(random_rows)]
    text = "\n".join(sections)
    (args.out / "summary.md").write_text(text)
    print()
    print(text)
    if not args.no_plots:
        plot(outcomes, random_rows, args.out)
        print(f"figures written to {args.out}/")


if __name__ == "__main__":
    main()
