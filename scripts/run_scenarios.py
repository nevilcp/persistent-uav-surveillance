"""Evaluation scenarios and a runner that writes results and figures to ``results/``."""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path

from uav_surveil import Parameters, Simulation

BASE = Parameters()

SCENARIOS: dict[str, Parameters] = {
    "under_provisioned": replace(BASE, fleet=replace(BASE.fleet, size=11)),
    "nominal": BASE,
    "over_provisioned": replace(BASE, fleet=replace(BASE.fleet, size=31)),
    "long_horizon": replace(BASE, mission=replace(BASE.mission, duration=9600.0)),
    "single_failure": replace(BASE, failure=replace(BASE.failure, enabled=True)),
}


def run_all(out: Path) -> dict[str, Simulation]:
    """Every scenario run and saved under ``out``."""
    sims = {}
    for name, params in SCENARIOS.items():
        sim = Simulation(params)
        summary = sim.run()
        sim.save(out / name, summary)
        sims[name] = sim
        print(f"{name:18s} done")
    return sims


def write_table(sims: dict[str, Simulation], path: Path) -> str:
    """Markdown summary table of the runs, written to ``path``."""
    header = (
        "| Scenario | N | Duration (s) | Mean cov. (%) | Peak (%) | Time >= 90% (s) "
        "| Steady-state cov. (%) | C2 violations | C3 violations / alarms |\n"
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|\n"
    )
    rows = []
    for name, sim in sims.items():
        s = sim.monitor.summary(sim.failure)
        c = s["coverage"]
        rows.append(
            f"| {name} | {sim.params.fleet.size} | {s['duration_s']:.0f} | {c['mean_pct']:.1f} "
            f"| {c['peak_pct']:.1f} | {c['time_at_or_above_90pct_s']:.0f} "
            f"| {c['steady_state_mean_pct']:.1f} | {s['c2_battery']['violating_steps']} "
            f"| {s['c3_spare_launch']['violations']} / {s['c3_spare_launch']['alarms']} |"
        )
    table = header + "\n".join(rows) + "\n"
    failure = sims["single_failure"].monitor.summary(sims["single_failure"].failure)[
        "c4_recovery"
    ]
    table += (
        f"\nSingle failure: t_f = {failure['t_fail_s']:.0f} s, hand-over at "
        f"{failure['t_handover_s']:.0f} s, recovery time T_rec = "
        f"{failure['recovery_time_s']:.0f} s (C4 limit {failure['limit_s']:.0f} s), "
        f"{failure['bridge_inserts']} bridge insertions.\n"
    )
    path.write_text(table)
    return table


def plot(sims: dict[str, Simulation], out: Path) -> None:
    """Figures of the runs, written to ``out``."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def series(name: str, key: str) -> tuple[list[float], list[float]]:
        rows = sims[name].monitor.rows
        return [r["time"] for r in rows], [r[key] for r in rows]

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    fleet_sizes = (
        ("under_provisioned", "N = 11"),
        ("nominal", "N = 21"),
        ("over_provisioned", "N = 31"),
    )
    for name, label in fleet_sizes:
        axes[0].plot(*series(name, "coverage_pct"), lw=0.8, label=label)
        axes[1].plot(*series(name, "n_overdue"), lw=0.8, label=label)
    axes[0].axhline(90, color="grey", ls=":", lw=0.8)
    axes[0].set_ylabel("coverage (%)")
    axes[1].set_ylabel("overdue cells")
    axes[1].set_xlabel("time (s)")
    axes[0].legend()
    fig.tight_layout()
    fig.savefig(out / "coverage_vs_fleet_size.png", dpi=150)
    plt.close(fig)

    t_fail = SCENARIOS["single_failure"].failure.time
    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    for name, label in (("nominal", "nominal"), ("single_failure", "failure")):
        axes[0].plot(*series(name, "coverage_pct"), lw=0.8, label=label)
        axes[1].plot(*series(name, "n_overdue"), lw=0.8, label=label)
    fleet_states = (
        ("n_on_route", "on route"),
        ("n_rotation_ready", "rotation ready"),
        ("n_spare_ready", "spare ready"),
    )
    for key, label in fleet_states:
        axes[2].plot(*series("single_failure", key), lw=0.8, label=label)
    for ax in axes:
        ax.axvline(t_fail, color="k", ls="--", lw=0.8)
    axes[0].set_ylabel("coverage (%)")
    axes[1].set_ylabel("overdue cells")
    axes[2].set_ylabel("UAVs (failure run)")
    axes[2].set_xlabel("time (s)")
    axes[0].legend()
    axes[2].legend()
    fig.tight_layout()
    fig.savefig(out / "failure_response.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    axes[0].plot(*series("nominal", "n_rotation_ready"), lw=0.8)
    axes[0].set_ylabel("rotation UAVs ready")
    axes[1].plot(*series("nominal", "n_routes_uncovered"), lw=0.8)
    axes[1].set_ylabel("routes without a UAV")
    axes[1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(out / "rotation_pool.png", dpi=150)
    plt.close(fig)


def main() -> None:
    """Command-line entry point."""
    parser = argparse.ArgumentParser(description="Run the five evaluation scenarios.")
    parser.add_argument("--out", type=Path, default=Path("results"))
    parser.add_argument(
        "--no-plots", action="store_true", help="skip the matplotlib figures"
    )
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    sims = run_all(args.out)
    print()
    print(write_table(sims, args.out / "summary.md"))
    if not args.no_plots:
        plot(sims, args.out)
        print(f"figures written to {args.out}/")


if __name__ == "__main__":
    main()
