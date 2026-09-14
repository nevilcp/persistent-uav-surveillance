"""
Temporary helper to visualize fleet state with an explicit contingency band.

Assumptions (for failure sim_023-style runs):
- UAV 21 is the single contingency spare before failure
- Failure happens at a known time (e.g., t_fail=1800s)
- After t_fail the contingency band disappears (promoted to mission), so the
  overall stacked total drops by one (FAILED aircraft is excluded).

Inputs:
- --base: simulations base tag without suffix (path before _metrics.csv)
- --t-fail: failure time in seconds (default 1800)
- --out: optional output path. If omitted, saved under figures/ with suffix _manual.

This script is intentionally conservative: it reads only metrics.csv and adds a
step function for the contingency band (1 before failure, 0 from failure onward).
It does not infer from route snapshots and does not require rerunning the sim.
"""

from __future__ import annotations

import argparse
import contextlib
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def _read_metrics_csv(path: str) -> dict[str, np.ndarray]:
    import csv

    try:
        import pandas as pd  # type: ignore

        df = pd.read_csv(path)
        df.columns = [c.strip().lower() for c in df.columns]
        return {c: df[c].to_numpy(dtype=float) for c in df.columns}
    except Exception:  # noqa: BLE001 - pandas missing or CSV unreadable, fall back to manual parsing
        cols: dict[str, list[float]] = {}
        with open(path, newline="") as f:
            rdr = csv.DictReader(f)
            for row in rdr:
                for k, v in row.items():
                    key = k.strip().lower()
                    cols.setdefault(key, [])
                    try:
                        cols[key].append(float(v))
                    except (ValueError, TypeError):
                        cols[key].append(float("nan"))
        return {k: np.asarray(v, dtype=float) for k, v in cols.items()}


def plot_manual_contingency(base_tag: str, t_fail: float, out_png: str) -> None:
    metrics_path = base_tag + "_metrics.csv"
    m = _read_metrics_csv(metrics_path)

    t = m.get("time")
    act = m.get("active_uavs")
    swp = m.get("swapping_uavs")
    rot = m.get(
        "spare_uavs"
    )  # rotation spares only (contingency excluded by sim metrics)
    if t is None or act is None or swp is None or rot is None:
        raise SystemExit(
            "metrics CSV missing required columns: time, active_uavs, swapping_uavs, spare_uavs"
        )

    # Contingency step: 1 up to t_fail (exclusive), then 0 afterwards
    cont = np.where(t < float(t_fail), 1.0, 0.0)

    plt.figure(figsize=(10, 4.2))
    ax = plt.gca()
    ax.stackplot(
        t,
        act,
        swp,
        rot,
        cont,
        labels=["On mission", "Swapping", "Rotation spares", "Contingency"],
        colors=["#2a9d8f", "#e9c46a", "#264653", "#7b1fa2"],
        alpha=0.9,
        step="post",
    )

    # Deployed overlay (active + swapping); dotted to avoid clashing with failure marker
    ax.plot(
        t,
        act + swp,
        color="#455a64",
        ls=":",
        lw=1.2,
        label="Deployed UAVs (active + swapping)",
    )

    # Failure annotation
    tf = float(t_fail)
    ax.axvline(tf, color="#6a1b9a", ls="--", lw=1.2)
    # Horizontal failure label near the bottom, offset below tick labels
    ax.annotate(
        f"Failure (t={int(tf)} s)",
        xy=(tf, ax.get_ylim()[0]),
        xycoords="data",
        xytext=(0, -22),  # pixels downward from the axis
        textcoords="offset points",
        ha="center",
        va="top",
        fontsize=9,
        color="#6a1b9a",
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("UAV count")
    ax.set_title("Fleet state over time")
    with contextlib.suppress(Exception):
        from matplotlib.ticker import MaxNLocator

        ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax.legend(loc="lower right", ncol=2)
    ax.grid(True, alpha=0.2)
    _ensure_dir(os.path.dirname(out_png))
    # Give extra bottom margin for the horizontal failure label
    plt.tight_layout()
    plt.gcf().subplots_adjust(bottom=0.18)
    plt.savefig(out_png, dpi=180)
    plt.close()


def main() -> None:
    p = argparse.ArgumentParser(
        description="Temporary manual contingency fleet-state plotter"
    )
    p.add_argument("--base", required=True, help="Base tag (path before _metrics.csv)")
    p.add_argument("--t-fail", type=float, default=1800.0, help="Failure time (s)")
    p.add_argument("--out", default="", help="Output PNG path (optional)")
    p.add_argument(
        "--make-tripanel",
        action="store_true",
        help="Also render a clean failure tripanel with contingency band",
    )
    args = p.parse_args()

    base = args.base
    out = args.out
    if not out:
        tag = os.path.basename(base)
        out = os.path.join("figures", f"{tag}_v15_fleet_state_contingency_manual.png")
    plot_manual_contingency(base, args.t_fail, out)
    print("Saved:", out)

    if args.make_tripanel:
        out_tri = os.path.join(
            "figures", f"{os.path.basename(base)}_v15_fail_timeline.png"
        )
        plot_failure_tripanel_clean(base, args.t_fail, out_tri, theta=180.0)
        print("Saved:", out_tri)


def plot_failure_tripanel_clean(
    base_tag: str, t_fail: float, out_png: str, theta: float = 180.0
) -> None:
    """Three aligned panels: coverage, overdue, fleet state (with manual contingency)."""
    m = _read_metrics_csv(base_tag + "_metrics.csv")
    t = m.get("time")
    cov = m.get("coverage_%")
    roll = m.get("rolling_avg_%")
    overdue = m.get("cells_overdue")
    act = m.get("active_uavs")
    swp = m.get("swapping_uavs")
    rot = m.get("spare_uavs")
    if (
        t is None
        or cov is None
        or overdue is None
        or act is None
        or swp is None
        or rot is None
    ):
        raise SystemExit("metrics CSV missing required columns")

    tf = float(t_fail)
    cont = np.where(t < tf, 1.0, 0.0)

    import matplotlib.pyplot as _plt

    fig, axs = _plt.subplots(3, 1, figsize=(11, 8.0), sharex=True)

    # Panel 1: Coverage
    axs[0].plot(t, cov, lw=1.3, color="#1976d2", label="Coverage %")
    if roll is not None and not np.all(np.isnan(roll)):
        axs[0].plot(t, roll, lw=1.8, color="#ef6c00", label="Rolling avg (240 s)")
    axs[0].axhline(90.0, color="#7cb342", ls=":", lw=1.1)
    axs[0].axvline(tf, color="#6a1b9a", ls="--", lw=1.1)
    axs[0].set_ylabel("Coverage (%)")
    axs[0].legend(loc="lower right", fontsize=9)
    axs[0].grid(True, alpha=0.3)

    # Panel 2: Overdue count
    axs[1].plot(t, overdue, lw=1.3, color="#c23b22", label="# cells > Θ")
    axs[1].axvline(tf, color="#6a1b9a", ls="--", lw=1.1)
    axs[1].set_ylabel("# cells > Θ")
    axs[1].legend(loc="upper right", fontsize=9)
    axs[1].grid(True, alpha=0.3)

    # Panel 3: Fleet state with contingency
    axs[2].stackplot(
        t,
        act,
        swp,
        rot,
        cont,
        labels=["On mission", "Swapping", "Rotation spares", "Contingency"],
        colors=["#2a9d8f", "#e9c46a", "#264653", "#7b1fa2"],
        alpha=0.9,
        step="post",
    )
    # Deployed overlay as dotted line
    axs[2].plot(
        t,
        act + swp,
        color="#455a64",
        ls=":",
        lw=1.2,
        label="Deployed UAVs (active + swapping)",
    )
    axs[2].axvline(tf, color="#6a1b9a", ls="--", lw=1.1)
    # Horizontal failure label below the bottom axis ticks
    axs[2].annotate(
        f"Failure (t={int(tf)} s)",
        xy=(tf, axs[2].get_ylim()[0]),
        xycoords="data",
        xytext=(0, -22),
        textcoords="offset points",
        ha="center",
        va="top",
        fontsize=9,
        color="#6a1b9a",
    )
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("UAV count")
    axs[2].legend(loc="lower right", ncol=2, fontsize=9)
    axs[2].grid(True, alpha=0.2)

    fig.suptitle("Failure timeline: coverage, violations, fleet state", y=0.98)
    _ensure_dir(os.path.dirname(out_png))
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.subplots_adjust(bottom=0.14)
    fig.savefig(out_png, dpi=180)
    _plt.close(fig)


if __name__ == "__main__":
    main()
