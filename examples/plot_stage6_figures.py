"""
Stage 6 – Verification & Validation figure generator

This script loads existing simulation CSVs (metrics, coverage gaps snapshots,
UAV route snapshots, SoC series) and produces a compact set of figures for
the thesis V&V section.

Inputs: Provide the base name(s) of runs (without suffixes) or let the
script auto-discover the latest steady-state and failure runs.

Outputs: PNG figures saved under `figures/` with descriptive filenames.

Dependencies: matplotlib, numpy. Pandas is optional; a lightweight CSV
fallback is included to avoid dependency issues.
"""

from __future__ import annotations

import contextlib
import csv
import glob
import json
import math
import os
from collections.abc import Sequence
from dataclasses import dataclass

import matplotlib

# Use a non-interactive backend so we can save images without a GUI (no Tk)
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

# -----------------------------------------------------------------------------
# Utilities
# -----------------------------------------------------------------------------


def _ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def _try_read_pandas(path: str):
    """Attempt to read a CSV with pandas; return None if pandas unavailable."""
    try:
        import pandas as pd  # type: ignore

        return pd.read_csv(path)
    except (
        Exception
    ):  # noqa: BLE001 - pandas missing or CSV unreadable, fall back to manual parsing
        return None


def read_metrics_csv(path: str) -> dict[str, np.ndarray]:
    """Read metrics.csv into a dict of numpy arrays with numeric types.

    Expected header (as written by visualize_simulation):
    time,active_uavs,spare_uavs,swapping_uavs,coverage_%,rolling_avg_%,
    cells_overdue,c2_violations,c3_alarms,c3_missed,avg_soc,min_soc
    """
    df = _try_read_pandas(path)
    if df is not None:
        # Normalize column names for robustness
        cols = {c: c.strip().lower() for c in df.columns}
        df.rename(columns=cols, inplace=True)
        out = {c: df[c].to_numpy(dtype=float) for c in df.columns}
        return out

    # Fallback to csv module
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        cols: dict[str, list[float]] = {}
        for row in reader:
            for k, v in row.items():
                key = k.strip().lower()
                if key not in cols:
                    cols[key] = []
                try:
                    cols[key].append(float(v))
                except (ValueError, TypeError):
                    # Treat empty/malformed as nan
                    cols[key].append(float("nan"))
    return {k: np.asarray(v, dtype=float) for k, v in cols.items()}


def read_soc_timeseries(path: str) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Read SoC time series exported by the simulation (time + columns per UAV).

    Returns:
        times (N,), dict of {uav_id: (N,) soc_percent}
    """
    df = _try_read_pandas(path)
    if df is not None:
        df.columns = [c.strip() for c in df.columns]
        time = df.iloc[:, 0].to_numpy(dtype=float)
        series = {str(c): df[c].to_numpy(dtype=float) for c in df.columns[1:]}
        return time, series

    # Fallback
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        ids = [h.strip() for h in header[1:]]
        times: list[float] = []
        data: dict[str, list[float]] = {uid: [] for uid in ids}
        for row in reader:
            if not row:
                continue
            try:
                t = float(row[0])
            except (ValueError, TypeError):
                continue
            times.append(t)
            for uid, val in zip(ids, row[1:]):
                try:
                    data[uid].append(float(val))
                except (ValueError, TypeError):
                    data[uid].append(float("nan"))
    return np.asarray(times, dtype=float), {
        k: np.asarray(v, dtype=float) for k, v in data.items()
    }


def read_coverage_snapshot(path: str) -> dict[str, np.ndarray]:
    """Read a coverage_gaps CSV snapshot exported at a given sim time.

    Returns dict with arrays for 'age' and 'overdue' (0/1).
    """
    df = _try_read_pandas(path)
    if df is not None:
        cols = {c: c.strip().lower() for c in df.columns}
        df.rename(columns=cols, inplace=True)
        overdue = df.get("overdue", 0).astype(float).to_numpy()
        age = df.get("age", 0).astype(float).to_numpy()
        return {"age": age, "overdue": overdue}

    ages: list[float] = []
    overdue: list[float] = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                ages.append(float(row.get("age", "nan")))
            except (ValueError, TypeError):
                ages.append(float("nan"))
            try:
                overdue.append(
                    1.0
                    if row.get("overdue", "False").strip().lower() == "true"
                    else 0.0
                )
            except (ValueError, TypeError):
                overdue.append(float("nan"))
    return {
        "age": np.asarray(ages, dtype=float),
        "overdue": np.asarray(overdue, dtype=float),
    }


# -----------------------------------------------------------------------------
# Plotting
# -----------------------------------------------------------------------------


def plot_coverage_time(
    metrics: dict[str, np.ndarray],
    out_png: str,
    annotations: dict[str, float] | None = None,
    y_tick_step: int = 5,
    show_global_avg: bool = True,
) -> None:
    t = metrics.get("time")
    cov = metrics.get("coverage_%")
    roll = metrics.get("rolling_avg_%")
    if t is None or cov is None:
        return

    plt.figure(figsize=(9, 4))
    plt.plot(t, cov, label="Coverage %", lw=1.5)
    if roll is not None and not np.all(np.isnan(roll)):
        plt.plot(t, roll, label="Rolling avg % (240s)", lw=1.5, alpha=0.8)

    if show_global_avg:
        # Global average from start until time t: cumulative time‑weighted mean
        dt = np.diff(t, prepend=t[0])
        # Prevent division by zero at the first sample
        dt[0] = max(dt[0], 1e-6)
        numer = np.cumsum(cov * dt)
        denom = np.cumsum(dt)
        glob = np.where(denom > 0, numer / denom, np.nan)
        plt.plot(t, glob, label="Global avg % (t0→t)", lw=1.2, ls="--", color="#2e7d32")

    if annotations:
        for label, tx in annotations.items():
            if tx is None or math.isnan(tx):
                continue
            plt.axvline(tx, color="#cc7722", ls="--", lw=1.2)
            # Put text near the top for readability
            ymax = 98
            plt.text(
                tx,
                ymax,
                f"{label}\n(t={tx:.0f}s)",
                rotation=90,
                va="top",
                ha="right",
                fontsize=8,
                color="#cc7722",
            )

    plt.ylim(0, 100)
    if y_tick_step > 0:
        yticks = np.arange(0, 101, y_tick_step)
        plt.yticks(yticks)
    plt.xlabel("Time (s)")
    plt.ylabel("Coverage (%)")
    plt.legend(loc="lower right")
    plt.grid(True, alpha=0.3)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=160)
    plt.close()


def plot_violations_time(metrics: dict[str, np.ndarray], out_png: str) -> None:
    t = metrics.get("time")
    overdue = metrics.get("cells_overdue")
    if t is None or overdue is None:
        return
    plt.figure(figsize=(9, 3.5))
    plt.plot(t, overdue, label="# cells > Θ", color="#c23b22")
    plt.xlabel("Time (s)")
    plt.ylabel("Count")
    plt.title("Revisit-gap violations over time")
    plt.grid(True, alpha=0.3)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=160)
    plt.close()


def plot_compare_coverage(
    metrics_list: Sequence[dict[str, np.ndarray]],
    labels: Sequence[str],
    out_png: str,
    y_tick_step: int = 5,
) -> None:
    plt.figure(figsize=(9, 4))
    for m, label in zip(metrics_list, labels):
        t, cov = m.get("time"), m.get("coverage_%")
        if t is None or cov is None:
            continue
        plt.plot(t, cov, lw=1.5, label=label)
    plt.ylim(0, 100)
    if y_tick_step > 0:
        plt.yticks(np.arange(0, 101, y_tick_step))
    plt.xlabel("Time (s)")
    plt.ylabel("Coverage (%)")
    plt.title("Coverage vs time – comparison")
    plt.legend(loc="lower right")
    plt.grid(True, alpha=0.3)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=160)
    plt.close()


def _rolling_mean(y: np.ndarray, t: np.ndarray, window_s: float) -> np.ndarray:
    if y is None or t is None or len(y) == 0:
        return y
    # Estimate sampling period (default 1s)
    dt = np.nanmedian(np.diff(t)) if len(t) > 1 else 1.0
    win = max(1, round(window_s / max(dt, 1e-6)))
    if win <= 1:
        return y
    # NaN-safe convolution
    y_safe = np.where(np.isnan(y), 0.0, y)
    mask = np.where(np.isnan(y), 0.0, 1.0)
    kern = np.ones(win, dtype=float)
    num = np.convolve(y_safe, kern, mode="same")
    den = np.convolve(mask, kern, mode="same")
    out = np.divide(num, den, out=np.full_like(num, np.nan), where=den > 0)
    return out


def plot_compare_coverage_clean(
    metrics_list: Sequence[dict[str, np.ndarray]],
    labels: Sequence[str],
    out_png: str,
    smooth_s: float = 60.0,
    y_tick_step: int = 5,
) -> None:
    """Cleaner overlay: smoothed coverage, concise labels, subdued grid."""
    plt.figure(figsize=(9, 4))
    palette = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]

    def pretty(lbl: str) -> str:
        label = lbl
        if "sim_024" in label:
            return "Under‑provisioned (N=11)"
        if "sim_020" in label:
            return "Baseline (N=21)"
        if "sim_025" in label:
            return "Over‑provisioned (N=31)"
        if "sim_026" in label:
            return "Baseline long‑run (N=21, 9600s)"
        return label

    for idx, (m, label) in enumerate(zip(metrics_list, labels)):
        t, cov = m.get("time"), m.get("coverage_%")
        if t is None or cov is None:
            continue
        cov_sm = _rolling_mean(cov, t, smooth_s)
        plt.plot(
            t, cov_sm, lw=2.0, label=pretty(label), color=palette[idx % len(palette)]
        )
    plt.ylim(0, 100)
    if y_tick_step > 0:
        plt.yticks(np.arange(0, 101, y_tick_step))
    plt.xlabel("Time (s)")
    plt.ylabel("Coverage (%)")
    plt.title(f"Coverage vs time – smoothed comparison ({int(smooth_s)} s rolling)")
    plt.legend(loc="lower right")
    plt.grid(True, alpha=0.25)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=180)
    plt.close()


def plot_compare_violations_clean(
    metrics_list: Sequence[dict[str, np.ndarray]],
    labels: Sequence[str],
    out_png: str,
    smooth_s: float = 60.0,
) -> None:
    """Smoothed violations overlay with semantic labels."""
    plt.figure(figsize=(9, 3.5))
    palette = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]

    def pretty(lbl: str) -> str:
        label = lbl
        if "sim_024" in label:
            return "Under‑provisioned (N=11)"
        if "sim_020" in label:
            return "Baseline (N=21)"
        if "sim_025" in label:
            return "Over‑provisioned (N=31)"
        if "sim_026" in label:
            return "Baseline long‑run (N=21, 9600s)"
        return label

    for idx, (m, label) in enumerate(zip(metrics_list, labels)):
        t, viol = m.get("time"), m.get("cells_overdue")
        if t is None or viol is None:
            continue
        viol_sm = _rolling_mean(viol, t, smooth_s)
        plt.plot(
            t, viol_sm, lw=2.0, label=pretty(label), color=palette[idx % len(palette)]
        )
    plt.xlabel("Time (s)")
    plt.ylabel("# cells > Θ")
    plt.title(
        f"Revisit-gap violations – smoothed comparison ({int(smooth_s)} s rolling)"
    )
    plt.legend(loc="upper right")
    plt.grid(True, alpha=0.3)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=180)
    plt.close()


def plot_compare_violations(
    metrics_list: Sequence[dict[str, np.ndarray]], labels: Sequence[str], out_png: str
) -> None:
    plt.figure(figsize=(9, 3.5))

    def pretty(lbl: str) -> str:
        label = lbl
        if "sim_024" in label:
            return "Under‑provisioned (N=11)"
        if "sim_020" in label:
            return "Baseline (N=21)"
        if "sim_025" in label:
            return "Over‑provisioned (N=31)"
        if "sim_026" in label:
            return "Baseline long‑run (N=21, 9600s)"
        return label

    for m, label in zip(metrics_list, labels):
        t, viol = m.get("time"), m.get("cells_overdue")
        if t is None or viol is None:
            continue
        plt.plot(t, viol, lw=1.4, label=pretty(label))
    plt.xlabel("Time (s)")
    plt.ylabel("# cells > Θ")
    plt.title("Revisit-gap violations – comparison")
    plt.legend(loc="upper right")
    plt.grid(True, alpha=0.3)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=160)
    plt.close()


def summarize_metrics(metrics: dict[str, np.ndarray]) -> dict[str, float]:
    t = metrics.get("time")
    cov = metrics.get("coverage_%")
    if t is None or cov is None or len(t) == 0:
        return {"avg": float("nan"), "peak": float("nan"), "time_ge_90": float("nan")}
    avg = float(np.nanmean(cov))
    peak = float(np.nanmax(cov))
    # Approximate seconds above 90
    dt = np.diff(t, prepend=t[0])
    dt[0] = max(dt[0], 1.0)
    time_ge_90 = float(np.nansum(dt[cov >= 90.0]))
    return {"avg": avg, "peak": peak, "time_ge_90": time_ge_90}


def _count_overdue_in_snapshot(path: str | None) -> int | None:
    if not path:
        return None
    try:
        df = _try_read_pandas(path)
        if df is not None:
            if "overdue" in df.columns:
                return int(df["overdue"].astype(str).str.lower().eq("true").sum())
            return None
        cnt = 0
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            for r in reader:
                val = str(r.get("overdue", "")).strip().lower()
                if val == "true":
                    cnt += 1
        return cnt
    except (
        Exception
    ):  # noqa: BLE001 - best-effort CSV read, missing/malformed file falls back to None
        return None


def plot_comparison_table(arts: Sequence[RunArtifacts], out_png: str) -> None:
    if not arts:
        return
    labels = [os.path.basename(a.base) for a in arts]
    metrics_list = [read_metrics_csv(a.metrics) for a in arts]
    rows = []
    for a, m, label in zip(arts, metrics_list, labels):
        s = summarize_metrics(m)
        overdue = _count_overdue_in_snapshot(a.final_coverage)
        rows.append(
            [
                label,
                f"{s['avg']:.1f}%",
                f"{s['peak']:.1f}%",
                f"{s['time_ge_90']:.0f}s",
                "" if overdue is None else str(overdue),
            ]
        )

    _fig, ax = plt.subplots(figsize=(8, 1 + 0.4 * len(rows)))
    ax.axis("off")
    col_labels = ["Run", "Avg %", "Peak %", "Time ≥90% (s)", "Final overdue"]
    table = ax.table(cellText=rows, colLabels=col_labels, loc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.0, 1.2)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=200)
    plt.close()


def plot_fleet_state(metrics: dict[str, np.ndarray], out_png: str) -> None:
    t = metrics.get("time")
    active = metrics.get("active_uavs")
    swap = metrics.get("swapping_uavs")
    spare = metrics.get("spare_uavs")
    if t is None or active is None or swap is None or spare is None:
        return
    plt.figure(figsize=(9, 3.8))
    plt.stackplot(
        t,
        active,
        swap,
        spare,
        labels=["On mission", "Swapping", "Spares"],
        colors=["#2a9d8f", "#e9c46a", "#264653"],
        alpha=0.9,
    )
    plt.xlabel("Time (s)")
    plt.ylabel("UAV count")
    plt.title("Fleet state over time")
    plt.legend(loc="upper right")
    with contextlib.suppress(Exception):
        from matplotlib.ticker import MaxNLocator

        ax = plt.gca()
        ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    plt.grid(True, alpha=0.2)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=160)
    plt.close()


def _read_uav_routes_csv(path: str) -> tuple[float, list[tuple[str, str]]]:
    """Lightweight reader for a single UAV routes snapshot CSV.

    Returns:
        (time_s, [(uav_id, state), ...])
    """
    # Fast path with pandas if available
    df = _try_read_pandas(path)
    if df is not None:
        cols = [c.strip().lower() for c in df.columns]
        df.columns = cols
        # Expect first column to be time, and columns 'uav_id','state'
        try:
            tval = float(df.iloc[0, 0]) if len(df) else float("nan")
        except (ValueError, TypeError):
            tval = float("nan")
        pairs = [
            (str(r["uav_id"]).strip(), str(r["state"]).strip().lower())
            for _, r in df.iterrows()
        ]
        return tval, pairs

    # Fallback to csv module
    import csv as _csv

    tval = float("nan")
    pairs: list[tuple[str, str]] = []
    with open(path, newline="") as f:
        rdr = _csv.DictReader(f)
        for i, row in enumerate(rdr):
            if i == 0:
                try:
                    tval = float(row.get("time", "nan"))
                except (ValueError, TypeError):
                    tval = float("nan")
            uid = str(row.get("uav_id", "")).strip()
            st = str(row.get("state", "")).strip().lower()
            pairs.append((uid, st))
    return tval, pairs


def _infer_contingency_ids_from_snapshot(pairs: list[tuple[str, str]]) -> list[str]:
    """Heuristic: treat the highest-numbered UAV id(s) that are spares as contingency.

    Works with our numbering where rotation spares follow active, and the final id(s)
    are contingency. For single-contingency setups this returns a one-element list.
    """
    # Extract numeric ids when possible
    nums: list[int] = []
    for uid, _ in pairs:
        with contextlib.suppress(Exception):
            nums.append(int(uid))
    if not nums:
        return []
    max_id = str(max(nums))
    return [max_id]


def plot_fleet_state_with_contingency(
    base_tag: str,
    metrics: dict[str, np.ndarray],
    out_png: str,
    annotate_fail: dict[str, float] | None = None,
) -> None:
    """Stacked fleet state including a separate band for contingency spares.

    We derive contingency presence from the per-snapshot UAV routes CSVs:
    the highest-numbered UAV id at start is assumed to be the contingency.
    The series shows 1 when that UAV is in state 'spare', and 0 otherwise.
    After failure, when it is promoted to mission, the value drops to 0.
    """
    import glob as _glob

    t = metrics.get("time")
    act = metrics.get("active_uavs")
    swp = metrics.get("swapping_uavs")
    spr = metrics.get("spare_uavs")
    if t is None or act is None or swp is None or spr is None:
        return

    # Discover route snapshots for this run
    routes = sorted(_glob.glob(f"{base_tag}_uav_routes_*.csv"))
    if not routes:
        # Fallback to classic plot if no snapshots available
        return plot_fleet_state(metrics, out_png)

    # Determine contingency id(s) from the earliest snapshot
    _t0, pairs0 = _read_uav_routes_csv(routes[0])
    cont_ids = _infer_contingency_ids_from_snapshot(pairs0)

    # Build step series for contingency spare count at snapshot times
    t_snap: list[float] = []
    c_snap: list[int] = []
    for p in routes:
        ts, pairs = _read_uav_routes_csv(p)
        if ts is None or (isinstance(ts, float) and (np.isnan(ts) or np.isinf(ts))):
            continue
        t_snap.append(float(ts))
        cnt = 0
        for uid, st in pairs:
            if uid in cont_ids and st == "spare":
                cnt += 1
        c_snap.append(cnt)
    if not t_snap:
        return plot_fleet_state(metrics, out_png)

    # Step-fill to per-second metric timeline
    t_snap_np = np.asarray(t_snap, dtype=float)
    c_snap_np = np.asarray(c_snap, dtype=float)
    # Ensure sorted by time
    order = np.argsort(t_snap_np)
    t_snap_np = t_snap_np[order]
    c_snap_np = c_snap_np[order]
    c_series = np.zeros_like(t, dtype=float)
    for i, ti in enumerate(t):
        j = int(np.searchsorted(t_snap_np, ti, side="right") - 1)
        j = max(j, 0)
        c_series[i] = c_snap_np[j]

    # Rotation spares are exactly what's in metrics (contingency already excluded)
    rot_spares = spr.copy()

    # If we know failure time, drop contingency to 0 from t_fail onward (promotion)
    if annotate_fail:
        t_fail = annotate_fail.get("Failure")
        if t_fail is not None and not np.isnan(t_fail):
            c_series = np.where(t >= t_fail, 0.0, c_series)

    # Plot with distinct band for contingency
    plt.figure(figsize=(10, 4.0))
    ax = plt.gca()
    ax.stackplot(
        t,
        act,
        swp,
        rot_spares,
        c_series,
        labels=["On mission", "Swapping", "Rotation spares", "Contingency"],
        colors=["#2a9d8f", "#e9c46a", "#264653", "#7b1fa2"],
        alpha=0.9,
        step="post",
    )
    # Overlay deployed (active+swapping)
    dep = act + swp
    ax.plot(t, dep, color="#455a64", ls="--", lw=1.1, label="Deployed (active+swap)")

    if annotate_fail:
        t_fail = annotate_fail.get("Failure")
        t_rec = annotate_fail.get("Recovered≥90%")
        if t_fail is not None and not np.isnan(t_fail):
            ax.axvline(t_fail, color="#c23b22", ls="--", lw=1.0)
        if t_rec is not None and not np.isnan(t_rec):
            ax.axvline(t_rec, color="#6a1b9a", ls="--", lw=1.0)
        if (
            t_fail is not None
            and t_rec is not None
            and not np.isnan(t_fail)
            and not np.isnan(t_rec)
        ):
            ax.axvspan(t_fail, t_rec, color="#ef9a9a", alpha=0.18)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("UAV count")
    ax.set_title("Fleet state over time (with contingency)")
    with contextlib.suppress(Exception):
        from matplotlib.ticker import MaxNLocator

        ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax.legend(loc="upper right", ncol=2)
    ax.grid(True, alpha=0.2)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=180)
    plt.close()


def plot_coverage_time_with_band(
    metrics: dict[str, np.ndarray],
    out_png: str,
    steady_start_s: float = 600.0,
    q_lo: float = 10.0,
    q_hi: float = 90.0,
    y_tick_step: int = 5,
    show_global_avg: bool = True,
) -> None:
    """Coverage with a horizontal steady-state band (P10–P90 after warmup)."""
    t = metrics.get("time")
    cov = metrics.get("coverage_%")
    roll = metrics.get("rolling_avg_%")
    if t is None or cov is None:
        return
    plt.figure(figsize=(9, 4))
    # Compute band from steady segment
    mask = t >= steady_start_s
    if np.any(mask):
        lo = float(np.nanpercentile(cov[mask], q_lo))
        hi = float(np.nanpercentile(cov[mask], q_hi))
        plt.axhspan(
            lo,
            hi,
            color="#90caf9",
            alpha=0.18,
            label=f"Steady-state band (P{int(q_lo)}–P{int(q_hi)})",
        )
        plt.axvspan(steady_start_s, t[-1], color="#bdbdbd", alpha=0.08)
    # Curves
    plt.plot(t, cov, label="Coverage %", lw=1.2, color="#1976d2", alpha=0.8)
    if roll is not None and not np.all(np.isnan(roll)):
        plt.plot(t, roll, label="Rolling avg % (240s)", lw=1.8, color="#ef6c00")
    if show_global_avg:
        dt = np.diff(t, prepend=t[0])
        dt[0] = max(dt[0], 1e-6)
        glob = np.cumsum(cov * dt) / np.cumsum(dt)
        plt.plot(t, glob, label="Global avg % (t0→t)", lw=1.2, ls="--", color="#2e7d32")
    plt.ylim(0, 100)
    if y_tick_step > 0:
        plt.yticks(np.arange(0, 101, y_tick_step))
    plt.xlabel("Time (s)")
    plt.ylabel("Coverage (%)")
    plt.legend(loc="lower right")
    plt.grid(True, alpha=0.3)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=180)
    plt.close()


def _estimate_recovery_time(
    metrics: dict[str, np.ndarray], t_fail: float, threshold: float = 90.0
) -> float:
    """Return time-to-recover (seconds) when rolling avg ≥ threshold after failure.
    Returns NaN if not reached.
    """
    t = metrics.get("time")
    roll = metrics.get("rolling_avg_%")
    if t is None or roll is None or np.all(np.isnan(roll)):
        return float("nan")
    j0 = int(np.searchsorted(t, t_fail))
    for j in range(j0, len(t)):
        if not np.isnan(roll[j]) and roll[j] >= threshold:
            return float(t[j] - t_fail)
    return float("nan")


def plot_failure_recovery(
    metrics: dict[str, np.ndarray],
    out_png: str,
    t_fail: float,
    t_rec90: float | None = None,
    y_tick_step: int = 5,
) -> None:
    t = metrics.get("time")
    cov = metrics.get("coverage_%")
    roll = metrics.get("rolling_avg_%")
    if t is None or cov is None:
        return
    if t_rec90 is None or np.isnan(t_rec90):
        t_rec90 = _estimate_recovery_time(metrics, t_fail)
    t_rec = t_fail + t_rec90 if not np.isnan(t_rec90) else None

    plt.figure(figsize=(10, 4))
    plt.plot(t, cov, label="Coverage %", lw=1.2, color="#1976d2")
    if roll is not None and not np.all(np.isnan(roll)):
        plt.plot(t, roll, label="Rolling avg % (240s)", lw=1.8, color="#ef6c00")
    # Shade failure window
    if t_rec is not None:
        plt.axvspan(
            t_fail, t_rec, color="#ef9a9a", alpha=0.25, label="Failure→Recovery ≥90%"
        )
    # Vertical markers
    plt.axvline(t_fail, color="#c23b22", ls="--", lw=1.2)
    if t_rec is not None:
        plt.axvline(t_rec, color="#6a1b9a", ls="--", lw=1.2)
    # Annotate min coverage in the window
    with contextlib.suppress(Exception):
        j0 = int(np.searchsorted(t, t_fail))
        j1 = (
            int(np.searchsorted(t, t_rec))
            if t_rec is not None
            else min(len(t) - 1, j0 + 1200)
        )
        seg = cov[j0:j1]
        if seg.size > 0:
            k = int(np.nanargmin(seg)) + j0
            plt.scatter([t[k]], [cov[k]], color="#c62828", zorder=5)
            plt.text(
                t[k],
                cov[k] - 4,
                f"min {cov[k]:.1f}%",
                ha="center",
                va="top",
                fontsize=8,
                color="#c62828",
            )
    plt.ylim(0, 100)
    if y_tick_step > 0:
        plt.yticks(np.arange(0, 101, y_tick_step))
    plt.xlabel("Time (s)")
    plt.ylabel("Coverage (%)")
    plt.title("Failure transient and recovery")
    plt.legend(loc="lower right")
    plt.grid(True, alpha=0.3)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=180)
    plt.close()


def plot_failure_zoom(
    metrics: dict[str, np.ndarray],
    out_png: str,
    t_fail: float,
    t_rec90: float | None = None,
    left_pad: float = 300.0,
    right_pad: float = 300.0,
    y_margin: float = 5.0,
) -> None:
    t = metrics.get("time")
    cov = metrics.get("coverage_%")
    roll = metrics.get("rolling_avg_%")
    if t is None or cov is None:
        return
    if t_rec90 is None or np.isnan(t_rec90):
        t_rec90 = _estimate_recovery_time(metrics, t_fail)
    t_rec = t_fail + t_rec90 if not np.isnan(t_rec90) else t_fail + 900.0
    t0 = max(t[0], t_fail - left_pad)
    t1 = min(t[-1], t_rec + right_pad)
    mask = (t >= t0) & (t <= t1)
    if not np.any(mask):
        return
    ymin = float(np.nanmin(cov[mask]))
    ymax = float(np.nanmax(cov[mask]))
    plt.figure(figsize=(10, 4))
    plt.plot(t[mask], cov[mask], label="Coverage %", lw=1.2, color="#1976d2")
    if roll is not None and not np.all(np.isnan(roll)):
        plt.plot(
            t[mask], roll[mask], label="Rolling avg % (240s)", lw=1.8, color="#ef6c00"
        )
    plt.axvspan(
        t_fail, t_rec, color="#ef9a9a", alpha=0.25, label="Failure→Recovery ≥90%"
    )
    plt.axvline(t_fail, color="#c23b22", ls="--", lw=1.2)
    plt.axvline(t_rec, color="#6a1b9a", ls="--", lw=1.2)
    plt.xlim(t0, t1)
    plt.ylim(max(0, ymin - y_margin), min(100, ymax + y_margin))
    plt.xlabel("Time (s)")
    plt.ylabel("Coverage (%)")
    plt.title("Failure transient (zoom)")
    plt.legend(loc="lower right")
    plt.grid(True, alpha=0.3)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=180)
    plt.close()


def _detect_reentry_post_fail(
    metrics: dict[str, np.ndarray],
    t_fail: float,
    window_s: float = 600.0,
    range_pct: float = 3.0,
) -> float | None:
    """Earliest time ≥ t_fail where rolling-average range over next window
    ≤ range_pct."""
    t = metrics.get("time")
    roll = metrics.get("rolling_avg_%")
    cov = metrics.get("coverage_%")
    if t is None or (roll is None and cov is None):
        return None
    if roll is None or np.all(np.isnan(roll)):
        roll = _rolling_mean(cov, t, 240.0)
    dt = np.nanmedian(np.diff(t)) if len(t) > 1 else 1.0
    win = max(1, round(window_s / max(dt, 1e-6)))
    start_idx = int(np.searchsorted(t, t_fail))
    for i in range(start_idx, max(start_idx, len(t) - win)):
        seg = roll[i : i + win]
        if seg.size == 0:
            break
        r = float(np.nanmax(seg) - np.nanmin(seg))
        if r <= range_pct:
            return float(t[i])
    return None


def plot_failure_tripanel(
    metrics: dict[str, np.ndarray],
    out_png: str,
    t_fail: float,
    theta: float = 180.0,
    window_s: float = 600.0,
    range_pct: float = 3.0,
) -> None:
    """Three stacked panels: coverage, overdue, fleet state; shaded fail→reentry."""
    import matplotlib.pyplot as _plt

    t = metrics.get("time")
    cov = metrics.get("coverage_%")
    roll = metrics.get("rolling_avg_%")
    overdue = metrics.get("cells_overdue")
    act = metrics.get("active_uavs")
    swp = metrics.get("swapping_uavs")
    spr = metrics.get("spare_uavs")
    if (
        t is None
        or cov is None
        or overdue is None
        or act is None
        or swp is None
        or spr is None
    ):
        return
    t_rec90 = _estimate_recovery_time(metrics, t_fail)
    t_reentry = _detect_reentry_post_fail(
        metrics, t_fail, window_s=window_s, range_pct=range_pct
    )
    t_rec = t_fail + t_rec90 if not np.isnan(t_rec90) else None

    fig, axs = _plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)

    # Panel 1: coverage
    axs[0].plot(t, cov, lw=1.2, color="#1976d2", label="Coverage %")
    if roll is not None and not np.all(np.isnan(roll)):
        axs[0].plot(t, roll, lw=1.8, color="#ef6c00", label="Rolling avg % (240s)")
    axs[0].axhline(90.0, color="#7cb342", ls=":", lw=1.2)
    if t_reentry is not None:
        axs[0].axvspan(t_fail, t_reentry, color="#ef9a9a", alpha=0.18)
    axs[0].axvline(t_fail, color="#c23b22", ls="--", lw=1.2)
    if t_rec is not None:
        axs[0].axvline(t_rec, color="#6a1b9a", ls="--", lw=1.2)
    # Min coverage annotation in [t_fail, t_rec or t_fail+900]
    with contextlib.suppress(Exception):
        j0 = int(np.searchsorted(t, t_fail))
        j1 = (
            int(np.searchsorted(t, t_rec))
            if t_rec is not None
            else min(len(t) - 1, j0 + int(window_s * 2))
        )
        seg = cov[j0:j1]
        if seg.size > 0:
            k = int(np.nanargmin(seg)) + j0
            axs[0].scatter([t[k]], [cov[k]], color="#c62828", zorder=5)
            axs[0].text(
                t[k],
                cov[k] - 4,
                f"min {cov[k]:.1f}%",
                ha="center",
                va="top",
                fontsize=8,
                color="#c62828",
            )
    axs[0].set_ylabel("Coverage (%)")
    axs[0].legend(loc="lower right", fontsize=9)
    axs[0].grid(True, alpha=0.3)

    # Panel 2: overdue
    axs[1].plot(t, overdue, color="#c23b22", lw=1.4, label="# overdue > Θ")
    if t_reentry is not None:
        axs[1].axvspan(t_fail, t_reentry, color="#ef9a9a", alpha=0.18)
    axs[1].axvline(t_fail, color="#c23b22", ls="--", lw=1.2)
    if t_rec is not None:
        axs[1].axvline(t_rec, color="#6a1b9a", ls="--", lw=1.2)
    # Peak annotation
    with contextlib.suppress(Exception):
        k = int(np.nanargmax(overdue))
        axs[1].scatter([t[k]], [overdue[k]], color="#b71c1c", zorder=5)
        axs[1].text(
            t[k],
            overdue[k],
            f"peak {int(overdue[k])}",
            ha="center",
            va="bottom",
            fontsize=8,
            color="#b71c1c",
        )
    axs[1].set_ylabel("# cells > Θ")
    axs[1].legend(loc="upper right", fontsize=9)
    axs[1].grid(True, alpha=0.3)

    # Panel 3: fleet state
    axs[2].stackplot(
        t,
        act,
        swp,
        spr,
        labels=["On mission", "Swapping", "Spares"],
        colors=["#2a9d8f", "#e9c46a", "#264653"],
        alpha=0.9,
    )
    if t_reentry is not None:
        axs[2].axvspan(t_fail, t_reentry, color="#ef9a9a", alpha=0.18)
    axs[2].axvline(t_fail, color="#c23b22", ls="--", lw=1.0)
    if t_rec is not None:
        axs[2].axvline(t_rec, color="#6a1b9a", ls="--", lw=1.0)
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("UAV count")
    axs[2].legend(loc="upper right", ncol=3, fontsize=8)
    axs[2].grid(True, alpha=0.2)

    fig.suptitle("Failure timeline: coverage, overdue, fleet state", y=0.98)
    _ensure_dir(os.path.dirname(out_png))
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_png, dpi=180)
    _plt.close(fig)


def plot_cost_index(
    metrics: dict[str, np.ndarray], out_png: str, rate_per_hour: float = 1.0
) -> None:
    """Plot a simple operating cost index: cumulative UAV-hours × rate.

    If an absolute rate is not relevant, keep `rate_per_hour=1.0` to produce
    a dimensionless cost index comparable across runs.
    """
    t = metrics.get("time")
    active = metrics.get("active_uavs")
    if t is None or active is None:
        return
    dt = np.diff(t, prepend=t[0])
    uav_hours = np.cumsum(active * dt) / 3600.0
    cost = uav_hours * rate_per_hour
    plt.figure(figsize=(9, 3.5))
    plt.plot(t, cost, color="#6a1b9a")
    plt.xlabel("Time (s)")
    ylabel = "Cumulative cost index"
    if rate_per_hour != 1.0:
        ylabel += f" (rate={rate_per_hour:.1f} per UAV-hour)"
    plt.ylabel(ylabel)
    plt.title("Operating cost index (∝ active UAV-hours)")
    plt.grid(True, alpha=0.3)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=160)
    plt.close()


def plot_soc_series(
    times: np.ndarray,
    series: dict[str, np.ndarray],
    out_png: str,
    highlight: Sequence[str] | None = None,
    soc_floor: float | None = None,
) -> None:
    plt.figure(figsize=(9, 3.5))
    # Choose a small subset if many columns
    keys = list(series.keys())
    if highlight:
        # Place highlight first
        keys = list(dict.fromkeys(list(highlight) + keys))
    keys = keys[:6]
    for uid in keys:
        plt.plot(times, series[uid], label=f"UAV {uid}")
    if soc_floor is not None:
        plt.axhline(
            soc_floor * 100.0, color="#c23b22", ls="--", lw=1.2, label="SoC floor"
        )
    plt.xlabel("Time (s)")
    plt.ylabel("SoC (%)")
    plt.title("SoC traces (selected UAVs)")
    plt.legend(loc="upper right", ncol=2, fontsize=8)
    plt.grid(True, alpha=0.3)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=160)
    plt.close()


def plot_revisit_hist(
    snapshot_path: str,
    out_png: str,
    theta: float = 180.0,
    snapshot_time: float | None = None,
) -> None:
    data = read_coverage_snapshot(snapshot_path)
    ages = data["age"]
    plt.figure(figsize=(8, 3.2))
    bins = np.linspace(0, max(theta * 2, np.nanmax(ages) + 1), 30)
    plt.hist(ages, bins=bins, color="#457b9d", alpha=0.85)
    plt.axvline(
        theta,
        color="#c23b22",
        ls="--",
        lw=1.2,
        label=f"Theta (max revisit) = {theta:.0f}s",
    )
    plt.xlabel("Cell age (s)")
    plt.ylabel("# cells")
    title = "Revisit-age distribution (snapshot)"
    if snapshot_time is not None:
        title += f" – t={snapshot_time:.0f}s"
    plt.title(title)
    plt.legend(loc="upper right")
    plt.grid(True, alpha=0.3)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=160)
    plt.close()


def plot_revisit_percentiles(
    snapshot_paths: Sequence[str],
    out_png: str,
    theta: float = 180.0,
    snapshot_period_s: float = 600.0,
) -> None:
    if not snapshot_paths:
        return
    times: list[float] = []
    p50: list[float] = []
    p90: list[float] = []
    p99: list[float] = []
    p100: list[float] = []
    for i, p in enumerate(snapshot_paths):
        data = read_coverage_snapshot(p)
        ages = data["age"]
        times.append((i + 1) * snapshot_period_s)
        p50.append(float(np.nanpercentile(ages, 50)))
        p90.append(float(np.nanpercentile(ages, 90)))
        p99.append(float(np.nanpercentile(ages, 99)))
        p100.append(float(np.nanmax(ages)))

    plt.figure(figsize=(9, 3.6))
    plt.plot(times, p50, label="P50 age", lw=1.6)
    plt.plot(times, p90, label="P90 age", lw=1.6)
    plt.plot(times, p99, label="P99 age", lw=1.6)
    plt.plot(times, p100, label="Max age", lw=1.6)
    plt.axhline(
        theta,
        color="#c23b22",
        ls="--",
        lw=1.2,
        label=f"Theta (max revisit) = {theta:.0f}s",
    )
    plt.xlabel("Time (s)")
    plt.ylabel("Cell age (s)")
    plt.title("Revisit age percentiles over time (snapshot-based)")
    plt.legend(loc="upper right", ncol=2)
    plt.grid(True, alpha=0.3)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=160)
    plt.close()


# -----------------------------------------------------------------------------
# Orchestration
# -----------------------------------------------------------------------------


@dataclass
class RunArtifacts:
    base: str  # e.g. results/sim_020_roundrobin_...
    metrics: str
    soc_series: str | None
    recovery_metrics: str | None
    coverage_snaps: list[str]
    final_coverage: str | None


def find_run_artifacts(tag: str) -> RunArtifacts | None:
    """Find artifacts for a run by partial tag, e.g., 'sim_020_roundrobin_2025'."""
    # Metrics
    m = sorted(glob.glob(f"results/{tag}*_metrics.csv"))
    # Prefer the main run metrics over any *_recovery_metrics.csv helper file
    m = [p for p in m if not p.endswith("_recovery_metrics.csv")] or m
    if not m:
        return None
    metrics = m[-1]
    base = metrics.replace("_metrics.csv", "")

    # Optional extras
    soc = sorted(glob.glob(f"{base}_soc_timeseries.csv"))
    rec = sorted(glob.glob(f"{base}_recovery_metrics.csv"))
    covs = sorted(glob.glob(f"{base}_coverage_gaps_*.csv"))
    final_cov = None
    finals = sorted(glob.glob(f"{base}_final_coverage_gaps_*.csv"))
    if finals:
        final_cov = finals[-1]

    return RunArtifacts(
        base=base,
        metrics=metrics,
        soc_series=soc[-1] if soc else None,
        recovery_metrics=rec[-1] if rec else None,
        coverage_snaps=covs,
        final_coverage=final_cov,
    )


def _parse_failure_time_from_tag(base_tag: str) -> float | None:
    """Extract t_fail from base tag like '...FAIL-time-u03-t1800_...'."""
    with contextlib.suppress(Exception):
        if "FAIL-time-" in base_tag and "-t" in base_tag:
            seg = base_tag.split("-t")[-1]
            num = "".join(ch for ch in seg if ch.isdigit())
            if num:
                return float(num)
    return None


def _load_snapshot_ages(paths: Sequence[str]) -> np.ndarray | None:
    if not paths:
        return None
    ages_list: list[np.ndarray] = []
    for p in paths:
        d = read_coverage_snapshot(p)
        ages_list.append(d["age"])
    try:
        return np.vstack(ages_list)  # shape: [S, C]
    except ValueError:
        return None


def _load_snapshot_overdue(paths: Sequence[str]) -> np.ndarray | None:
    if not paths:
        return None
    o_list: list[np.ndarray] = []
    for p in paths:
        d = read_coverage_snapshot(p)
        o_list.append(d["overdue"])
    try:
        return np.vstack(o_list)  # shape: [S, C]
    except ValueError:
        return None


def plot_orphan_percentiles(
    snapshot_paths: Sequence[str],
    out_png: str,
    t_fail: float,
    snapshot_period_s: float = 600.0,
    top_k: int = 100,
    theta: float = 180.0,
    epsilon: float = 60.0,
) -> None:
    """Approximate orphan set: cells whose age jump is largest immediately
    after failure. Plot their age percentiles over subsequent snapshots,
    with Θ and Θ+ε.
    """
    if not snapshot_paths:
        return
    ages = _load_snapshot_ages(snapshot_paths)
    if ages is None:
        return
    S, C = ages.shape
    # Determine snapshot index right before and after failure
    idx_fail = int(
        np.floor(t_fail / snapshot_period_s)
    )  # 0-based; snapshot i ~ (i+1)*period
    # Map our snapshot timing convention
    after_idx = min(max(1, idx_fail), S - 1)
    before_idx = max(0, after_idx - 1)
    # Age jump
    delta = ages[after_idx] - ages[before_idx]
    # Choose top_k indices with largest positive jump
    k = min(top_k, C)
    orphan_idx = np.argsort(delta)[-k:]
    # Percentiles over subsequent snapshots for orphan set
    times = [(i + 1) * snapshot_period_s for i in range(after_idx, S)]
    series = ages[after_idx:S, :][:, orphan_idx]
    p50 = np.nanpercentile(series, 50, axis=1)
    p90 = np.nanpercentile(series, 90, axis=1)
    p99 = np.nanpercentile(series, 99, axis=1)

    plt.figure(figsize=(9, 3.6))
    plt.plot(times, p50, label="Orphan P50", lw=1.6)
    plt.plot(times, p90, label="Orphan P90", lw=1.6)
    plt.plot(times, p99, label="Orphan P99", lw=1.6)
    plt.axhline(theta, color="#c23b22", ls="--", lw=1.2, label=f"Θ = {theta:.0f}s")
    plt.axhline(
        theta + epsilon, color="#8e24aa", ls=":", lw=1.4, label=f"Θ+ε ({int(epsilon)}s)"
    )
    plt.xlabel("Time (s)")
    plt.ylabel("Age (s)")
    plt.title("Orphan-cell age percentiles after failure (approx.)")
    plt.legend(loc="upper right", ncol=2)
    plt.grid(True, alpha=0.3)
    _ensure_dir(os.path.dirname(out_png))
    plt.tight_layout()
    plt.savefig(out_png, dpi=160)
    plt.close()


def plot_orphan_coverage(
    snapshot_paths: Sequence[str],
    out_cov_png: str,
    out_vio_png: str,
    t_fail: float,
    snapshot_period_s: float = 600.0,
    top_k: int = 100,
    theta: float = 180.0,
) -> None:
    """Coverage% and overdue count over time for approximate orphan set
    (top-K age jump)."""
    ages = _load_snapshot_ages(snapshot_paths)
    overdue = _load_snapshot_overdue(snapshot_paths)
    if ages is None or overdue is None:
        return
    S, C = ages.shape
    idx_fail = int(np.floor(t_fail / snapshot_period_s))
    after_idx = min(max(1, idx_fail), S - 1)
    before_idx = max(0, after_idx - 1)
    delta = ages[after_idx] - ages[before_idx]
    k = min(top_k, C)
    orphan_idx = np.argsort(delta)[-k:]

    times = [(i + 1) * snapshot_period_s for i in range(after_idx, S)]
    series_age = ages[after_idx:S, :][:, orphan_idx]
    series_over = overdue[after_idx:S, :][:, orphan_idx]

    # Coverage% within Θ for orphan set and overdue count
    cov_pct = (series_age <= theta).mean(axis=1) * 100.0
    vio_cnt = (series_over > 0.5).sum(axis=1)

    # Coverage plot
    plt.figure(figsize=(9, 3.4))
    plt.plot(times, cov_pct, lw=1.6, color="#1976d2")
    plt.axhline(100.0, color="#7cb342", ls=":", lw=1.0)
    plt.xlabel("Time (s)")
    plt.ylabel("Orphan coverage (%)")
    plt.title("Coverage of orphan set after failure (approx.)")
    plt.grid(True, alpha=0.3)
    _ensure_dir(os.path.dirname(out_cov_png))
    plt.tight_layout()
    plt.savefig(out_cov_png, dpi=160)
    plt.close()

    # Overdue count plot
    plt.figure(figsize=(9, 3.2))
    plt.plot(times, vio_cnt, lw=1.6, color="#c23b22")
    plt.xlabel("Time (s)")
    plt.ylabel("# orphan cells > Θ")
    plt.title("Orphan overdue count after failure (approx.)")
    plt.grid(True, alpha=0.3)
    _ensure_dir(os.path.dirname(out_vio_png))
    plt.tight_layout()
    plt.savefig(out_vio_png, dpi=160)
    plt.close()


def load_recovery_annotations(rec_path: str | None) -> dict[str, float]:
    ann: dict[str, float] = {}
    if not rec_path:
        return ann
    # recovery_metrics.csv has header: t_fail, min_coverage_post_fail,
    # time_under_90_s, recovery_time_to_90_s
    with contextlib.suppress(Exception):
        with open(rec_path, newline="") as f:
            r = list(csv.DictReader(f))
        if not r:
            return ann
        row = r[0]
        t_fail = float(row.get("t_fail", "nan"))
        rec_90 = float(row.get("recovery_time_to_90_s", "nan"))
        if not math.isnan(t_fail):
            ann["Failure"] = t_fail
        if not math.isnan(t_fail) and not math.isnan(rec_90):
            ann["Recovered≥90%"] = t_fail + rec_90
    return ann


def _detect_steady_start(
    metrics: dict[str, np.ndarray],
    warmup_s: float = 600.0,
    window_s: float = 600.0,
    range_pct: float = 3.0,
) -> float:
    """Heuristic: earliest t ≥ warmup_s where range(rolling_avg) ≤ range_pct
    over a window of length window_s. Falls back to warmup_s if not found.
    """
    t = metrics.get("time")
    roll = metrics.get("rolling_avg_%")
    cov = metrics.get("coverage_%")
    if t is None or (roll is None and cov is None):
        return warmup_s
    if roll is None or np.all(np.isnan(roll)):
        roll = _rolling_mean(cov, t, 240.0)
    dt = np.nanmedian(np.diff(t)) if len(t) > 1 else 1.0
    win = max(1, round(window_s / max(dt, 1e-6)))
    start_idx = np.searchsorted(t, warmup_s)
    for i in range(start_idx, max(start_idx, len(t) - win)):
        seg = roll[i : i + win]
        if seg.size == 0:
            break
        r = float(np.nanmax(seg) - np.nanmin(seg))
        if r <= range_pct:
            return float(t[i])
    return warmup_s


def generate_figures_for_run(
    art: RunArtifacts,
    theta: float = 180.0,
    soc_floor: float = 0.10,
    cost_rate_per_hour: float = 1.0,
    snapshot_period_s: float = 600.0,
    suffix: str = "",
    band_start: str | None = None,
    band_window_s: float = 600.0,
    band_range_pct: float = 3.0,
) -> list[str]:
    """Generate all Stage-6 figures for the given run.

    Returns list of saved PNG paths.
    """
    out_paths: list[str] = []
    figs_dir = "figures"
    base_tag = os.path.basename(art.base)

    # Metrics-based plots
    metrics = read_metrics_csv(art.metrics)
    ann = load_recovery_annotations(art.recovery_metrics)
    tag = f"{base_tag}{('_' + suffix) if suffix else ''}"
    p = os.path.join(figs_dir, f"{tag}_coverage_time.png")
    plot_coverage_time(metrics, p, annotations=ann, y_tick_step=5, show_global_avg=True)
    out_paths.append(p)
    # Additional: coverage with steady-state band
    # Determine steady band start
    steady_start = 600.0
    if band_start:
        if band_start.strip().lower() == "auto":
            steady_start = _detect_steady_start(
                metrics,
                warmup_s=600.0,
                window_s=band_window_s,
                range_pct=band_range_pct,
            )
        else:
            try:
                steady_start = float(band_start)
            except (ValueError, TypeError):
                steady_start = 600.0
    p_band = os.path.join(figs_dir, f"{tag}_coverage_time_band.png")
    plot_coverage_time_with_band(metrics, p_band, steady_start_s=steady_start)
    out_paths.append(p_band)

    p = os.path.join(figs_dir, f"{tag}_violations_time.png")
    plot_violations_time(metrics, p)
    out_paths.append(p)

    p = os.path.join(figs_dir, f"{tag}_fleet_state.png")
    plot_fleet_state(metrics, p)
    out_paths.append(p)

    p = os.path.join(figs_dir, f"{tag}_cost_index.png")
    plot_cost_index(metrics, p, rate_per_hour=cost_rate_per_hour)
    out_paths.append(p)

    # SoC plots (if available)
    if art.soc_series:
        times, series = read_soc_timeseries(art.soc_series)
        p = os.path.join(figs_dir, f"{tag}_soc_traces.png")
        # Try to highlight contingency '21' and/or bridging neighbors if present
        highlights = [k for k in series if k in {"21", "20", "22"}]
        plot_soc_series(times, series, p, highlight=highlights, soc_floor=soc_floor)
        out_paths.append(p)

    # Revisit histograms
    if art.coverage_snaps:
        # Choose first, middle, last snapshots for a concise progression view
        idxs = [0, len(art.coverage_snaps) // 2, len(art.coverage_snaps) - 1]
        labels = ["early", "mid", "late"]
        for snap_idx, lbl in zip(idxs, labels):
            snap = art.coverage_snaps[snap_idx]
            # Infer snapshot time ≈ (snap_idx+1) * snapshot_period_s, rough but useful
            t_guess: float | None = (snap_idx + 1) * snapshot_period_s
            p = os.path.join(figs_dir, f"{tag}_revisit_hist_{lbl}.png")
            plot_revisit_hist(snap, p, theta=theta, snapshot_time=t_guess)
            out_paths.append(p)

        # Also create percentile evolution plot across all snapshots
        p = os.path.join(figs_dir, f"{tag}_revisit_percentiles.png")
        plot_revisit_percentiles(
            art.coverage_snaps, p, theta=theta, snapshot_period_s=snapshot_period_s
        )
        out_paths.append(p)
    elif art.final_coverage:
        p = os.path.join(figs_dir, f"{tag}_revisit_hist.png")
        plot_revisit_hist(art.final_coverage, p, theta=theta, snapshot_time=None)
        out_paths.append(p)

    return out_paths


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Generate Stage-6 figures from simulation CSVs"
    )
    parser.add_argument(
        "--steady-tag",
        type=str,
        default="sim_020_roundrobin_",
        help="Partial tag to find steady-state run (prefix of filenames)",
    )
    parser.add_argument(
        "--failure-tag",
        type=str,
        default="sim_023_roundrobin_FAIL",
        help="Partial tag to find failure run (prefix of filenames)",
    )
    parser.add_argument("--theta", type=float, default=180.0, help="Revisit gap Θ")
    parser.add_argument(
        "--soc-floor", type=float, default=0.10, help="SoC floor fraction"
    )
    parser.add_argument(
        "--cost-rate",
        type=float,
        default=1.0,
        help="Operating cost rate per UAV-hour for cost index",
    )
    parser.add_argument(
        "--gap-snapshot-period",
        type=float,
        default=600.0,
        help="Assumed period between coverage_gaps snapshots (s)",
    )
    parser.add_argument(
        "--suffix",
        type=str,
        default="",
        help="Suffix to append to output filenames to avoid overwrites (e.g., v2)",
    )
    # Steady-state band controls
    parser.add_argument(
        "--band-start",
        type=str,
        default="auto",
        help="Steady band start: seconds or 'auto' (default)",
    )
    parser.add_argument(
        "--band-window",
        type=float,
        default=600.0,
        help="Window (s) for auto band detection",
    )
    parser.add_argument(
        "--band-range",
        type=float,
        default=3.0,
        help="Max rolling-average range (%) to declare steady in window",
    )
    parser.add_argument(
        "--compare-tags",
        type=str,
        default="",
        help=(
            "Semicolon-separated list of run tags to compare "
            "(coverage & violations overlays)"
        ),
    )
    parser.add_argument(
        "--make-table",
        action="store_true",
        help="Export comparison table PNG when --compare-tags is given",
    )
    parser.add_argument(
        "--make-composites",
        action="store_true",
        help="Export composite page per compared run",
    )
    args = parser.parse_args()

    steady = find_run_artifacts(args.steady_tag)
    failure = find_run_artifacts(args.failure_tag)

    if not steady and not failure:
        print("No runs found. Provide --steady-tag/--failure-tag matching your CSVs.")
        return

    _ensure_dir("figures")
    index: dict[str, list[str]] = {}

    if steady:
        print(f"Generating figures for steady-state: {steady.base}")
        index[os.path.basename(steady.base)] = generate_figures_for_run(
            steady,
            theta=args.theta,
            soc_floor=args.soc_floor,
            cost_rate_per_hour=args.cost_rate,
            snapshot_period_s=args.gap_snapshot_period,
            suffix=args.suffix,
            band_start=args.band_start,
            band_window_s=args.band_window,
            band_range_pct=args.band_range,
        )

    if failure:
        print(f"Generating figures for failure run: {failure.base}")
        index[os.path.basename(failure.base)] = generate_figures_for_run(
            failure,
            theta=args.theta,
            soc_floor=args.soc_floor,
            cost_rate_per_hour=args.cost_rate,
            snapshot_period_s=args.gap_snapshot_period,
            suffix=args.suffix,
            band_start=args.band_start,
            band_window_s=args.band_window,
            band_range_pct=args.band_range,
        )
        # Also emit dedicated failure transient plots using recovery annotations
        m = read_metrics_csv(failure.metrics)
        ann = load_recovery_annotations(failure.recovery_metrics)
        t_fail = ann.get("Failure") or _parse_failure_time_from_tag(
            os.path.basename(failure.base)
        )
        t_rec_line = ann.get("Recovered≥90%")
        t_rec90 = (
            (t_rec_line - t_fail)
            if (t_fail is not None and t_rec_line is not None)
            else None
        )
        if t_fail is not None:
            out_full = os.path.join(
                "figures",
                os.path.basename(failure.base) + f"_{args.suffix}_fail_recovery.png",
            )
            plot_failure_recovery(m, out_full, t_fail=t_fail, t_rec90=t_rec90)
            out_zoom = os.path.join(
                "figures",
                os.path.basename(failure.base) + f"_{args.suffix}_fail_zoom.png",
            )
            plot_failure_zoom(m, out_zoom, t_fail=t_fail, t_rec90=t_rec90)
            out_tri = os.path.join(
                "figures",
                os.path.basename(failure.base) + f"_{args.suffix}_fail_timeline.png",
            )
            plot_failure_tripanel(m, out_tri, t_fail=t_fail, theta=args.theta)
            # Orphan-percentile plot (Θ+ε visualization)
            if failure.coverage_snaps:
                orphan_out = os.path.join(
                    "figures",
                    os.path.basename(failure.base)
                    + f"_{args.suffix}_orphan_percentiles.png",
                )
                plot_orphan_percentiles(
                    failure.coverage_snaps,
                    orphan_out,
                    t_fail=t_fail,
                    snapshot_period_s=args.gap_snapshot_period,
                    top_k=100,
                    theta=args.theta,
                    epsilon=60.0,
                )
                orphan_cov = os.path.join(
                    "figures",
                    os.path.basename(failure.base)
                    + f"_{args.suffix}_orphan_coverage.png",
                )
                orphan_vio = os.path.join(
                    "figures",
                    os.path.basename(failure.base)
                    + f"_{args.suffix}_orphan_overdue.png",
                )
                plot_orphan_coverage(
                    failure.coverage_snaps,
                    orphan_cov,
                    orphan_vio,
                    t_fail=t_fail,
                    snapshot_period_s=args.gap_snapshot_period,
                    top_k=100,
                    theta=args.theta,
                )

    # Comparison overlays
    if args.compare_tags:
        tags = [t.strip() for t in args.compare_tags.split(";") if t.strip()]
        arts = [find_run_artifacts(t) for t in tags]
        arts = [a for a in arts if a is not None]
        if len(arts) >= 2:
            metric_paths = [a.metrics for a in arts]
            labels = [os.path.basename(a.base) for a in arts]
            metrics_list = [read_metrics_csv(p) for p in metric_paths]

            comp_dir = "figures"
            tag = "_vs_".join(lbl.split("_")[1] for lbl in labels)
            cov_out = os.path.join(comp_dir, f"compare_coverage_{tag}.png")
            plot_compare_coverage(metrics_list, labels, cov_out, y_tick_step=5)
            # Clean, smoothed variant
            cov_out_clean = cov_out.replace(".png", "_clean.png")
            plot_compare_coverage_clean(
                metrics_list, labels, cov_out_clean, smooth_s=60.0, y_tick_step=5
            )

            tag = "_vs_".join(lbl.split("_")[1] for lbl in labels)
            vio_out = os.path.join(comp_dir, f"compare_violations_{tag}.png")
            plot_compare_violations(metrics_list, labels, vio_out)
            # Clean, smoothed variant
            vio_out_clean = vio_out.replace(".png", "_clean.png")
            plot_compare_violations_clean(
                metrics_list, labels, vio_out_clean, smooth_s=60.0
            )

            # Text summary
            summary = {
                lab: summarize_metrics(m) for lab, m in zip(labels, metrics_list)
            }
            with open(os.path.join(comp_dir, "compare_summary.txt"), "w") as f:
                f.writelines(
                    f"{lab}: avg={s['avg']:.1f}%, peak={s['peak']:.1f}%, "
                    f"time>=90%={s['time_ge_90']:.0f}s\n"
                    for lab, s in summary.items()
                )

            if args.make_table:
                plot_comparison_table(arts, os.path.join(comp_dir, "compare_table.png"))

    # Write an index JSON for convenience
    with open("figures/index.json", "w", newline="") as f:
        json.dump(index, f, indent=2)
    print("\n✅ Stage-6 figures generated. See figures/index.json for list.")


if __name__ == "__main__":
    main()
