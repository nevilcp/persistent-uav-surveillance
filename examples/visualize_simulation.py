import os, sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir)))
import matplotlib

matplotlib.use("Agg")  # Use non-GUI backend for headless environments
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from uav_surveil.config import load_scenario
from uav_surveil.gss.simulation import GSSSimulation
from uav_surveil.analysis_tools import (
    compare_algorithm_performance,
    log_cell_coverage_gaps,
    make_simulation_info,
)
import numpy as np
import time
from datetime import datetime

# ---------------------------------------------------------------------------
# Visual configuration
# ---------------------------------------------------------------------------
import matplotlib.cm as cm

# Colormap used for UAV SoC (red = empty, green = full)
COLORMAP = cm.get_cmap("RdYlGn")

# Separate colormap for cell staleness (fresh green → red overdue)
CELL_CMAP = cm.get_cmap("RdYlGn")

# Toggle whether to draw per-UAV SoC text labels (may clutter with many UAVs)
SHOW_SOC_LABELS = True
MAX_UAV_LABELS = 25  # draw labels only for first N UAVs if many

# --- Load scenario and initialize simulation ---
config = load_scenario("baseline")

# Generate simulation info for consistent file naming
import datetime
import os

# Ensure simulations directory exists
os.makedirs("results", exist_ok=True)

# ⚙️  MANUAL SIMULATION NUMBER - Change this for each run!
SIMULATION_NUMBER = 27  # 🔄 INCREMENT THIS FOR EACH NEW SIMULATION

# Store simulation info globally for analysis tools
simulation_info = make_simulation_info(config, SIMULATION_NUMBER)
filename_base = simulation_info["base_name"]

sim = GSSSimulation(config)
# Pass simulation info for consistent file naming
sim._simulation_info = simulation_info
sim.initialize()

# --- Visualization parameters ---
CELL_SIZE = config.grid.cell_size
AREA_W = config.mission.area_width
AREA_L = config.mission.area_length
DEPOT_X = config.mission.depot_x
DEPOT_Y = config.mission.depot_y
REVISIT_GAP = config.stl.max_revisit_gap

# --- Simulation & output settings ---
SIM_DURATION = int(
    config.mission.mission_duration
)  # match config mission duration (e.g., 7200s)
FRAME_INTERVAL = 50  # ms between frames in output file (visual playback rate)
N_FRAMES = int(SIM_DURATION / sim.metrics.time_step)  # one step = 1 s

# Output format toggles
SAVE_GIF = False  # set True if you still want a GIF snapshot
SAVE_MP4 = True  # MP4 only by default (requires ffmpeg)

# --- Prepare grid for plotting ---
cells = sim.cells
uavs = sim.uavs

# Compute longest loop for title
longest_loop = max(r.loop_time or 0 for r in sim.routes) if sim.routes else 0

fig, ax = plt.subplots(figsize=(8, 8))
# After AREA variables, compute origin
ORIGIN_X, ORIGIN_Y = config.grid.origin

# Update limits
xmin = min(DEPOT_X, ORIGIN_X) - 50
xmax = max(DEPOT_X, ORIGIN_X + AREA_W) + 50
ymin = min(DEPOT_Y, ORIGIN_Y) - 50
ymax = max(DEPOT_Y, ORIGIN_Y + AREA_L) + 50

ax.set_xlim(xmin, xmax)
ax.set_ylim(ymin, ymax)
ax.set_aspect("equal")
title_str = (
    f"Continuous Patrol – {config.config_name} | Algo: {config.optimization.route_algorithm} "
    f"| Fleet {len(uavs)} UAVs"
)
ax.set_title(title_str)

# If events.csv exists for this run, we will show only compact markers in-plot
_events_path = None
try:
    if hasattr(sim, "_simulation_info") and sim._simulation_info:
        _events_path = f"results/{sim._simulation_info['base_name']}_events.csv"
except Exception:
    _events_path = None

# Draw depot
ax.plot([DEPOT_X], [DEPOT_Y], marker="*", color="red", markersize=15, label="Depot")

# Draw grid cell rectangles
cell_patches = []
for cell in cells:
    rect = plt.Rectangle(
        (cell.x - CELL_SIZE / 2, cell.y - CELL_SIZE / 2),
        CELL_SIZE,
        CELL_SIZE,
        color="white",
        ec="gray",
        lw=0.5,
    )
    ax.add_patch(rect)
    cell_patches.append(rect)

# UAV scatter plot, colored by SoC
uav_scatter = ax.scatter(
    [uav.x for uav in uavs],
    [uav.y for uav in uavs],
    c=[COLORMAP(uav.soc) for uav in uavs],
    marker="^",
    edgecolors="k",
    s=90,
    label="UAVs",
)

# Colorbar for SoC scale
import matplotlib as mpl

norm = mpl.colors.Normalize(vmin=0, vmax=1)
cbar = fig.colorbar(
    mpl.cm.ScalarMappable(norm=norm, cmap=COLORMAP), ax=ax, fraction=0.046, pad=0.04
)
cbar.set_label("UAV SoC  /  Cell Freshness")

# Prepare optional SoC text labels
uav_labels = []
if SHOW_SOC_LABELS:
    for i, uav in enumerate(uavs[:MAX_UAV_LABELS]):
        lbl = ax.text(uav.x, uav.y + 5, "", fontsize=6, ha="center", va="bottom")
        uav_labels.append(lbl)

# Add UAV ID number labels (always shown)
uav_id_labels = []
for i, uav in enumerate(uavs):
    # UAV ID is now just a number like "01", "02", etc.
    uav_number = uav.id
    lbl = ax.text(
        uav.x,
        uav.y - 8,
        uav_number,
        fontsize=8,
        ha="center",
        va="top",
        fontweight="bold",
        color="black",
        bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.8),
    )
    uav_id_labels.append(lbl)

# HUD text placeholder (updated each frame)
hud_text = ax.text(
    0.01,
    0.99,
    "",
    transform=ax.transAxes,
    va="top",
    ha="left",
    fontsize=9,
    bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7),
)
# Move failure banner outside the axes, bottom center of the figure
fail_text = fig.text(
    0.5,
    0.02,
    "",
    va="bottom",
    ha="center",
    fontsize=8,
    color="purple",
    bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.7),
)

# Legend
ax.legend(loc="upper right")


# --- Animation update function ---
def update(frame):
    # Step simulation
    sim.step()
    current_time = sim.metrics.current_time
    # Update grid cell colors based on age (fresh green → red when overdue)
    for i, cell in enumerate(cells):
        age = cell.age(current_time)
        frac = min(age / REVISIT_GAP, 1.0)
        # CELL_CMAP: 1 → green, 0 → red; we want fresh green so invert frac
        cell_patches[i].set_facecolor(CELL_CMAP(1 - frac))
    # Update UAV positions
    if uavs:
        positions = np.array([[uav.x, uav.y] for uav in uavs])
        uav_scatter.set_offsets(positions)
        # Update UAV colors based on SoC
        uav_scatter.set_facecolors([COLORMAP(uav.soc) for uav in uavs])
    else:
        # Handle empty UAV list
        uav_scatter.set_offsets(np.empty((0, 2)))
        uav_scatter.set_facecolors([])

    # Update HUD metrics
    total_time = sim.metrics.current_time if sim.metrics.current_time > 0 else 1
    c2_pct = sim.metrics.stl_c2_violations / total_time * 100
    c3_pct = sim.metrics.stl_c3_violations / total_time * 100

    hud_text.set_text(
        f"t = {current_time:4.0f}s\n"
        f"Active {sim.metrics.active_uavs}  Spare {sim.metrics.spare_uavs}  "
        f"Return@{config.battery.soc_return_threshold*100:.0f}%\n"
        f"C2 violations {sim.metrics.stl_c2_violations}  "
        f"C3: {sim.metrics.c3_missed}/{sim.metrics.c3_alarms} miss  Loop {longest_loop:.0f}s\n"
        f"Coverage {sim.metrics.coverage_percentage:4.1f}%  "
        f"Rolling(240s) {sim.metrics.rolling_coverage_avg:4.1f}%"
    )

    # Draw failure markers if available
    if hasattr(sim, "_failure_markers") and sim._failure_markers:
        t_fail = sim._failure_markers.get("t_fail")
        t_takeover = sim._failure_markers.get("t_takeover_start")
        t_handover = sim._failure_markers.get("t_handover")
        # Minimal on-plot markers
        if t_fail is not None:
            ax.scatter([xmin + 20], [ymin + 20], s=0)  # keep axes warmed
        # Banner text kept concise (outside plot)
        # Concise banner text
        parts = []
        if t_fail is not None and current_time >= t_fail:
            parts.append(f"Fail t={t_fail:.0f}s")
        if t_takeover is not None and current_time >= t_takeover:
            parts.append("contingency launched")
        if t_handover is not None and current_time >= t_handover:
            parts.append("handover complete")
        fail_text.set_text("  |  ".join(parts))

    # Update SoC text labels (limited to first MAX_UAV_LABELS)
    if SHOW_SOC_LABELS:
        for lbl, uav in zip(uav_labels, uavs[:MAX_UAV_LABELS]):
            lbl.set_position((uav.x, uav.y + 5))
            lbl.set_text(f"{uav.soc*100:3.0f}%")

    # Update UAV ID number labels
    for lbl, uav in zip(uav_id_labels, uavs):
        uav_number = uav.id
        lbl.set_position((uav.x, uav.y - 8))
        lbl.set_text(uav_number)

    return cell_patches + [uav_scatter, hud_text] + uav_labels + uav_id_labels


# --- Run animation and save as MP4 ---
ani = animation.FuncAnimation(
    fig, update, frames=N_FRAMES, interval=FRAME_INTERVAL, blit=True
)

# Use existing simulation info for file naming
mp4_path = f"results/{simulation_info['base_name']}.mp4"
print(f"Saving simulation: {mp4_path}")

# Save as MP4 with descriptive name
fps = int(1000 / FRAME_INTERVAL)
ani.save(mp4_path, writer="ffmpeg", fps=fps, bitrate=1800)

# 🔍 FINAL ANALYSIS: Post-simulation summary
print(f"\n📈 === Final Simulation Analysis ===")

# ✅ FLUSH CSV: Ensure all metrics are written to disk before reading
if hasattr(sim, "_csv_writer") and sim._csv_writer:
    sim._csv_writer.writerow(
        [
            sim.metrics.current_time,
            sim.metrics.active_uavs,
            sim.metrics.spare_uavs,
            sim.metrics.uavs_swapping_battery,
            round(sim.metrics.coverage_percentage, 1),
            round(sim.metrics.rolling_coverage_avg, 1),
            sim.metrics.cells_overdue,
            sim.metrics.stl_c2_violations,
            sim.metrics.c3_alarms,
            sim.metrics.c3_missed,
            round(sim.metrics.avg_soc, 1),
            round(sim.metrics.min_soc, 1),
        ]
    )
if hasattr(sim, "_csv_file") and sim._csv_file:
    sim._csv_file.flush()

# Use same file naming pattern as simulation
filename = None
if hasattr(sim, "_simulation_info") and sim._simulation_info:
    info = sim._simulation_info
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"results/{info['base_name']}_final_coverage_gaps_{timestamp}.csv"

final_coverage_gaps, overdue_cells = log_cell_coverage_gaps(
    sim.cells,
    sim.metrics.current_time,
    threshold=config.stl.max_revisit_gap,
    filename=filename,
)

# Extract performance metrics for comparison
# Calculate peak coverage from actual simulation data (read CSV file)
peak_coverage = 0.0
global_avg = 0.0

try:
    import pandas as pd

    csv_filename = f"results/{simulation_info['base_name']}_metrics.csv"
    if os.path.exists(csv_filename):
        df = pd.read_csv(csv_filename)
        peak_coverage = df["coverage_%"].max() if "coverage_%" in df.columns else 0.0
        global_avg = df["coverage_%"].mean() if "coverage_%" in df.columns else 0.0
        print(f"📊 Peak calculated from CSV: {peak_coverage:.1f}%")
except ImportError:
    print("⚠️  Pandas not available - install with: pip install pandas")
    # Manual CSV parsing fallback
    csv_filename = f"results/{simulation_info['base_name']}_metrics.csv"
    if os.path.exists(csv_filename):
        try:
            with open(csv_filename, "r") as f:
                lines = f.readlines()
            coverage_values = []
            for line in lines[1:]:  # Skip header
                if line.strip():
                    parts = line.split(",")
                    if len(parts) > 4:  # coverage_% is 5th column (index 4)
                        try:
                            coverage = float(parts[4])
                            coverage_values.append(coverage)
                        except (ValueError, IndexError):
                            continue
            if coverage_values:
                peak_coverage = max(coverage_values)
                global_avg = sum(coverage_values) / len(coverage_values)
                print(f"📊 Peak calculated manually: {peak_coverage:.1f}%")
            else:
                peak_coverage = sim.metrics.coverage_percentage
                global_avg = sim.metrics.coverage_percentage
        except Exception as e:
            print(f"⚠️  CSV parsing failed: {e}")
            peak_coverage = sim.metrics.coverage_percentage
            global_avg = sim.metrics.coverage_percentage
    else:
        peak_coverage = sim.metrics.coverage_percentage
        global_avg = sim.metrics.coverage_percentage
except Exception as e:
    print(f"⚠️  Error calculating metrics: {e}")
    peak_coverage = sim.metrics.coverage_percentage
    global_avg = sim.metrics.coverage_percentage

# If failure enabled, write recovery metrics and SoC series
if getattr(config, "failure", None) and getattr(config.failure, "enabled", False):
    base = simulation_info["base_name"]
    # Recovery metrics from metrics CSV
    try:
        t_fail = getattr(sim, "_failure_markers", {}).get("t_fail", None)
        src = f"results/{base}_metrics.csv"
        if os.path.exists(src) and t_fail is not None:
            times, coverages = [], []
            with open(src, "r") as f:
                rows = f.readlines()[1:]
            for line in rows:
                parts = line.strip().split(",")
                if len(parts) > 4:
                    try:
                        times.append(float(parts[0]))
                        coverages.append(float(parts[4]))
                    except ValueError:
                        pass
            min_cov = None
            time_under_90 = 0.0
            rec_time_90 = None
            for t, c in zip(times, coverages):
                if t >= t_fail:
                    min_cov = c if min_cov is None else min(min_cov, c)
                    if c < 90.0:
                        time_under_90 += 1.0
                    elif rec_time_90 is None:
                        rec_time_90 = t - t_fail
            if rec_time_90 is None and times:
                rec_time_90 = times[-1] - t_fail
            with open(f"results/{base}_recovery_metrics.csv", "w", newline="") as f:
                import csv as _csv

                w = _csv.writer(f)
                w.writerow(
                    [
                        "t_fail",
                        "min_coverage_post_fail",
                        "time_under_90_s",
                        "recovery_time_to_90_s",
                    ]
                )
                w.writerow(
                    [
                        f"{t_fail:.0f}",
                        f"{min_cov:.1f}" if min_cov is not None else "",
                        f"{time_under_90:.0f}",
                        f"{rec_time_90:.0f}" if rec_time_90 is not None else "",
                    ]
                )
    except Exception as e:
        print(f"⚠️  Recovery metrics export failed: {e}")

    # SoC time series dump
    try:
        if hasattr(sim, "_soc_log") and sim._soc_log:
            ids = sorted(sim._soc_log.keys())
            length = max((len(v) for v in sim._soc_log.values()), default=0)
            with open(f"results/{base}_soc_timeseries.csv", "w", newline="") as f:
                import csv as _csv

                w = _csv.writer(f)
                w.writerow(["time"] + ids)
                for i in range(length):
                    t_val = None
                    row = []
                    for uid in ids:
                        if i < len(sim._soc_log[uid]):
                            t, soc = sim._soc_log[uid][i]
                            if t_val is None:
                                t_val = t
                            row.append(f"{soc*100:.1f}")
                        else:
                            row.append("")
                    if t_val is not None:
                        w.writerow([f"{t_val:.0f}"] + row)
    except Exception as e:
        print(f"⚠️  SoC series export failed: {e}")

rolling_avg = (
    sim.metrics.rolling_coverage_avg
    if hasattr(sim.metrics, "rolling_coverage_avg")
    else 0
)
overdue_count = len(overdue_cells)

print(f"\n🏆 Performance Summary - {route_algo.upper()}")
print(f"   Peak Coverage: {peak_coverage:.1f}%")
print(f"   Global Average: {global_avg:.1f}%")
print(f"   Window Average: {rolling_avg:.1f}% (last 240s)")
print(f"   Final Overdue Cells: {overdue_count}")
print(f"   Simulation Duration: {sim.metrics.current_time:.0f}s")
print("=" * 50)

# plt.show()  # Commented out to avoid GUI errors in headless environments
