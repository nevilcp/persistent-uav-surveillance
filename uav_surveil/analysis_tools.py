"""
Analysis and logging tools for UAV surveillance simulation.
Provides detailed route analysis, coverage gap detection, and performance comparison.
"""

import csv
import os
from collections.abc import Sequence
from datetime import datetime
from math import hypot
from typing import Any

from .core.cell import Cell
from .core.route import Route


def calculate_route_distance(
    route: Route, cells: Sequence[Cell], depot: tuple[float, float] = (-500.0, 0.0)
) -> float:
    """Calculate total distance for a route including depot legs."""
    if not route.cell_sequence:
        return 0.0

    # Find cell objects from IDs
    cell_dict = {cell.id: cell for cell in cells}
    route_cells = [
        cell_dict[cell_id] for cell_id in route.cell_sequence if cell_id in cell_dict
    ]

    if not route_cells:
        return 0.0

    total_distance = 0.0
    depot_x, depot_y = depot

    # Distance from depot to first cell
    first_cell = route_cells[0]
    total_distance += hypot(first_cell.x - depot_x, first_cell.y - depot_y)

    # Distance between consecutive cells
    for i in range(len(route_cells) - 1):
        cell1 = route_cells[i]
        cell2 = route_cells[i + 1]
        total_distance += hypot(cell2.x - cell1.x, cell2.y - cell1.y)

    # Distance from last cell back to depot
    last_cell = route_cells[-1]
    total_distance += hypot(depot_x - last_cell.x, depot_y - last_cell.y)

    return total_distance


def make_simulation_info(config, number: int) -> dict[str, Any]:
    """Build the {number, algorithm, timestamp, base_name} info dict used to
    tag every CSV/plot output for a run.

    `base_name` follows the sim_<number>_<algo>[_FAIL-...]_<timestamp>
    naming scheme shared by visualize_simulation.py and tools/run_scenario.py.
    """
    route_algo = config.optimization.route_algorithm
    failure_tag = ""
    if getattr(config, "failure", None) and getattr(config.failure, "enabled", False):
        trig = config.failure.trigger
        if trig.kind == "time" and trig.uav_id and trig.t_s is not None:
            failure_tag = f"_FAIL-time-u{trig.uav_id}-t{int(trig.t_s)}"
        elif trig.kind == "position" and trig.uav_id is not None:
            failure_tag = f"_FAIL-pos-u{trig.uav_id}"
        elif trig.kind == "soc" and trig.uav_id and trig.soc_threshold is not None:
            failure_tag = f"_FAIL-soc-u{trig.uav_id}-s{int(trig.soc_threshold*100)}"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")  # noqa: DTZ005
    base_name = f"sim_{number:03d}_{route_algo}{failure_tag}_{timestamp}"

    return {
        "number": number,
        "algorithm": route_algo,
        "timestamp": timestamp,
        "base_name": base_name,
    }


def log_route_analysis(
    routes: list[Route],
    cells: Sequence[Cell],
    algorithm_name: str,
    cruise_speed: float = 15.0,
    depot: tuple[float, float] = (-500.0, 0.0),
):
    """Detailed route analysis and logging."""
    print(f"\n📊 Route Analysis - {algorithm_name.upper()}")
    print("=" * 80)

    total_cells = sum(len(route.cell_sequence) for route in routes)
    total_distance = 0.0
    route_times = []
    route_efficiencies = []

    for i, route in enumerate(routes):
        cells_count = len(route.cell_sequence)
        distance = calculate_route_distance(route, cells, depot)
        time_calc = distance / cruise_speed if distance > 0 else 0.0
        efficiency = cells_count / distance if distance > 0 else 0.0

        total_distance += distance
        route_times.append(time_calc)
        route_efficiencies.append(efficiency)

        print(
            f"   Route {i+1:02d}: {cells_count:2d} cells, {distance:6.1f}m, "
            f"{time_calc:5.1f}s, {efficiency:.4f} cells/m"
        )

    print("-" * 80)
    print(f"   Summary: {len(routes)} routes, {total_cells} cells total")
    print(
        f"   Balance: {min(len(r.cell_sequence) for r in routes)}-"
        f"{max(len(r.cell_sequence) for r in routes)} cells/route"
    )
    print(
        f"   Avg route: {total_cells/len(routes):.1f} cells, "
        f"{total_distance/len(routes):.1f}m, {sum(route_times)/len(routes):.1f}s"
    )
    print(f"   Max loop time: {max(route_times):.1f}s")
    print(
        f"   Efficiency range: {min(route_efficiencies):.4f} - "
        f"{max(route_efficiencies):.4f} cells/m"
    )
    print("=" * 80)


def log_cell_coverage_gaps(
    cells: Sequence[Cell],
    current_time: float,
    threshold: float = 120.0,
    filename: str | None = None,
):
    """Analyze and log cells with coverage gaps."""
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")  # noqa: DTZ005
        filename = f"results/coverage_gaps_{timestamp}.csv"

    # Ensure simulations directory exists
    os.makedirs("results", exist_ok=True)

    overdue_cells = []
    gap_analysis = {}

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "cell_id",
                "x",
                "y",
                "last_observed",
                "age",
                "overdue",
                "grid_row",
                "grid_col",
            ]
        )

        for cell in cells:
            # Handle both possible attribute names for backwards compatibility
            last_seen = getattr(cell, "last_seen_ts", None) or getattr(
                cell, "last_observed", None
            )
            if last_seen is not None and last_seen >= 0:
                age = current_time - last_seen
            else:
                age = current_time  # Never observed
            overdue = age > threshold

            if overdue:
                overdue_cells.append(cell)

            # Extract grid position from cell ID (assuming format like "cell_003_007")
            parts = cell.id.split("_")
            grid_row = int(parts[1]) if len(parts) > 1 else -1
            grid_col = int(parts[2]) if len(parts) > 2 else -1

            writer.writerow(
                [cell.id, cell.x, cell.y, last_seen, age, overdue, grid_row, grid_col]
            )

            # Spatial analysis
            region_key = f"r{grid_row//3}_c{grid_col//3}"  # 3x3 grid regions
            if region_key not in gap_analysis:
                gap_analysis[region_key] = {"total": 0, "overdue": 0}
            gap_analysis[region_key]["total"] += 1
            if overdue:
                gap_analysis[region_key]["overdue"] += 1

    print(f"\n🗺️  Coverage Gap Analysis (t={current_time:.1f}s)")
    print("=" * 60)
    print(f"   Total cells: {len(cells)}")
    print(
        f"   Overdue cells: {len(overdue_cells)} "
        f"({len(overdue_cells)/len(cells)*100:.1f}%)"
    )
    print(f"   Coverage threshold: {threshold}s")

    if overdue_cells:
        print("\n   📍 Overdue Cell Locations:")
        for cell in sorted(
            overdue_cells,
            key=lambda c: current_time - (getattr(c, "last_seen_ts", None) or 0),
            reverse=True,
        )[:10]:
            last_seen = getattr(cell, "last_seen_ts", None) or getattr(
                cell, "last_observed", None
            )
            if last_seen is not None and last_seen >= 0:
                age = current_time - last_seen
            else:
                age = current_time
            print(f"      {cell.id}: ({cell.x:6.1f}, {cell.y:6.1f}), age: {age:6.1f}s")
        if len(overdue_cells) > 10:
            print(f"      ... and {len(overdue_cells) - 10} more")

    print("\n   🗺️  Regional Coverage Gaps:")
    for region, stats in sorted(gap_analysis.items()):
        if stats["overdue"] > 0:
            pct = stats["overdue"] / stats["total"] * 100
            print(
                f"      {region}: {stats['overdue']}/{stats['total']} "
                f"overdue ({pct:.1f}%)"
            )

    print(f"\n   📄 Detailed log saved: {filename}")
    print("=" * 60)

    return filename, overdue_cells


def compare_algorithm_performance(results: dict[str, dict[str, Any]]):
    """Compare performance metrics across different algorithms."""
    print("\n🏆 Algorithm Performance Comparison")
    print("=" * 80)
    print(
        f"{'Algorithm':<12} {'Peak':<8} {'Rolling':<8} {'Min':<8} "
        f"{'Overdue':<8} {'Loop Time':<10}"
    )
    print("-" * 80)

    for algo_name, metrics in sorted(results.items()):
        peak = metrics.get("peak_coverage", 0)
        rolling = metrics.get("rolling_avg", 0)
        min_cov = metrics.get("min_coverage", 0)
        overdue = metrics.get("avg_overdue", 0)
        loop_time = metrics.get("max_loop_time", 0)

        print(
            f"{algo_name:<12} {peak:<7.1f}% {rolling:<7.1f}% {min_cov:<7.1f}% "
            f"{overdue:<7.1f} {loop_time:<9.1f}s"
        )

    print("=" * 80)


def track_uav_routes(uavs, current_time: float, filename: str | None = None):
    """Track individual UAV route progress and assignments."""
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")  # noqa: DTZ005
        filename = f"results/uav_routes_{timestamp}.csv"

    # Ensure simulations directory exists
    os.makedirs("results", exist_ok=True)

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "time",
                "uav_id",
                "state",
                "x",
                "y",
                "soc",
                "route_id",
                "waypoint_idx",
                "current_cell",
                "remaining_cells",
                "progress_pct",
            ]
        )

        for uav in uavs:
            # Get current route (first in route_list if available)
            current_route = uav.route_list[0] if uav.route_list else None
            waypoint_idx = getattr(uav, "_waypoint_idx", 0)

            if current_route and current_route.cell_sequence:
                remaining = len(current_route.cell_sequence) - waypoint_idx
                progress = waypoint_idx / len(current_route.cell_sequence) * 100
                current_cell = (
                    current_route.cell_sequence[waypoint_idx]
                    if waypoint_idx < len(current_route.cell_sequence)
                    else "None"
                )
                route_id = current_route.id
            else:
                remaining = 0
                progress = 0
                current_cell = "None"
                route_id = "None"

            writer.writerow(
                [
                    current_time,
                    uav.id,
                    uav.state.value,
                    uav.x,
                    uav.y,
                    uav.soc,
                    route_id,
                    waypoint_idx,
                    current_cell,
                    remaining,
                    progress,
                ]
            )

    print(f"📋 UAV route tracking saved: {filename}")
    return filename


def analyze_spare_utilization(simulation_logs: list[str], filename: str | None = None):
    """Analyze spare launch patterns and timing from simulation logs."""
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")  # noqa: DTZ005
        filename = f"results/spare_analysis_{timestamp}.csv"

    spare_launches = []
    skipped_launches = []

    # Parse simulation logs for spare events
    for log_line in simulation_logs:
        if "🚀 Launching spare" in log_line:
            # Extract spare launch details
            parts = log_line.split()
            spare_id = parts[2] if len(parts) > 2 else "unknown"
            origin_id = parts[4] if len(parts) > 4 else "unknown"
            spare_launches.append(
                {"spare_id": spare_id, "origin_id": origin_id, "event": "launch"}
            )
        elif "⚠️  Route too short for spare" in log_line:
            # Extract skipped launch details
            parts = log_line.split()
            spare_id = parts[5] if len(parts) > 5 else "unknown"
            skipped_launches.append({"spare_id": spare_id, "reason": "route_too_short"})
        elif "⚠️  No spares available" in log_line:
            # Extract no-spare-available events
            origin_id = parts[-1] if len(parts) > 0 else "unknown"
            skipped_launches.append({"origin_id": origin_id, "reason": "no_spares"})

    # Save analysis
    os.makedirs("results", exist_ok=True)
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["event_type", "spare_id", "origin_id", "reason"])

        for launch in spare_launches:
            writer.writerow(["launch", launch["spare_id"], launch["origin_id"], ""])

        for skip in skipped_launches:
            writer.writerow(
                [
                    "skip",
                    skip.get("spare_id", ""),
                    skip.get("origin_id", ""),
                    skip["reason"],
                ]
            )

    print("\n🔄 Spare Utilization Analysis")
    print(f"   Successful launches: {len(spare_launches)}")
    print(f"   Skipped launches: {len(skipped_launches)}")
    print(f"   Analysis saved: {filename}")

    return filename
