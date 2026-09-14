"""Stage 0: Battery Feasibility Checker.

This module implements the battery constraint validation for UAV operations.
It ensures that the furthest ferry distance plus grid patrol plus return
is within the single battery endurance limit.
"""

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .config import SystemParameters


@dataclass
class BatteryOptimizationResult:
    """Result of battery reserve deficit analysis.

    Attributes:
        xi_optimal: Battery deficit fraction (0 = feasible, >0 = shortfall
            beyond the usable battery budget, per architecture.md §3)
        is_feasible: Whether mission is feasible within constraints
        margin_seconds: Time margin in seconds (positive = safe, negative = deficit)
        margin_distance: Distance margin in meters
        utilization: Fraction of usable battery distance the mission needs
    """

    xi_optimal: float
    is_feasible: bool
    margin_seconds: float
    margin_distance: float
    utilization: float


def optimize_battery_reserve(
    d_ferry: float,
    l_grid: float,
    v_max: float,
    endurance: float,
    xi_max: float = 0.5,
    soc_floor: float = 0.1,
) -> BatteryOptimizationResult:
    """
    Compute the battery deficit slack ξ for a mission (architecture.md §3 /
    thesis Alg. 2).

        D_usable = v_max * endurance * (1 - soc_floor)
        D_need   = 2*d_ferry + l_grid
        feasible ⇔ D_need ≤ D_usable
        ξ = max(0, D_need / (v_max * endurance) - (1 - soc_floor))

    ξ is a *deficit*: it is 0 whenever the mission is feasible, and grows
    above 0 by however much the mission overruns the usable battery budget
    when infeasible.

    Args:
        d_ferry: Maximum ferry distance from depot to grid edge (meters)
        l_grid: Total surveillance grid patrol distance (meters)
        v_max: Maximum cruise velocity (meters/second)
        endurance: Total battery endurance (seconds)
        xi_max: Maximum allowed battery reserve fraction (default 0.5 = 50%)
        soc_floor: Minimum allowed battery reserve (default 0.1 = 10%)

    Returns:
        BatteryOptimizationResult with deficit ξ and feasibility analysis

    Raises:
        ValueError: If parameters are invalid
    """
    # Parameter validation
    if d_ferry < 0:
        raise ValueError(f"Ferry distance must be non-negative, got {d_ferry}")
    if l_grid < 0:
        raise ValueError(f"Grid patrol distance must be non-negative, got {l_grid}")
    if v_max <= 0:
        raise ValueError(f"Max velocity must be positive, got {v_max}")
    if endurance <= 0:
        raise ValueError(f"Endurance must be positive, got {endurance}")
    if not 0.0 <= xi_max <= 1.0:
        raise ValueError(f"xi_max must be in [0, 1], got {xi_max}")
    if not 0.0 <= soc_floor <= 1.0:
        raise ValueError(f"soc_floor must be in [0, 1], got {soc_floor}")

    d_need = 2 * d_ferry + l_grid
    max_distance = v_max * endurance
    d_usable = max_distance * (1 - soc_floor)

    is_feasible = d_need <= d_usable
    xi_optimal = max(0.0, d_need / max_distance - (1 - soc_floor))
    utilization = d_need / d_usable if d_usable > 0 else 1.0

    margin_distance = d_usable - d_need
    margin_seconds = margin_distance / v_max if v_max > 0 else 0.0

    return BatteryOptimizationResult(
        xi_optimal=xi_optimal,
        is_feasible=is_feasible,
        margin_seconds=margin_seconds,
        margin_distance=margin_distance,
        utilization=utilization,
    )


def battery_feasible(
    d_ferry: float,
    l_grid: float,
    v_max: float,
    endurance: float,
    soc_floor: float = 0.1,
) -> bool:
    """Check if mission is feasible within single battery constraints.

    This is the FEASIBILITY VALIDATOR approach - checks if a mission
    can be completed with a predetermined battery reserve.

    Validates the fundamental battery constraint from the reference:
    2*d_ferry + l_grid <= v_max * endurance * (1 - soc_floor)

    This ensures UAV can:
    1. Ferry from depot to grid edge (d_ferry)
    2. Complete surveillance pattern (l_grid)
    3. Return to depot (d_ferry)
    4. Maintain SoC above safety floor (soc_floor)

    Args:
        d_ferry: Maximum ferry distance from depot to grid edge (m)
        l_grid: Total surveillance grid patrol distance (m)
        v_max: Maximum cruise velocity (m/s)
        endurance: Total battery endurance in seconds
        soc_floor: Minimum SoC safety floor (default 0.1 = 10%)

    Returns:
        True if mission is battery-feasible

    Raises:
        ValueError: If parameters are invalid or mission is infeasible
    """
    # Parameter validation
    if d_ferry < 0:
        raise ValueError(f"Ferry distance must be non-negative, got {d_ferry}")
    if l_grid < 0:
        raise ValueError(f"Grid patrol distance must be non-negative, got {l_grid}")
    if v_max <= 0:
        raise ValueError(f"Max velocity must be positive, got {v_max}")
    if endurance <= 0:
        raise ValueError(f"Endurance must be positive, got {endurance}")
    if not 0.0 <= soc_floor < 1.0:
        raise ValueError(f"SoC floor must be in [0, 1), got {soc_floor}")
    if soc_floor < 0.1:
        raise ValueError(f"SoC floor must be >= 0.1 for safety, got {soc_floor}")

    # Calculate total mission distance
    total_distance = 2 * d_ferry + l_grid  # Ferry out + patrol + ferry back

    # Calculate available flight distance with SoC reserve
    available_distance = v_max * endurance * (1.0 - soc_floor)

    # Check feasibility constraint
    if total_distance > available_distance:
        raise ValueError(
            f"Mission infeasible: total distance {total_distance:.1f}m "
            f"exceeds available {available_distance:.1f}m "
            f"(ferry: {d_ferry:.1f}m, grid: {l_grid:.1f}m, "
            f"v_max: {v_max:.1f}m/s, endurance: {endurance:.1f}s, "
            f"soc_floor: {soc_floor:.1%})"
        )

    return True


def calculate_max_grid_distance(
    d_ferry: float, v_max: float, endurance: float, soc_floor: float = 0.1
) -> float:
    """Calculate maximum feasible grid patrol distance.

    Given ferry distance and battery constraints, calculates the maximum
    surveillance grid distance that can be covered in a single battery cycle.

    Args:
        d_ferry: Ferry distance from depot to grid edge (m)
        v_max: Maximum cruise velocity (m/s)
        endurance: Total battery endurance in seconds
        soc_floor: Minimum SoC safety floor (default 0.1)

    Returns:
        Maximum feasible grid patrol distance (m)

    Raises:
        ValueError: If parameters are invalid or no grid patrol is possible
    """
    # Parameter validation (reuse from battery_feasible)
    if d_ferry < 0:
        raise ValueError(f"Ferry distance must be non-negative, got {d_ferry}")
    if v_max <= 0:
        raise ValueError(f"Max velocity must be positive, got {v_max}")
    if endurance <= 0:
        raise ValueError(f"Endurance must be positive, got {endurance}")
    if not 0.0 <= soc_floor < 1.0:
        raise ValueError(f"SoC floor must be in [0, 1), got {soc_floor}")
    if soc_floor < 0.1:
        raise ValueError(f"SoC floor must be >= 0.1 for safety, got {soc_floor}")

    # Calculate available distance budget
    available_distance = v_max * endurance * (1.0 - soc_floor)

    # Subtract ferry distances (out and back)
    grid_distance = available_distance - 2 * d_ferry

    if grid_distance <= 0:
        raise ValueError(
            f"No grid patrol possible: ferry distances {2*d_ferry:.1f}m "
            f"exceed available {available_distance:.1f}m"
        )

    return grid_distance


def estimate_mission_time(d_ferry: float, l_grid: float, v_max: float) -> float:
    """Estimate total mission time for given distances.

    Args:
        d_ferry: Ferry distance (m)
        l_grid: Grid patrol distance (m)
        v_max: Cruise velocity (m/s)

    Returns:
        Estimated mission time (seconds)
    """
    total_distance = 2 * d_ferry + l_grid
    return total_distance / v_max


def analyze_battery_margin(
    d_ferry: float,
    l_grid: float,
    v_max: float,
    endurance: float,
    target_xi: float = 0.1,
    soc_floor: float = 0.1,
) -> tuple[float, float, bool]:
    """
    Analyze battery margin for mission planning.

    Uses the same deficit-slack convention as `optimize_battery_reserve`:
    ``required_xi`` is 0 when the mission fits the usable battery budget and
    grows above 0 by the shortfall when it does not.

    Args:
        d_ferry: Ferry distance from depot to grid edge (meters).
        l_grid: Total grid patrol distance (meters).
        v_max: Cruise velocity of UAV (meters/second).
        endurance: Total battery endurance (seconds).
        target_xi: Maximum tolerated deficit fraction (default 0.1 = 10%).
        soc_floor: Minimum allowed battery reserve (default 0.1 = 10%).

    Returns:
        required_xi: The battery deficit fraction (0 = feasible).
        margin_seconds: Time margin (in seconds) between usable battery and required mission duration.
        is_safe: True if the mission is feasible and its deficit is within target_xi.
    """
    d_need = 2 * d_ferry + l_grid
    max_distance = v_max * endurance
    d_usable = max_distance * (1 - soc_floor)

    required_xi = max(0.0, d_need / max_distance - (1 - soc_floor))
    margin_distance = d_usable - d_need
    margin_seconds = margin_distance / v_max if v_max > 0 else 0.0
    is_safe = (d_need <= d_usable) and (required_xi <= target_xi)

    return required_xi, margin_seconds, is_safe


# Configuration-based convenience functions


def optimize_battery_from_config(
    config: "SystemParameters", l_grid: float
) -> BatteryOptimizationResult:
    """Optimize battery reserve using system configuration.

    Args:
        config: System configuration with all parameters
        l_grid: Grid patrol distance (calculated by Stage 1)

    Returns:
        Battery optimization result
    """
    params = config.get_battery_constraint_params()
    return optimize_battery_reserve(
        d_ferry=params["d_ferry"],
        l_grid=l_grid,
        v_max=params["v_max"],
        endurance=params["endurance"],
        xi_max=params["xi_max"],
        soc_floor=params["soc_floor"],
    )


def validate_mission_feasibility(config: "SystemParameters", l_grid: float) -> bool:
    """Validate mission feasibility using system configuration.

    Args:
        config: System configuration
        l_grid: Grid patrol distance (calculated by Stage 1)

    Returns:
        True if mission is feasible
    """
    params = config.get_battery_constraint_params()
    return battery_feasible(
        d_ferry=params["d_ferry"],
        l_grid=l_grid,
        v_max=params["v_max"],
        endurance=params["endurance"],
        soc_floor=params["soc_floor"],
    )


def get_max_grid_from_config(config: "SystemParameters") -> float:
    """Calculate maximum grid patrol distance from configuration.

    Args:
        config: System configuration

    Returns:
        Maximum feasible grid patrol distance (m)
    """
    params = config.get_battery_constraint_params()
    return calculate_max_grid_distance(
        d_ferry=params["d_ferry"],
        v_max=params["v_max"],
        endurance=params["endurance"],
        soc_floor=params["soc_floor"],
    )


def analyze_mission_from_config(
    config: "SystemParameters", l_grid: float
) -> BatteryOptimizationResult:
    """Complete battery analysis from configuration.

    Args:
        config: System configuration
        l_grid: Grid patrol distance

    Returns:
        Complete battery analysis result with feasibility and margins
    """
    return optimize_battery_from_config(config, l_grid)
