"""Predefined scenario configurations for different mission types.

This module provides common parameter sets for various surveillance scenarios,
enabling quick setup for different research experiments and mission types.
"""

from typing import Dict, List
from .parameters import (
    SystemParameters,
    MissionParameters,
    UAVParameters,
    BatteryParameters,
)
import copy
from pathlib import Path
import json


def get_baseline_config() -> SystemParameters:
    """Get the baseline configuration from your thesis reference.

    This matches your exact parameters:
    - 500m × 500m surveillance area
    - 500m max ferry distance
    - 4 m/s cruise speed
    - 2100s (35 min) battery endurance
    - 10% SoC safety floor
    """
    return SystemParameters(
        config_name="baseline",
        description="Baseline configuration from thesis reference document",
        version="1.0",
        # Uses all default values which match your specifications
    )


def get_urban_config() -> SystemParameters:
    """Urban surveillance scenario configuration.

    Characteristics:
    - Smaller area with higher cell density
    - Shorter ferry distances
    - Higher priority zones around key infrastructure
    - More conservative battery management
    """
    config = get_baseline_config()
    config.config_name = "urban"
    config.description = "Urban surveillance with high-density coverage"

    # Smaller, denser urban area
    config.mission.area_width = 300.0
    config.mission.area_length = 300.0
    config.mission.max_ferry_distance = 200.0

    # Higher resolution grid
    config.grid.cell_size = 25.0

    # Priority zones for urban infrastructure
    config.grid.priority_zones = {
        "city_center": 2.0,
        "transport_hubs": 1.5,
        "industrial": 1.2,
    }

    # More conservative battery management in urban environment
    config.battery.soc_floor = 0.15
    config.battery.soc_return_threshold = 0.25

    # Tighter STL contracts for urban response times
    config.stl.max_revisit_gap = 90.0
    config.stl.spare_launch_deadline = 0.5

    return config


def get_rural_config() -> SystemParameters:
    """Rural surveillance scenario configuration.

    Characteristics:
    - Large area with coarser grid
    - Long ferry distances
    - Relaxed timing constraints
    - Extended battery optimization
    """
    config = get_baseline_config()
    config.config_name = "rural"
    config.description = "Rural surveillance with extended coverage area"

    # Large rural area
    config.mission.area_width = 1000.0
    config.mission.area_length = 800.0
    config.mission.max_ferry_distance = 800.0
    config.mission.mission_duration = 7200.0  # 2 hours

    # Coarser grid for rural coverage
    config.grid.cell_size = 60.0

    # Extended battery endurance for rural operations
    config.battery.total_endurance = 3600.0  # 1 hour
    config.battery.usable_endurance = 3240.0

    # Relaxed STL contracts for rural environment
    config.stl.max_revisit_gap = 180.0
    config.stl.spare_launch_deadline = 2.0

    # Higher fleet budget for rural operations
    config.optimization.max_fleet_budget = 2000000.0
    config.optimization.spare_floor_ratio = 0.3

    return config


def get_test_config() -> SystemParameters:
    """Minimal test configuration for development and debugging.

    Characteristics:
    - Very small area for quick testing
    - Short mission duration
    - Relaxed constraints
    - Fast simulation parameters
    """
    config = get_baseline_config()
    config.config_name = "test"
    config.description = "Minimal configuration for testing and development"

    # Tiny test area
    config.mission.area_width = 100.0
    config.mission.area_length = 100.0
    config.mission.max_ferry_distance = 50.0
    config.mission.mission_duration = 300.0  # 5 minutes

    # Coarse grid for fast testing
    config.grid.cell_size = 50.0

    # Fast simulation
    config.simulation.time_step = 0.5
    config.simulation.monte_carlo_runs = 10
    config.simulation.real_time_factor = 10.0

    # Relaxed optimization limits
    config.optimization.route_optimization_time_limit = 5.0
    config.optimization.alns_iterations = 100

    return config


def get_performance_test_config() -> SystemParameters:
    """High-performance scenario for stress testing the system.

    Characteristics:
    - Large area with fine grid
    - Tight timing constraints
    - High fleet requirements
    - Challenging optimization
    """
    config = get_baseline_config()
    config.config_name = "performance_test"
    config.description = "High-performance scenario for system stress testing"

    # Large, high-resolution area
    config.mission.area_width = 2000.0
    config.mission.area_length = 1500.0
    config.mission.max_ferry_distance = 1000.0
    config.mission.mission_duration = 14400.0  # 4 hours

    # Fine grid resolution
    config.grid.cell_size = 30.0

    # Tight STL contracts
    config.stl.max_revisit_gap = 60.0
    config.stl.spare_launch_deadline = 0.3

    # High-budget fleet
    config.optimization.max_fleet_budget = 5000000.0
    config.optimization.spare_floor_ratio = 0.4

    # Extended optimization time
    config.optimization.route_optimization_time_limit = 60.0
    config.optimization.alns_iterations = 5000

    # High Monte Carlo runs for thorough V&V
    config.simulation.monte_carlo_runs = 5000

    return config


def get_battery_study_config() -> SystemParameters:
    """Configuration optimized for battery constraint studies.

    Characteristics:
    - Various battery endurance scenarios
    - Conservative SoC management
    - Focus on Stage 0 optimization
    """
    config = get_baseline_config()
    config.config_name = "battery_study"
    config.description = "Configuration for battery constraint analysis"

    # Standard area but challenging ferry requirements
    config.mission.max_ferry_distance = 600.0

    # Conservative battery parameters
    config.battery.soc_floor = 0.12
    config.battery.soc_return_threshold = 0.25
    config.optimization.max_battery_reserve = 0.15

    return config


# Scenario registry
_SCENARIOS: Dict[str, callable] = {
    "baseline": get_baseline_config,
    "urban": get_urban_config,
    "rural": get_rural_config,
    "test": get_test_config,
    "performance_test": get_performance_test_config,
    "battery_study": get_battery_study_config,
}


def load_scenario(scenario_name: str) -> SystemParameters:
    """Load a predefined scenario configuration.

    Args:
        scenario_name: Name of the scenario to load

    Returns:
        SystemParameters instance for the requested scenario

    Raises:
        ValueError: If scenario_name is not found
    """
    configs_dir = Path(__file__).resolve().parent.parent.parent / "configs"
    # Prefer an explicit <name>.json file
    json_file = configs_dir / f"{scenario_name}.json"
    if not json_file.exists():
        # Fallback: pick highest version <name>_v*.json if any
        candidates = sorted(configs_dir.glob(f"{scenario_name}_v*.json"))
        if candidates:
            json_file = candidates[-1]
    if json_file.exists():
        try:
            with open(json_file, "r") as f:
                data = json.load(f)
            return SystemParameters(**data)
        except Exception as e:
            raise ValueError(
                f"Failed to load scenario JSON '{json_file.name}': {e}"
            ) from e
    # Fallback to Python registry
    if scenario_name not in _SCENARIOS:
        available = ", ".join(
            list(_SCENARIOS.keys()) + [p.stem for p in configs_dir.glob("*.json")]
        )
        raise ValueError(f"Unknown scenario '{scenario_name}'. Available: {available}")
    return _SCENARIOS[scenario_name]()


def list_available_scenarios() -> List[str]:
    """Get list of available predefined scenarios.

    Returns:
        List of scenario names
    """
    return list(_SCENARIOS.keys())


def create_parameter_sweep(
    base_scenario: str,
    parameter_path: str,
    values: List[float],
    sweep_name: str = "parameter_sweep",
) -> List[SystemParameters]:
    """Create a parameter sweep for sensitivity analysis.

    Args:
        base_scenario: Base scenario to modify
        parameter_path: Dot-notation path to parameter (e.g., "battery.soc_floor")
        values: List of values to sweep over
        sweep_name: Name prefix for generated configurations

    Returns:
        List of configurations with swept parameter values

    Example:
        >>> configs = create_parameter_sweep(
        ...     "baseline",
        ...     "battery.soc_floor",
        ...     [0.1, 0.15, 0.2, 0.25]
        ... )
    """
    base_config = load_scenario(base_scenario)
    configurations = []

    for i, value in enumerate(values):
        config = copy.deepcopy(base_config)
        config.config_name = f"{sweep_name}_{i:03d}"
        config.description = f"Parameter sweep: {parameter_path} = {value}"

        # Navigate to the parameter using dot notation
        obj = config
        parts = parameter_path.split(".")
        for part in parts[:-1]:
            obj = getattr(obj, part)
        setattr(obj, parts[-1], value)

        configurations.append(config)

    return configurations


def compare_scenarios(scenario1: str, scenario2: str) -> Dict[str, any]:
    """Compare two scenarios and highlight differences.

    Args:
        scenario1: First scenario name
        scenario2: Second scenario name

    Returns:
        Dictionary showing parameter differences
    """
    config1 = load_scenario(scenario1)
    config2 = load_scenario(scenario2)

    def find_differences(obj1, obj2, path=""):
        differences = {}

        # Compare all fields
        for field_name in obj1.model_fields:
            val1 = getattr(obj1, field_name)
            val2 = getattr(obj2, field_name)

            current_path = f"{path}.{field_name}" if path else field_name

            if hasattr(val1, "model_fields"):  # Nested Pydantic model
                nested_diffs = find_differences(val1, val2, current_path)
                differences.update(nested_diffs)
            elif val1 != val2:
                differences[current_path] = {scenario1: val1, scenario2: val2}

        return differences

    return find_differences(config1, config2)
