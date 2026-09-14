"""Configuration management for UAV surveillance system."""

from .config_manager import (
    ConfigManager,
    get_config_manager,
    get_global_config,
    load_global_config,
)
from .parameters import (
    BatteryParameters,
    GridParameters,
    MissionParameters,
    OptimizationParameters,
    SimulationParameters,
    STLParameters,
    SystemParameters,
    UAVParameters,
)
from .scenarios import (
    compare_scenarios,
    create_parameter_sweep,
    get_baseline_config,
    get_battery_study_config,
    get_performance_test_config,
    get_rural_config,
    get_test_config,
    get_urban_config,
    list_available_scenarios,
    load_scenario,
)

__all__ = [  # noqa: RUF022 - grouped by category, not alphabetical
    # Parameter classes
    "SystemParameters",
    "MissionParameters",
    "UAVParameters",
    "BatteryParameters",
    "GridParameters",
    "OptimizationParameters",
    "STLParameters",
    "SimulationParameters",
    # Scenario functions
    "load_scenario",
    "get_baseline_config",
    "get_urban_config",
    "get_rural_config",
    "get_test_config",
    "get_performance_test_config",
    "get_battery_study_config",
    "list_available_scenarios",
    "create_parameter_sweep",
    "compare_scenarios",
    # Config manager
    "ConfigManager",
    "get_config_manager",
    "load_global_config",
    "get_global_config",
]
