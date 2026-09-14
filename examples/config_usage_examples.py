"""Configuration system usage examples.

This script demonstrates how to use the centralized configuration system
for different mission scenarios, parameter studies, and integration with
existing modules.
"""

import sys
from pathlib import Path

# Add the uav_surveil package to the path
sys.path.insert(0, str(Path(__file__).parent.parent))

from uav_surveil.config import (
    compare_scenarios,
    create_parameter_sweep,
    list_available_scenarios,
    load_scenario,
)
from uav_surveil.config.config_manager import ConfigManager
from uav_surveil.stage0_battery import (
    get_max_grid_from_config,
    optimize_battery_from_config,
    validate_mission_feasibility,
)


def example_1_basic_usage():
    """Example 1: Basic configuration loading and usage."""
    print("=" * 60)
    print("EXAMPLE 1: Basic Configuration Usage")
    print("=" * 60)

    # List available scenarios
    print("Available scenarios:", list_available_scenarios())

    # Load baseline configuration
    config = load_scenario("baseline")
    print(f"\nLoaded configuration: {config.config_name}")
    print(f"Description: {config.description}")

    # Print summary
    print(config.summary())

    # Validate configuration consistency
    warnings = config.validate_consistency()
    if warnings:
        print("\nConfiguration Warnings:")
        for warning in warnings:
            print(f"  • {warning}")
    else:
        print("\n✅ Configuration is consistent!")


def example_2_scenario_comparison():
    """Example 2: Compare different scenarios."""
    print("\n" + "=" * 60)
    print("EXAMPLE 2: Scenario Comparison")
    print("=" * 60)

    # Compare baseline vs urban scenarios
    differences = compare_scenarios("baseline", "urban")

    print("Differences between 'baseline' and 'urban' scenarios:")
    for param_path, values in differences.items():
        print(f"  {param_path}:")
        print(f"    baseline: {values['baseline']}")
        print(f"    urban: {values['urban']}")


def example_3_stage0_integration():
    """Example 3: Integration with Stage 0 battery analysis."""
    print("\n" + "=" * 60)
    print("EXAMPLE 3: Stage 0 Battery Analysis Integration")
    print("=" * 60)

    config = load_scenario("baseline")

    # Estimate grid patrol distance (this would come from Stage 1)
    l_grid = 2000.0  # meters

    # Run battery optimization using config
    result = optimize_battery_from_config(config, l_grid)

    print("Battery optimization results:")
    print(f"  Required reserve (ξ): {result.xi_optimal:.3f} ({result.xi_optimal:.1%})")
    print(f"  Mission feasible: {result.is_feasible}")
    print(f"  Battery utilization: {result.utilization:.1%}")
    print(f"  Time margin: {result.margin_seconds:.1f} seconds")
    print(f"  Distance margin: {result.margin_distance:.1f} meters")

    # Check feasibility with SoC floor
    is_feasible = validate_mission_feasibility(config, l_grid)
    print(
        f"  SoC floor validation: {'✅ Feasible' if is_feasible else '❌ Not feasible'}"
    )

    # Calculate maximum possible grid distance
    max_grid = get_max_grid_from_config(config)
    print(f"  Maximum grid distance: {max_grid:.1f} meters")


def example_4_parameter_studies():
    """Example 4: Parameter sensitivity studies."""
    print("\n" + "=" * 60)
    print("EXAMPLE 4: Parameter Sensitivity Studies")
    print("=" * 60)

    # Create parameter sweep for SoC floor values
    soc_values = [0.10, 0.15, 0.20, 0.25]
    configs = create_parameter_sweep(
        "baseline", "battery.soc_floor", soc_values, "soc_floor_study"
    )

    print(f"Created {len(configs)} configurations for SoC floor study:")

    l_grid = 2000.0  # Fixed grid distance for comparison

    for config in configs:
        result = optimize_battery_from_config(config, l_grid)
        soc_floor = config.battery.soc_floor
        print(
            f"  SoC floor {soc_floor:.1%}: ξ={result.xi_optimal:.3f}, "
            f"feasible={result.is_feasible}, margin={result.margin_seconds:.1f}s"
        )


def example_5_config_manager():
    """Example 5: Using ConfigManager for advanced operations."""
    print("\n" + "=" * 60)
    print("EXAMPLE 5: Advanced ConfigManager Usage")
    print("=" * 60)

    # Create config manager
    manager = ConfigManager()

    # Load configuration
    config = manager.load_config("rural")
    print(f"Loaded: {config.config_name}")

    # Save configuration
    saved_path = manager.save_config()
    print(f"Saved configuration to: {saved_path}")

    # Create multi-parameter study
    studies = {"battery.soc_floor": [0.1, 0.15, 0.2], "grid.cell_size": [30, 40, 50]}
    study_configs = manager.create_study_configs(
        "baseline", studies, "multi_param_study"
    )
    print(f"Created {len(study_configs)} configurations for multi-parameter study")

    # Print validation summary
    manager.print_config_summary()


def example_6_custom_configuration():
    """Example 6: Creating and modifying custom configurations."""
    print("\n" + "=" * 60)
    print("EXAMPLE 6: Custom Configuration Creation")
    print("=" * 60)

    # Start with baseline and modify
    config = load_scenario("baseline")
    config.config_name = "custom_mission"
    config.description = "Custom mission configuration"

    # Modify parameters for specific mission
    config.mission.area_width = 750.0
    config.mission.area_length = 600.0
    config.mission.max_ferry_distance = 400.0

    config.battery.total_endurance = 2400.0  # 40 minutes
    config.battery.usable_endurance = 2160.0

    config.grid.cell_size = 35.0
    config.stl.max_revisit_gap = 100.0

    print("Created custom configuration:")
    print(config.summary())

    # Save custom config
    manager = ConfigManager()
    manager.load_config(config)
    saved_path = manager.save_config(filename="custom_mission.json")
    print(f"\nSaved custom configuration to: {saved_path}")


def example_7_batch_analysis():
    """Example 7: Batch analysis across multiple scenarios."""
    print("\n" + "=" * 60)
    print("EXAMPLE 7: Batch Analysis Across Scenarios")
    print("=" * 60)

    scenarios = ["baseline", "urban", "rural", "test"]
    l_grid = 1500.0  # Common grid distance

    print(f"Battery analysis for grid distance {l_grid}m:")
    print(
        f"{'Scenario':<12} {'ξ_optimal':<10} {'Feasible':<10} {'Margin (s)':<12} {'Max Grid (m)':<12}"
    )
    print("-" * 60)

    for scenario_name in scenarios:
        try:
            config = load_scenario(scenario_name)
            result = optimize_battery_from_config(config, l_grid)
            max_grid = get_max_grid_from_config(config)

            feasible_str = "✅ Yes" if result.is_feasible else "❌ No"
            print(
                f"{scenario_name:<12} {result.xi_optimal:<10.3f} {feasible_str:<10} "
                f"{result.margin_seconds:<12.1f} {max_grid:<12.1f}"
            )
        except Exception as e:  # noqa: BLE001 - per-scenario error report, must not abort the loop
            print(f"{scenario_name:<12} ERROR: {e!s}")


if __name__ == "__main__":
    """Run all configuration examples."""
    print("UAV Surveillance Configuration System Examples")
    print("=" * 60)

    try:
        example_1_basic_usage()
        example_2_scenario_comparison()
        example_3_stage0_integration()
        example_4_parameter_studies()
        example_5_config_manager()
        example_6_custom_configuration()
        example_7_batch_analysis()

        print("\n" + "=" * 60)
        print("✅ All examples completed successfully!")
        print("=" * 60)

    except Exception as e:  # noqa: BLE001 - top-level script guard, reports full traceback
        print(f"\n❌ Error running examples: {e}")
        import traceback

        traceback.print_exc()
