"""Configuration manager for easy parameter access and scenario management.

This module provides convenient methods for loading configurations,
running parameter studies, and integrating with existing modules.
"""

from datetime import datetime
from pathlib import Path

from .parameters import SystemParameters
from .scenarios import create_parameter_sweep, list_available_scenarios, load_scenario


class ConfigManager:
    """Centralized configuration management for UAV surveillance system."""

    def __init__(self, config_dir: Path | None = None):
        """Initialize configuration manager.

        Args:
            config_dir: Directory for storing configuration files
        """
        self.config_dir = config_dir or Path("configs")
        self.config_dir.mkdir(exist_ok=True)
        self._current_config: SystemParameters | None = None

    def load_config(self, source: str | Path | SystemParameters) -> SystemParameters:
        """Load configuration from various sources.

        Args:
            source: Configuration source - can be:
                    - Scenario name (str): "baseline", "urban", etc.
                    - File path (Path): path to JSON config file
                    - SystemParameters: existing config object

        Returns:
            Loaded SystemParameters configuration
        """
        if isinstance(source, SystemParameters):
            self._current_config = source
        elif isinstance(source, str):
            # Try to load as scenario first
            try:
                self._current_config = load_scenario(source)
            except ValueError:
                # If not a scenario, try as file path
                self._current_config = SystemParameters.load_from_file(Path(source))
        elif isinstance(source, Path):
            self._current_config = SystemParameters.load_from_file(source)
        else:
            raise TypeError(f"Unsupported config source type: {type(source)}")

        return self._current_config

    def get_current_config(self) -> SystemParameters:
        """Get the currently loaded configuration.

        Returns:
            Current SystemParameters configuration

        Raises:
            RuntimeError: If no configuration is loaded
        """
        if self._current_config is None:
            raise RuntimeError("No configuration loaded. Call load_config() first.")
        return self._current_config

    def save_config(
        self, config: SystemParameters | None = None, filename: str | None = None
    ) -> Path:
        """Save configuration to file.

        Args:
            config: Configuration to save (uses current if None)
            filename: Output filename (auto-generated if None)

        Returns:
            Path to saved configuration file
        """
        if config is None:
            config = self.get_current_config()

        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")  # noqa: DTZ005
            filename = f"{config.config_name}_{timestamp}.json"

        filepath = self.config_dir / filename
        config.save_to_file(filepath)
        return filepath

    def list_scenarios(self) -> list[str]:
        """List all available predefined scenarios."""
        return list_available_scenarios()

    def list_saved_configs(self) -> list[Path]:
        """List all saved configuration files."""
        return list(self.config_dir.glob("*.json"))

    def create_study_configs(
        self,
        base_scenario: str,
        parameter_studies: dict[str, list[float]],
        study_name: str = "parameter_study",
    ) -> list[SystemParameters]:
        """Create configurations for parameter sensitivity studies.

        Args:
            base_scenario: Base scenario to modify
            parameter_studies: Dictionary mapping parameter paths to value lists
            study_name: Name for the study

        Returns:
            List of configurations for parameter sweep

        Example:
            >>> studies = {
            ...     "battery.soc_floor": [0.1, 0.15, 0.2],
            ...     "grid.cell_size": [30, 40, 50, 60]
            ... }
            >>> configs = manager.create_study_configs("baseline", studies)
        """
        all_configs = []

        for param_path, values in parameter_studies.items():
            sweep_configs = create_parameter_sweep(
                base_scenario,
                param_path,
                values,
                f"{study_name}_{param_path.replace('.', '_')}",
            )
            all_configs.extend(sweep_configs)

        return all_configs

    def validate_config(self, config: SystemParameters | None = None) -> list[str]:
        """Validate configuration consistency.

        Args:
            config: Configuration to validate (uses current if None)

        Returns:
            List of validation warnings
        """
        if config is None:
            config = self.get_current_config()

        return config.validate_consistency()

    def print_config_summary(self, config: SystemParameters | None = None) -> None:
        """Print human-readable configuration summary.

        Args:
            config: Configuration to summarize (uses current if None)
        """
        if config is None:
            config = self.get_current_config()

        print(config.summary())

        # Print validation warnings if any
        warnings = self.validate_config(config)
        if warnings:
            print("\n⚠️  Configuration Warnings:")
            for warning in warnings:
                print(f"  • {warning}")

    # Convenience methods for specific modules

    def get_stage0_params(self, config: SystemParameters | None = None) -> dict:
        """Get parameters for Stage 0 battery optimization.

        Args:
            config: Configuration to use (uses current if None)

        Returns:
            Dictionary with Stage 0 parameters
        """
        if config is None:
            config = self.get_current_config()

        return config.get_battery_constraint_params()

    def get_stage1_params(self, config: SystemParameters | None = None) -> dict:
        """Get parameters for Stage 1 grid generation.

        Args:
            config: Configuration to use (uses current if None)

        Returns:
            Dictionary with Stage 1 parameters
        """
        if config is None:
            config = self.get_current_config()

        return config.get_grid_build_params()

    def get_stage2_params(self, config: SystemParameters | None = None) -> dict:
        """Get parameters for Stage 2 fleet optimization.

        Args:
            config: Configuration to use (uses current if None)

        Returns:
            Dictionary with Stage 2 parameters
        """
        if config is None:
            config = self.get_current_config()

        return config.get_fleet_optimization_params()


# Global configuration manager instance
_global_config_manager = None


def get_config_manager() -> ConfigManager:
    """Get global configuration manager instance."""
    global _global_config_manager
    if _global_config_manager is None:
        _global_config_manager = ConfigManager()
    return _global_config_manager


def load_global_config(source: str | Path | SystemParameters) -> SystemParameters:
    """Load configuration into global manager.

    Args:
        source: Configuration source (scenario name, file path, or config object)

    Returns:
        Loaded configuration
    """
    manager = get_config_manager()
    return manager.load_config(source)


def get_global_config() -> SystemParameters:
    """Get currently loaded global configuration."""
    manager = get_config_manager()
    return manager.get_current_config()
