"""Tests for Stage 0: Battery Feasibility Checker and Optimization."""

import pytest

from ..stage0_battery import (
    analyze_battery_margin,
    battery_feasible,
    calculate_max_grid_distance,
    estimate_mission_time,
    optimize_battery_reserve,
)


class TestBatteryOptimization:
    """Test battery reserve optimization (mathematical model implementation)."""

    def test_optimal_reserve_basic(self):
        """Test basic optimization with your baseline parameters."""
        # Your baseline: 500m ferry, 1000m grid, 4m/s, 2100s endurance
        result = optimize_battery_reserve(
            d_ferry=500.0,
            l_grid=1000.0,
            v_max=4.0,
            endurance=2100.0,
            xi_max=0.5,  # Allow high reserves for testing
        )

        # Total distance: 2*500 + 1000 = 2000m
        # Max distance: 4 * 2100 = 8400m
        # Utilization needed: 2000/8400 = 0.238 (23.8%)
        # Minimum reserve xi: 1 - 0.238 = 0.762 (76.2% can be kept as reserve)
        assert result.xi_optimal >= 0.0  # Should be non-negative
        assert result.is_feasible is True

    def test_tight_mission_optimization(self):
        """A mission needing exactly 90% of range has zero deficit at soc_floor=0.1."""
        d_ferry = 500.0
        v_max = 4.0
        endurance = 2100.0
        total_capacity = v_max * endurance  # 8400m

        # Grid distance for exactly 90% utilization: 0.9 * 8400 - 1000 = 6560m
        l_grid = 0.9 * total_capacity - 2 * d_ferry

        result = optimize_battery_reserve(d_ferry, l_grid, v_max, endurance)

        # Exactly at the usable-budget boundary: feasible, zero deficit.
        assert result.xi_optimal == pytest.approx(0.0, abs=1e-9)
        assert result.is_feasible is True
        assert result.margin_seconds == pytest.approx(0.0, abs=1e-6)

    def test_impossible_mission(self):
        """Test mission that exceeds 100% battery capacity."""
        result = optimize_battery_reserve(
            d_ferry=1000.0,  # Very long ferry
            l_grid=8000.0,  # Very long grid patrol
            v_max=4.0,
            endurance=2100.0,
        )

        # Total: 2*1000 + 8000 = 10000m > 8400m max capacity
        assert result.is_feasible is False
        assert result.xi_optimal > 0.0  # Positive deficit, not clamped to 0
        assert result.margin_seconds < 0  # Negative margin
        assert result.margin_distance < 0

    def test_deficit_when_exceeding_budget(self):
        """A mission beyond the usable battery budget reports a positive deficit."""
        result = optimize_battery_reserve(
            d_ferry=500.0,
            l_grid=7000.0,
            v_max=4.0,
            endurance=2100.0,
        )

        # d_need = 2*500 + 7000 = 8000m; d_usable = 4*2100*0.9 = 7560m -> infeasible.
        expected_xi = 8000.0 / 8400.0 - 0.9
        assert result.is_feasible is False
        assert result.xi_optimal == pytest.approx(expected_xi, abs=1e-6)
        assert result.margin_seconds < 0

    def test_zero_distances(self):
        """Test optimization with zero distances."""
        result = optimize_battery_reserve(0.0, 0.0, 4.0, 2100.0)

        # No distance required = no deficit, fully feasible.
        assert result.xi_optimal == 0.0
        assert result.is_feasible is True
        assert result.utilization == 0.0
        assert result.margin_seconds == pytest.approx(2100.0 * 0.9)


class TestBatteryAnalysis:
    """Test battery margin analysis functions."""

    def test_analyze_battery_margin_safe(self):
        """Test margin analysis for a feasible, well-within-budget mission."""
        required_xi, margin_seconds, is_safe = analyze_battery_margin(
            d_ferry=500.0,
            l_grid=1000.0,
            v_max=4.0,
            endurance=2100.0,
            target_xi=0.3,
        )

        # d_need = 2000m, d_usable = 7560m -> feasible, zero deficit.
        assert is_safe is True
        assert margin_seconds > 0  # Positive margin
        assert required_xi == 0.0

    def test_analyze_battery_margin_tight(self):
        """Test margin analysis for a mission beyond the usable budget."""
        required_xi, margin_seconds, is_safe = analyze_battery_margin(
            d_ferry=500.0,
            l_grid=7000.0,
            v_max=4.0,
            endurance=2100.0,
            target_xi=0.1,
        )

        # d_need = 8000m, d_usable = 7560m -> infeasible, positive deficit.
        assert is_safe is False
        assert margin_seconds < 0  # Negative margin
        assert required_xi > 0.0


class TestBatteryFeasible:
    """Test battery feasibility validation (existing functionality)."""

    def test_feasible_mission(self):
        """Test a feasible mission passes validation."""
        # Baseline parameters from reference doc
        d_ferry = 500.0  # depot to grid edge (m)
        l_grid = 1000.0  # grid patrol distance (m)
        v_max = 4.0  # cruise speed (m/s)
        endurance = 2100.0  # battery endurance (s)

        # Should pass: 2*500 + 1000 = 2000m < 4*2100*0.9 = 7560m
        assert battery_feasible(d_ferry, l_grid, v_max, endurance) is True

    def test_infeasible_mission_raises_error(self):
        """Test infeasible mission raises ValueError."""
        d_ferry = 500.0
        l_grid = 8000.0  # Too long for battery
        v_max = 4.0
        endurance = 2100.0

        with pytest.raises(ValueError, match="Mission infeasible"):
            battery_feasible(d_ferry, l_grid, v_max, endurance)

    def test_parameter_validation(self):
        """Test parameter validation catches invalid inputs."""
        valid_params = (500.0, 1000.0, 4.0, 2100.0)

        # Negative ferry distance
        with pytest.raises(ValueError, match="Ferry distance must be non-negative"):
            battery_feasible(-100.0, *valid_params[1:])

        # Negative grid distance
        with pytest.raises(
            ValueError, match="Grid patrol distance must be non-negative"
        ):
            battery_feasible(valid_params[0], -100.0, *valid_params[2:])

        # Zero velocity
        with pytest.raises(ValueError, match="Max velocity must be positive"):
            battery_feasible(*valid_params[:2], 0.0, valid_params[3])

        # Zero endurance
        with pytest.raises(ValueError, match="Endurance must be positive"):
            battery_feasible(*valid_params[:3], 0.0)

        # Invalid SoC floor
        with pytest.raises(ValueError, match="SoC floor must be >= 0.1"):
            battery_feasible(*valid_params, soc_floor=0.05)

    def test_soc_floor_effect(self):
        """Test that SoC floor reduces available capacity."""
        d_ferry, l_grid, v_max, endurance = 500.0, 1000.0, 4.0, 2100.0

        # Should pass with 10% floor
        assert battery_feasible(d_ferry, l_grid, v_max, endurance, soc_floor=0.1)

        # Should fail with 80% floor (very high SoC reserve)
        # Available: 4 * 2100 * 0.2 = 1680m < 2000m needed
        with pytest.raises(ValueError, match="Mission infeasible"):
            battery_feasible(d_ferry, l_grid, v_max, endurance, soc_floor=0.8)


class TestCalculateMaxGridDistance:
    """Test maximum grid distance calculation."""

    def test_baseline_calculation(self):
        """Test calculation with baseline parameters."""
        d_ferry = 500.0
        v_max = 4.0
        endurance = 2100.0
        soc_floor = 0.1

        # Available: 4 * 2100 * 0.9 = 7560m
        # Ferry: 2 * 500 = 1000m
        # Grid: 7560 - 1000 = 6560m
        expected = 6560.0
        result = calculate_max_grid_distance(d_ferry, v_max, endurance, soc_floor)
        assert abs(result - expected) < 0.1

    def test_no_grid_possible(self):
        """Test when ferry distance exceeds capacity."""
        d_ferry = 5000.0  # Very long ferry
        v_max = 4.0
        endurance = 2100.0

        with pytest.raises(ValueError, match="No grid patrol possible"):
            calculate_max_grid_distance(d_ferry, v_max, endurance)


class TestEstimateMissionTime:
    """Test mission time estimation."""

    def test_time_calculation(self):
        """Test basic mission time calculation."""
        d_ferry = 500.0  # m
        l_grid = 1000.0  # m
        v_max = 4.0  # m/s

        # Total: 2*500 + 1000 = 2000m
        # Time: 2000/4 = 500s
        expected = 500.0
        result = estimate_mission_time(d_ferry, l_grid, v_max)
        assert abs(result - expected) < 0.1

    def test_zero_distances(self):
        """Test with zero distances."""
        result = estimate_mission_time(0.0, 0.0, 4.0)
        assert result == 0.0


class TestInitializeFeasibilityGate:
    """Test that GSSSimulation.initialize() enforces Stage 0 per-route."""

    def test_initialize_succeeds_on_feasible_config(self):
        from ..config import get_test_config
        from ..gss.simulation import GSSSimulation

        sim = GSSSimulation(config=get_test_config())
        assert sim.initialize() is True

    def test_initialize_raises_on_infeasible_battery(self):
        from ..config import get_test_config
        from ..gss.simulation import GSSSimulation

        config = get_test_config()
        # Push the depot far from the grid at the endurance floor so every
        # route's 2*d_ferry alone exceeds the usable battery budget.
        config.battery.total_endurance = 300.0
        config.mission.depot_x = -1000.0

        sim = GSSSimulation(config=config)
        with pytest.raises(ValueError, match="Stage 0"):
            sim.initialize()
