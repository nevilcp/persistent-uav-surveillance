#!/usr/bin/env python3
"""Test adaptive β feedback with different fleet sizes.

This test demonstrates when adaptive β feedback becomes effective.
Run with: py -m pytest uav_surveil/tests/test_adaptive_beta.py -v -s

Or directly: py uav_surveil/tests/test_adaptive_beta.py
"""

import sys, os

sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))

from uav_surveil.config import load_scenario
from uav_surveil.stage2_fleet import optimize_fleet_from_config
from uav_surveil.stage3_route_factory import get_route_generator
from uav_surveil.stage3_schedule import schedule_from_config
from uav_surveil.stage1_grid import build_grid_from_config
import copy


def test_adaptive_beta_effectiveness():
    """Test that adaptive β feedback works correctly with different inventory sizes."""
    config = load_scenario("baseline")

    # Generate cells once
    cells = build_grid_from_config(config)
    route_fn = get_route_generator(config.optimization.route_algorithm)

    print("\n🧪 Testing Adaptive β with Different Inventory Sizes")
    print("=" * 60)

    changes_observed = False

    for total_inv in [20, 30, 40, 50]:
        print(f"\n📊 Total Inventory: {total_inv} UAVs")

        # Update config
        test_config = copy.deepcopy(config)
        test_config.optimization.total_inventory = total_inv

        # Stage-2 (initial): Default β = 0.2
        fleet_result_initial = optimize_fleet_from_config(test_config)
        print(
            f"   Initial (β=0.2): {fleet_result_initial.n_launch} active + {fleet_result_initial.n_spare} spare"
        )

        # Stage-3: Generate routes and schedule
        routes, _ = route_fn(
            cells, fleet_result_initial.n_launch, config.uav.cruise_speed
        )
        schedule_summary = schedule_from_config(config, routes)

        print(
            f"   Stage-3B: n_surge={schedule_summary.n_surge}, β_adapt={schedule_summary.β_adapt:.3f}"
        )

        # Stage-2 (adaptive): Use β_adapt
        adaptive_config = copy.deepcopy(test_config)
        adaptive_config.optimization.spare_floor_ratio = schedule_summary.β_adapt
        fleet_result_adaptive = optimize_fleet_from_config(adaptive_config)

        print(
            f"   Adaptive (β={schedule_summary.β_adapt:.3f}): {fleet_result_adaptive.n_launch} active + {fleet_result_adaptive.n_spare} spare"
        )

        # Show change
        spare_change = fleet_result_adaptive.n_spare - fleet_result_initial.n_spare
        if spare_change != 0:
            print(f"   ✅ CHANGE: {spare_change:+d} spares (adaptive β working!)")
            changes_observed = True
        else:
            print(f"   ⚪ NO CHANGE: discrete constraints limit adjustment")

    # Verify that adaptive β shows changes with larger fleets
    assert changes_observed, "Adaptive β should show changes with larger fleet sizes"
    print(f"\n✅ Test passed: Adaptive β feedback working correctly!")


if __name__ == "__main__":
    test_adaptive_beta_effectiveness()
