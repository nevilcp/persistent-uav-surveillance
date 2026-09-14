"""Tests for MILP-backed fleet sizing (Stage 2)."""

import math

from uav_surveil.config import load_scenario
from uav_surveil.stage2_fleet import (
    optimize_fleet_from_config,
    optimize_fleet_size,
)


def test_milp_matches_closed_form():
    cfg = load_scenario("baseline")

    res_closed = optimize_fleet_size(
        K_inv=cfg.optimization.total_inventory,
        beta_min=cfg.optimization.spare_floor_ratio,
    )

    res_milp = optimize_fleet_from_config(cfg, use_milp=True)

    # If MILP unavailable, the wrapper falls back and we still match.
    assert res_milp.n_launch == res_closed.n_launch
    assert res_milp.n_spare == res_closed.n_spare
    assert math.isclose(res_milp.total_cost, res_closed.total_cost)
