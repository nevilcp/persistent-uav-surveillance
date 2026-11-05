"""Stages 1-3: fleet split, routes and schedule."""

import itertools
import math
from dataclasses import replace
from pathlib import Path

import pytest

from uav_surveil.fleet import split_fleet
from uav_surveil.geometry import distance
from uav_surveil.grid import Grid, boustrophedon_order
from uav_surveil.params import FleetParams, Parameters, UAVParams
from uav_surveil.routing import (
    airborne_time,
    check_admissible,
    ferry_time,
    loop_distance,
    round_robin_routes,
)
from uav_surveil.schedule import assign_departures

PARAMS = Parameters()
GRID = Grid(PARAMS.region)


def test_nominal_grid_has_144_cells():
    assert (GRID.n_rows, GRID.n_cols, len(GRID.cells)) == (12, 12, 144)


def test_cell_lookup_by_position():
    assert GRID.cell_at((20.0, 20.0)).index == 0
    assert GRID.cell_at((479.9, 479.9)).index == 143
    assert GRID.cell_at((-5.0, 100.0)) is None  # west of A
    assert GRID.cell_at((490.0, 100.0)) is None  # beyond the last whole cell


def test_ferry_distance_is_distance_to_farthest_corner():
    assert GRID.ferry_distance(PARAMS.region.depot) == pytest.approx(
        math.hypot(980, 480)
    )
    assert ferry_time(GRID, PARAMS.region.depot, PARAMS.uav) == pytest.approx(
        math.hypot(980, 480) / 6.0
    )


def test_usable_endurance_matches_baseline():
    assert PARAMS.uav.usable_endurance == pytest.approx(1890.0)


def test_boustrophedon_order_alternates_scan_direction():
    ordered = boustrophedon_order(GRID.cells)
    assert [c.col for c in ordered[:12]] == list(range(12))
    assert [c.col for c in ordered[12:24]] == list(range(11, -1, -1))
    assert len({c.index for c in ordered}) == 144


@pytest.mark.parametrize("n_fly", [6, 16, 26])
def test_round_robin_partitions_cells_and_balances_routes(n_fly):
    routes = round_robin_routes(GRID, n_fly)
    assigned = [c.index for r in routes for c in r.cells]
    assert sorted(assigned) == list(range(144))  # every cell on exactly one route
    sizes = [len(r.cells) for r in routes]
    assert max(sizes) - min(sizes) <= 1


def test_round_robin_deals_cells_cyclically():
    ordered = boustrophedon_order(GRID.cells)
    routes = round_robin_routes(GRID, 16)
    assert routes[0].cells == tuple(ordered[0::16])
    assert routes[5].cells == tuple(ordered[5::16])


def test_loop_distance_is_open_polyline():
    cells = round_robin_routes(GRID, 16)[0].cells
    expected = sum(
        distance(a.position, b.position) for a, b in itertools.pairwise(cells)
    )
    assert loop_distance(cells) == pytest.approx(expected)


def test_nominal_routes_satisfy_endurance_condition():
    ferry = ferry_time(GRID, PARAMS.region.depot, PARAMS.uav)
    for n_fly in (6, 16, 26):
        check_admissible(round_robin_routes(GRID, n_fly), ferry, PARAMS.uav)


def test_route_exceeding_the_usable_window_is_rejected():
    short_battery = UAVParams(endurance=600.0)  # T_use = 540 s < T_airborne = 643 s
    ferry = ferry_time(GRID, PARAMS.region.depot, short_battery)
    routes = round_robin_routes(GRID, 16)
    assert (
        airborne_time(routes[0].cells, ferry, short_battery)
        > short_battery.usable_endurance
    )
    with pytest.raises(ValueError, match="not admissible"):
        check_admissible(routes, ferry, short_battery)


@pytest.mark.parametrize(
    "size, expected", [(11, (6, 4, 1)), (21, (16, 4, 1)), (31, (26, 4, 1))]
)
def test_fleet_split_for_the_reference_fleet_sizes(size, expected):
    split = split_fleet(replace(PARAMS.fleet, size=size))
    assert (split.n_fly, split.n_rotate, split.n_spare) == expected


def test_fleet_split_is_optimal_among_all_feasible_splits():
    fleet = FleetParams(size=15, batch_size=3, reserve_fraction=0.2)
    split = split_fleet(fleet)
    cost = lambda f, r, s: fleet.fly_cost * f + fleet.reserve_cost * (r + s)
    feasible = [
        (n - r - s, r, s)
        for r in range(fleet.batch_size, 15)
        for s in range(math.ceil(0.2 * 15), 15)
        for n in [15]
        if n - r - s >= 1
    ]
    assert cost(split.n_fly, split.n_rotate, split.n_spare) == min(
        cost(*x) for x in feasible
    )
    assert split.total == 15


def test_fleet_split_rejects_infeasible_inventory():
    with pytest.raises(ValueError, match="infeasible"):
        split_fleet(FleetParams(size=5, batch_size=4, reserve_fraction=0.03))


def test_departure_times_follow_batch_formula():
    routes = assign_departures(
        round_robin_routes(GRID, 16), PARAMS.fleet, PARAMS.schedule
    )
    departures = [r.departure for r in routes]
    assert departures[:4] == [0.0, 60.0, 120.0, 180.0]  # batch 0
    assert departures[4:8] == [150.0, 210.0, 270.0, 330.0]  # batch 1 starts at T_batch
    assert departures[15] == 3 * 150.0 + 3 * 60.0


def test_baseline_json_equals_default_parameters():
    path = Path(__file__).parent.parent / "configs" / "baseline.json"
    assert Parameters.from_json(path) == PARAMS


def test_parameter_validation_enforces_launch_gap():
    with pytest.raises(ValueError, match="launch_gap"):
        Parameters(schedule=replace(PARAMS.schedule, launch_gap=30.0))


@pytest.mark.parametrize("rho_min", [-0.1, 1.0])
def test_parameter_validation_enforces_rho_min_range(rho_min):
    with pytest.raises(ValueError, match="rho_min"):
        Parameters(fleet=replace(PARAMS.fleet, reserve_fraction=rho_min))


def test_unknown_json_entry_is_rejected(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text('{"uav": {"spd": 5}}')
    with pytest.raises(ValueError, match="unknown"):
        Parameters.from_json(path)
