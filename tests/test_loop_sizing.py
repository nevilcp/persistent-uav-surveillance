"""Sizing of the shared-loop fleet."""

import math
from dataclasses import replace
from itertools import combinations

import pytest

from uav_surveil.airspace import Airspace
from uav_surveil.fleet import size_loop_fleet
from uav_surveil.grid import Grid
from uav_surveil.params import (
    Parameters,
    PatrolParams,
    RegionParams,
    UAVParams,
)
from uav_surveil.routing import shared_loop

PORT_REGION = RegionParams(
    width=1200.0,
    length=1200.0,
    depots=((-40.0, 600.0), (1240.0, 600.0), (600.0, 1240.0)),
    zones=(
        (200.0, 520.0, 520.0, 680.0),
        (960.0, 640.0, 1040.0, 1120.0),
        (680.0, 640.0, 960.0, 720.0),
    ),
)
PORT = Parameters(
    region=PORT_REGION,
    uav=UAVParams(depot_slots=2),
    patrol=PatrolParams(mode="shared_loop"),
)
GRID = Grid(PORT_REGION)
AIR = Airspace.from_region(GRID, PORT_REGION)
LOOP = shared_loop(GRID, AIR)


def test_single_base_matches_the_formulas_by_hand():
    params = Parameters(
        uav=UAVParams(depot_slots=2), patrol=PatrolParams(response_time=400.0)
    )
    grid = Grid(params.region)
    air = Airspace.from_region(grid, params.region)
    loop = shared_loop(grid, air)
    fleet = size_loop_fleet(loop, air, params)

    assert loop.length == pytest.approx(144 * 40.0)
    assert fleet.n_fly == math.ceil(144 * 40.0 / (6.0 * 180.0)) + 1 == 7
    (base,) = fleet.bases
    transit = sum(math.dist(v, (-500.0, 0.0)) for v in loop.vertices) / 144 / 6.0
    window = 1890.0 - 2.0 * transit - 10.0
    rate = 7 / window
    assert base.mean_transit == pytest.approx(transit)
    assert base.arrival_rate == pytest.approx(rate)
    assert base.n_rotate == math.ceil(rate * (2.0 * transit + 60.0 + 10.0))
    assert base.utilisation == pytest.approx(rate * 60.0 / 2)
    assert base.spare and fleet.n_spare == 1
    assert fleet.total == 7 + base.n_rotate + 1


def test_unlimited_slots_give_zero_utilisation():
    params = Parameters(patrol=PatrolParams(response_time=400.0))
    grid = Grid(params.region)
    air = Airspace.from_region(grid, params.region)
    fleet = size_loop_fleet(shared_loop(grid, air), air, params)
    assert fleet.bases[0].utilisation == 0.0


def test_port_flyers_are_shared_by_stretch_and_add_up():
    fleet = size_loop_fleet(LOOP, AIR, PORT)
    assert fleet.n_fly == math.ceil(LOOP.length / (6.0 * 180.0)) + 1 == 39
    assert sum(b.n_fly for b in fleet.bases) == 39
    assert sum(b.share for b in fleet.bases) == pytest.approx(LOOP.length)
    for b in fleet.bases:
        assert b.n_fly == pytest.approx(39 * b.share / LOOP.length, abs=1.0)
        assert b.utilisation < 1.0


def test_port_needs_spares_at_all_three_bases():
    fleet = size_loop_fleet(LOOP, AIR, PORT)
    assert [b.spare for b in fleet.bases] == [True, True, True]


def _brute_force(response_time: float) -> int | None:
    reach = [
        [AIR.depot_distance(v, m) / 6.0 for m in range(3)] for v in LOOP.vertices
    ]
    for size in (1, 2, 3):
        for subset in combinations(range(3), size):
            if max(min(row[j] for j in subset) for row in reach) <= response_time:
                return size
    return None


@pytest.mark.parametrize("response_time", [150.0, 170.0, 250.0])
def test_spare_set_is_the_smallest_cover(response_time):
    params = replace(PORT, patrol=PatrolParams(response_time=response_time))
    fleet = size_loop_fleet(LOOP, AIR, params)
    assert fleet.n_spare == _brute_force(response_time)


def test_infeasible_response_time_raises():
    params = replace(PORT, patrol=PatrolParams(response_time=100.0))
    with pytest.raises(ValueError, match="T_resp"):
        size_loop_fleet(LOOP, AIR, params)


def test_closed_base_hands_its_stretch_to_the_others():
    params = replace(PORT, patrol=PatrolParams(response_time=400.0))
    fleet = size_loop_fleet(LOOP, AIR, params, open_bases=[0, 1])
    assert [b.index for b in fleet.bases] == [0, 1]
    assert sum(b.share for b in fleet.bases) == pytest.approx(LOOP.length)
    assert sum(b.n_fly for b in fleet.bases) == fleet.n_fly


def test_failures_absorbed_adds_flyers():
    more = replace(PORT, patrol=PatrolParams(failures_absorbed=3))
    assert size_loop_fleet(LOOP, AIR, more).n_fly == 41
