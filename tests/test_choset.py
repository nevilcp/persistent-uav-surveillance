"""Connectivity events, exhaustive sweeps and sector-local round-robin routes."""

import itertools
from dataclasses import replace

import pytest

from uav_surveil import Parameters, Simulation, make_simulation
from uav_surveil.airspace import Airspace
from uav_surveil.grid import (
    Grid,
    boustrophedon_decomposition,
    boustrophedon_order,
    choset_order,
)
from uav_surveil.params import RegionParams
from uav_surveil.routing import round_robin_routes


def test_choset_splits_and_merges_at_a_rectangle():
    region = RegionParams(width=120.0, length=120.0, zones=((40., 40., 80., 80.),))
    grid = Grid(region)
    air = Airspace.from_region(grid, region)
    regions = boustrophedon_decomposition(air.free_cells)
    assert [{c.index for c in r.cells} for r in regions] == [
        {0, 1, 2}, {3}, {5}, {6, 7, 8}
    ]
    assert [r.neighbours for r in regions] == [(1, 2), (0, 3), (0, 3), (1, 2)]
    order = choset_order(air.free_cells)
    assert sorted(c.index for c in order) == [0, 1, 2, 3, 5, 6, 7, 8]
    # Traverse whole decomposition cells rather than skipping holes row by row.
    assert order != boustrophedon_order(air.free_cells)
    routes = round_robin_routes(grid, 2, air.free_cells)
    assert routes[0].cells == tuple(order[::2])
    assert routes[1].cells == tuple(order[1::2])


def test_one_to_one_interval_changes_do_not_split_a_region():
    grid = Grid(RegionParams(width=120., length=120.))
    cells = [c for c in grid.cells if c.index not in (0, 8)]
    regions = boustrophedon_decomposition(cells)
    assert len(regions) == 1
    assert set(regions[0].cells) == set(cells)


def test_open_space_retains_the_baseline_sweep_and_supports_odd_grids():
    grid = Grid(RegionParams(width=200., length=200.))
    assert len(boustrophedon_decomposition(grid.cells)) == 1
    assert choset_order(grid.cells) == boustrophedon_order(grid.cells)
    assert len(round_robin_routes(grid, 5)) == 5


def test_disconnected_sweep_components_are_all_visited_once():
    grid = Grid(RegionParams(width=120., length=120.))
    cells = [c for c in grid.cells if c.row != 1]
    assert len(boustrophedon_decomposition(cells)) == 2
    assert sorted(c.index for c in choset_order(cells)) == [0, 1, 2, 6, 7, 8]


def test_port_routes_cover_free_space_once_and_stay_with_their_depot_sector():
    params = Parameters.from_json("configs/port.json")
    sim = make_simulation(params)
    assert isinstance(sim, Simulation)
    assert not hasattr(sim, "loop")
    visited = [c.index for r in sim.routes for c in r.cells]
    assert sorted(visited) == sorted(c.index for c in sim.airspace.free_cells)
    assert len(sim.routes) == 87
    for route in sim.routes:
        assert route.cells
        assert all(sim.airspace.nearest_depot(c.position) == route.depot for c in route.cells)
        for a, b in zip(route.cells, (*route.cells[1:], route.cells[0])):
            path = sim.airspace.path(a.position, b.position)
            assert all(sim.airspace.clear(p, q) for p, q in itertools.pairwise(path))


@pytest.mark.parametrize("count", [0, 10])
def test_empty_or_excess_routes_are_rejected(count):
    grid = Grid(RegionParams(width=120., length=120.))
    with pytest.raises(ValueError, match="route count"):
        round_robin_routes(grid, count)


def test_closure_reroutes_inbound_uavs_and_makes_stored_reserves_unavailable():
    params = Parameters.from_json("configs/port.json")
    params = replace(params, failure=replace(params.failure, closed_depot=2, closure_time=0.))
    sim = Simulation(params)
    from uav_surveil.uav import UAVState
    incoming = next(u for u in sim.uavs if u.home == 2 and u.route)
    incoming.state = UAVState.RETURNING
    incoming.position = (600., 1180.)
    sim.step()
    assert incoming.home in (0, 1)
    assert all(r.depot != 2 for r in sim.routes)
    assert not sim.depots[2].is_open
    assert sim._take_reserve(True, 2).home != 2

