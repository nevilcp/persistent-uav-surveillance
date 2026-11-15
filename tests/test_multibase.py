"""Round-robin patrol over several bases and no-fly zones."""

import collections
import math
from dataclasses import replace

import pytest

from uav_surveil.airspace import Airspace
from uav_surveil.coverage import CoverageMap
from uav_surveil.failure import _insertion_cost
from uav_surveil.fleet import apportion
from uav_surveil.grid import Grid
from uav_surveil.params import (
    FleetParams,
    MissionParams,
    Parameters,
    RegionParams,
    UAVParams,
)
from uav_surveil.routing import (
    Route,
    airborne_time,
    assign_bases,
    base_ferry_times,
    ferry_time,
    loop_distance,
    round_robin_routes,
)
from uav_surveil.simulation import Simulation
from uav_surveil.uav import UAVState

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
    fleet=FleetParams(size=120, batch_size=27, reserve_fraction=0.05),
    mission=MissionParams(duration=600.0),
)
GRID = Grid(PORT_REGION)
AIR = Airspace.from_region(GRID, PORT_REGION)


@pytest.fixture(scope="module")
def port() -> Simulation:
    sim = Simulation(PORT)
    for _ in range(600):
        sim.step()
    return sim


@pytest.mark.parametrize(
    ("total", "weights", "expected"),
    [
        (27, [56, 20, 11], [17, 6, 4]),
        (6, [1, 1, 1], [2, 2, 2]),
        (5, [1, 1, 1], [2, 2, 1]),
    ],
)
def test_apportion_is_proportional_and_exact(total, weights, expected):
    assert apportion(total, weights) == expected
    assert sum(apportion(total, weights)) == total


def test_coverage_counts_only_active_cells():
    coverage = CoverageMap(4, active=[0, 2])
    coverage.observe(0, 10.0)
    coverage.observe(1, 10.0)
    assert len(coverage) == 2
    assert coverage.n_fresh(20.0, 180.0) == 1  # cell 2 never seen; cell 3 ignored
    assert math.isinf(coverage.max_age(20.0))
    coverage.observe(2, 15.0)
    assert coverage.max_age(20.0) == 10.0


def test_round_robin_over_free_cells_skips_zone_cells():
    routes = round_robin_routes(GRID, 87, AIR.free_cells)
    visited = [c.index for r in routes for c in r.cells]
    assert sorted(visited) == sorted(c.index for c in AIR.free_cells)


def test_default_round_robin_covers_every_cell():
    routes = round_robin_routes(GRID, 87)
    assert sum(len(r.cells) for r in routes) == 900


def test_route_goes_to_the_base_nearest_its_cells():
    east = tuple(c for c in AIR.free_cells if c.x > 1100.0 and c.y < 600.0)
    west = tuple(c for c in AIR.free_cells if c.x < 100.0)
    north = tuple(c for c in AIR.free_cells if c.y > 1150.0 and 400.0 < c.x < 800.0)
    routes = [Route(id=i + 1, cells=c) for i, c in enumerate((west, east, north))]
    assert [r.depot for r in assign_bases(routes, AIR)] == [0, 1, 2]


def test_single_base_keeps_routes_untouched():
    routes = round_robin_routes(Grid(RegionParams()), 4)
    open_sky = Airspace.from_region(Grid(RegionParams()), RegionParams())
    assert assign_bases(routes, open_sky) is routes


def test_ferry_time_without_zones_is_the_corner_formula():
    region = RegionParams()
    grid = Grid(region)
    uav = UAVParams()
    times = base_ferry_times(grid, Airspace.from_region(grid, region), uav)
    assert times == [ferry_time(grid, region.depot, uav)]


def test_port_ferry_times_exceed_the_straight_line_to_the_farthest_cell():
    for m, t in enumerate(base_ferry_times(GRID, AIR, UAVParams())):
        straight = max(math.dist(AIR.depots[m], c.position) for c in AIR.free_cells)
        assert t >= straight / 6.0 - 1e-9


def test_loop_and_airborne_time_grow_with_detours():
    cells = (GRID.cells[14 * 30 + 3], GRID.cells[14 * 30 + 25])  # across the tank
    straight = loop_distance(cells)
    flyable = loop_distance(cells, AIR)
    assert flyable > straight
    assert loop_distance(cells, None) == straight
    uav = UAVParams()
    assert airborne_time(cells, 100.0, uav, AIR) == pytest.approx(
        200.0 + flyable / uav.speed
    )


def test_insertion_cost_uses_the_flyable_distance():
    cells = GRID.cells
    a, b, c = cells[15 * 30 + 3], cells[15 * 30 + 25], cells[10 * 30 + 14]
    assert _insertion_cost([a, b], c, 1, AIR) != _insertion_cost([a, b], c, 1)
    assert _insertion_cost([a, b], c, 1, AIR) >= 0.0


def test_port_split_matches_the_design_numbers(port):
    split = port.split
    assert (split.n_fly, split.n_rotate, split.n_spare) == (87, 27, 6)
    assert len(port.coverage) == 830


def test_fleet_is_spread_over_the_bases_with_their_routes():
    port = Simulation(PORT)
    routes = collections.Counter(r.depot for r in port.routes)
    flyers = collections.Counter(u.home for u in port.uavs if u.launch_at is not None)
    assert flyers == routes
    rotation = collections.Counter(
        u.home for u in port.uavs if u.route is None and not u.is_contingency
    )
    assert sum(rotation.values()) == 27 and set(rotation) <= set(routes)
    spares = [u for u in port.uavs if u.is_contingency]
    assert len(spares) == 6


def test_no_uav_is_inside_a_zone(port):
    assert all(
        port.airspace.signed_distance(u.position) >= -1e-6
        for u in port.uavs
        if u.airborne
    )
    assert any(u.state is UAVState.ON_ROUTE for u in port.uavs)


def test_rotation_reserve_comes_from_the_home_base_only():
    sim = Simulation(PORT)
    for uav in sim.uavs:
        if uav.route is None and not uav.is_contingency and uav.home == 2:
            uav.state = UAVState.FAILED
    assert sim._take_reserve(False, 2) is None
    assert sim._take_reserve(False, 0).home == 0


def test_spare_falls_back_to_the_nearest_base_that_has_one():
    sim = Simulation(PORT)
    own = [u for u in sim.uavs if u.is_contingency and u.home == 1]
    for uav in own:
        uav.state = UAVState.FAILED
    spare = sim._take_reserve(True, 1)
    assert spare is not None and spare.home != 1
    near = min(
        (m for m in (0, 2) if any(u.is_contingency and u.home == m for u in sim.uavs)),
        key=lambda m: AIR.distance(AIR.depots[1], AIR.depots[m]),
    )
    assert spare.home == near


def test_replace_keeps_departure_and_depot():
    route = Route(id=1, cells=(), departure=30.0, depot=2)
    assert replace(route, departure=60.0).depot == 2
