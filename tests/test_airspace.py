import itertools
import math

import pytest

from uav_surveil.airspace import Airspace
from uav_surveil.geometry import distance, path_length
from uav_surveil.grid import Grid
from uav_surveil.params import RegionParams

PORT = RegionParams(
    width=1200.0,
    length=1200.0,
    depots=((-40.0, 600.0), (1240.0, 600.0), (600.0, 1240.0)),
    zones=(
        (200.0, 520.0, 520.0, 680.0),  # tank
        (960.0, 640.0, 1040.0, 1120.0),  # tower bar
        (680.0, 640.0, 960.0, 720.0),  # tower arm
    ),
)
DRONE_A = (920.0, 760.0)
GRID = Grid(PORT)
AIR = Airspace.from_region(GRID, PORT)
OPEN = Airspace.from_region(GRID, RegionParams(width=1200.0, length=1200.0))


def test_port_has_830_free_cells():
    assert len(AIR.free_cells) == 830
    assert len(AIR.free_index) == 830


def test_without_zones_everything_is_euclidean():
    p, q = (30.0, 50.0), (700.0, 900.0)
    assert OPEN.distance(p, q) == distance(p, q)
    assert OPEN.path(p, q) == [p, q]
    assert OPEN.depot_distance(p, 0) == distance(p, OPEN.depots[0])
    assert len(OPEN.free_cells) == 900
    assert OPEN.signed_distance(p) == math.inf


def test_drone_a_distances_to_the_three_bases():
    to_east = AIR.depot_distance(DRONE_A, 1)
    assert distance(DRONE_A, AIR.depots[1]) == pytest.approx(358.0, abs=1.0)
    # the exact geodesic is 884 m; the cell graph adds a few percent
    assert 884.0 <= to_east <= 884.0 * 1.06
    assert AIR.depot_distance(DRONE_A, 2) == pytest.approx(577.0, abs=1.0)
    assert AIR.nearest_depot(DRONE_A) == 2
    assert AIR.nearest_depot(DRONE_A, [0, 1]) == 1


def test_flyable_distance_never_beats_straight_line():
    points = [DRONE_A, (100.0, 600.0), (1100.0, 300.0), (600.0, 1000.0)]
    for p in points:
        for q in points:
            assert AIR.distance(p, q) >= distance(p, q) - 1e-9


def test_distance_matches_path_length_and_paths_avoid_zones():
    pairs = [(DRONE_A, (100.0, 600.0)), ((300.0, 300.0), (300.0, 900.0))]
    pairs += [(DRONE_A, depot) for depot in AIR.depots]
    for p, q in pairs:
        route = AIR.path(p, q)
        assert route[0] == p and route[-1] == q
        assert path_length(route) == pytest.approx(AIR.distance(p, q))
        assert all(AIR.clear(a, b) for a, b in itertools.pairwise(route))
        assert all(AIR.signed_distance(pt) >= -1e-6 for pt in route)


def test_depot_path_matches_depot_distance():
    route = AIR.depot_path(DRONE_A, 1)
    assert route[-1] == AIR.depots[1]
    assert path_length(route) == pytest.approx(AIR.depot_distance(DRONE_A, 1))


def test_clear_blocks_crossings_and_allows_grazing():
    assert not AIR.clear((100.0, 600.0), (600.0, 600.0))  # through the tank
    assert AIR.clear((100.0, 700.0), (600.0, 700.0))  # above it
    assert AIR.clear((100.0, 520.0), (600.0, 520.0))  # along its lower edge
    assert AIR.clear((520.0, 400.0), (520.0, 800.0))  # along its right edge
    assert not AIR.clear((300.0, 600.0), (300.0, 600.0))  # a point inside


def test_signed_distance_signs():
    assert AIR.signed_distance((300.0, 600.0)) == pytest.approx(-80.0)
    assert AIR.signed_distance((200.0, 600.0)) == pytest.approx(0.0)
    assert AIR.signed_distance((140.0, 600.0)) == pytest.approx(60.0)
    assert AIR.signed_distance((560.0, 720.0)) == pytest.approx(math.hypot(40, 40))


def test_margin_removes_neighbouring_cells_and_blocks_the_inflated_edge():
    region = RegionParams(
        width=1200.0, length=1200.0, zones=((200.0, 520.0, 520.0, 680.0),)
    )
    wide = Airspace(GRID, region.zones, 30.0, [(-40.0, 600.0)])
    assert len(wide.free_cells) < 900 - 40
    assert wide.blocked((190.0, 600.0))
    assert not wide.clear((100.0, 520.0), (600.0, 520.0))


def test_base_inside_a_zone_is_rejected():
    with pytest.raises(ValueError, match="base 0"):
        Airspace(GRID, PORT.zones, 0.0, [(300.0, 600.0)])


def test_disconnected_free_space_is_rejected():
    wall = ((0.0, 560.0, 1200.0, 640.0),)
    with pytest.raises(ValueError, match="not connected"):
        Airspace(GRID, wall, 0.0, [(600.0, -40.0)])


def test_point_inside_a_zone_has_no_flyable_link():
    with pytest.raises(ValueError, match="no-fly zone"):
        AIR.distance((300.0, 600.0), (100.0, 100.0))
