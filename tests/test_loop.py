"""Shared patrol loop."""

import math

import pytest

from uav_surveil.airspace import Airspace
from uav_surveil.grid import Grid
from uav_surveil.params import RegionParams
from uav_surveil.routing import _cycle, shared_loop

PORT = RegionParams(
    width=1200.0,
    length=1200.0,
    depots=((-40.0, 600.0), (1240.0, 600.0), (600.0, 1240.0)),
    zones=(
        (200.0, 520.0, 520.0, 680.0),
        (960.0, 640.0, 1040.0, 1120.0),
        (680.0, 640.0, 960.0, 720.0),
    ),
)
GRID = Grid(PORT)
AIR = Airspace.from_region(GRID, PORT)
LOOP = shared_loop(GRID, AIR)
OPEN_REGION = RegionParams(width=1200.0, length=1200.0)
OPEN_GRID = Grid(OPEN_REGION)
OPEN_LOOP = shared_loop(OPEN_GRID, Airspace.from_region(OPEN_GRID, OPEN_REGION))


@pytest.mark.parametrize(("rows", "cols"), [(4, 5), (5, 4), (6, 6), (2, 2)])
def test_cycle_is_hamiltonian_and_closed(rows, cols):
    cycle = _cycle(rows, cols)
    assert sorted(cycle) == [(r, c) for r in range(rows) for c in range(cols)]
    for a, b in zip(cycle, cycle[1:] + cycle[:1]):
        assert abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1


def test_odd_by_odd_grid_has_no_closed_loop():
    with pytest.raises(ValueError, match="even"):
        _cycle(3, 5)


def test_open_loop_visits_every_cell_and_is_n_times_delta():
    assert len(OPEN_LOOP.cells) == 900
    assert OPEN_LOOP.length == pytest.approx(900 * 40.0)


def test_port_loop_visits_every_free_cell_once():
    assert sorted(c.index for c in LOOP.cells) == sorted(
        c.index for c in AIR.free_cells
    )
    assert set(LOOP.cells) <= set(AIR.free_cells)


def test_port_loop_length_is_between_the_free_cells_and_a_bound():
    assert 830 * 40.0 < LOOP.length < 45_000.0


def test_loop_is_closed_and_consistent():
    points = list(LOOP.vertices)
    total = sum(math.dist(a, b) for a, b in zip(points, points[1:] + points[:1]))
    assert total == pytest.approx(LOOP.length)
    assert LOOP.arcs[0] == 0.0 and all(
        a < b for a, b in zip(LOOP.arcs, LOOP.arcs[1:])
    )
    assert LOOP.point_at(0.0) == LOOP.vertices[0]
    assert LOOP.point_at(LOOP.length) == pytest.approx(LOOP.vertices[0])


def test_no_loop_segment_crosses_a_zone():
    points = list(LOOP.vertices)
    for a, b in zip(points, points[1:] + points[:1]):
        assert AIR.clear(a, b)


def test_point_at_interpolates_and_wraps():
    x0, y0 = OPEN_LOOP.vertices[0]
    x1, y1 = OPEN_LOOP.vertices[1]
    mid = OPEN_LOOP.point_at(20.0)
    assert mid == pytest.approx(((x0 + x1) / 2, (y0 + y1) / 2))
    assert OPEN_LOOP.point_at(20.0 + OPEN_LOOP.length) == pytest.approx(mid)
    assert OPEN_LOOP.point_at(-OPEN_LOOP.length + 20.0) == pytest.approx(mid)


def test_cell_arcs_match_cell_positions():
    for cell, arc in zip(LOOP.cells, LOOP.cell_arcs):
        assert LOOP.point_at(arc) == pytest.approx(cell.position)


def test_cells_near_includes_the_cell_itself_and_wraps_the_start():
    cell, arc = LOOP.cells[100], LOOP.cell_arcs[100]
    near = LOOP.cells_near(arc, 1.0)
    assert near == [cell]
    start = LOOP.cells_near(0.0, 50.0)
    assert LOOP.cells[0] in start and LOOP.cells[-1] in start
