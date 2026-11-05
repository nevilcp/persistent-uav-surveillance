"""Stage 4 rules and the Stage 5 building blocks."""

import pytest

from uav_surveil.dispatch import (
    must_return,
    needs_replacement,
    predicted_arrival_battery,
    return_threshold,
)
from uav_surveil.failure import _insertion_cost
from uav_surveil.grid import Cell
from uav_surveil.params import DispatchParams, UAVParams
from uav_surveil.uav import UAV

UAV_P = UAVParams()
DISPATCH = DispatchParams()


def test_return_threshold_grows_with_distance_to_depot():
    near, far = return_threshold(0.0, UAV_P, DISPATCH), return_threshold(
        1000.0, UAV_P, DISPATCH
    )
    assert near == pytest.approx(0.1 + 0.02)  # sigma_min + sigma_buf at the depot
    assert far == pytest.approx(0.1 + 1000.0 / (6.0 * 2100.0) + 0.02)


def test_predicted_arrival_battery():
    b_hat = predicted_arrival_battery(0.5, 630.0, UAV_P)
    assert b_hat == pytest.approx(0.5 - 630 / 12600)


@pytest.mark.parametrize("distance_to_depot", [0.0, 520.0, 1100.0])
def test_replacement_condition_precedes_return_condition(distance_to_depot):
    """Replacement is true no later than return since sigma_pre > sigma_buf."""
    b = return_threshold(distance_to_depot, UAV_P, DISPATCH)
    assert must_return(b, distance_to_depot, UAV_P, DISPATCH)
    assert needs_replacement(b, distance_to_depot, UAV_P, DISPATCH)
    assert not needs_replacement(b + 0.1, distance_to_depot, UAV_P, DISPATCH)


def test_direct_return_from_threshold_ends_above_sigma_min():
    """Returning at the threshold leaves sigma_buf of margin on arrival."""
    d = 800.0
    arrival = predicted_arrival_battery(return_threshold(d, UAV_P, DISPATCH), d, UAV_P)
    assert arrival == pytest.approx(UAV_P.sigma_min + DISPATCH.return_buffer)


def _cell(i: int, x: float, y: float) -> Cell:
    return Cell(index=i, row=0, col=i, x=x, y=y)


def _uav_with(cells, target):
    uav = UAV(id=1, position=(0.0, 0.0))
    uav.waypoints, uav.target = list(cells), target
    return uav


def test_insertion_cost_matches_loop_distance_increase():
    a, b, c = _cell(0, 0, 0), _cell(1, 100, 0), _cell(2, 50, 30)
    assert _insertion_cost([a, b], c, 1) == pytest.approx(
        2 * (50**2 + 30**2) ** 0.5 - 100
    )
    assert _insertion_cost([a, b], c, 0) == pytest.approx(
        (50**2 + 30**2) ** 0.5
    )  # front
    assert _insertion_cost([a, b], c, 2) == pytest.approx(
        (50**2 + 30**2) ** 0.5
    )  # back
    assert _insertion_cost([], c, 0) == 0.0


def test_insertion_behind_the_target_does_not_change_it():
    uav = _uav_with([_cell(i, 10 * i, 0) for i in range(4)], target=2)
    current = uav.waypoints[uav.target]
    uav.insert_waypoint(3, _cell(99, 5, 5))
    assert uav.waypoints[uav.target] is current and uav.target == 2


def test_insertion_ahead_of_the_target_shifts_the_index_not_the_waypoint():
    uav = _uav_with([_cell(i, 10 * i, 0) for i in range(4)], target=2)
    current = uav.waypoints[uav.target]
    uav.insert_waypoint(0, _cell(99, 5, 5))
    assert uav.waypoints[uav.target] is current and uav.target == 3


def test_insertion_at_the_target_slot_is_visited_next():
    uav = _uav_with([_cell(i, 10 * i, 0) for i in range(4)], target=2)
    uav.insert_waypoint(2, _cell(99, 5, 5))
    assert uav.waypoints[uav.target].index == 99


def test_removing_inserted_waypoints_restores_original_list_and_progress():
    cells = [_cell(i, 10 * i, 0) for i in range(4)]
    uav = _uav_with(cells, target=2)
    uav.insert_waypoint(0, _cell(98, 1, 1))
    uav.insert_waypoint(4, _cell(99, 2, 2))
    next_original = uav.waypoints[uav.target]
    uav.remove_inserted_waypoints()
    assert [c.index for c in uav.waypoints] == [0, 1, 2, 3]
    assert uav.waypoints[uav.target] is next_original
    assert not uav.inserted


def test_removing_the_current_target_advances_to_the_next_waypoint():
    cells = [_cell(i, 10 * i, 0) for i in range(3)]
    uav = _uav_with(cells, target=1)
    uav.insert_waypoint(1, _cell(99, 5, 5))  # the new cell becomes the target
    assert uav.waypoints[uav.target].index == 99
    uav.remove_inserted_waypoints()
    assert uav.waypoints[uav.target].index == 1  # carries on with the original waypoint


def test_inserting_a_cell_already_on_the_route_is_rejected():
    """A duplicate would be deleted with the UAV's own waypoint on removal."""
    cells = [_cell(i, 10 * i, 0) for i in range(3)]
    uav = _uav_with(cells, target=0)
    with pytest.raises(ValueError, match="already a waypoint"):
        uav.insert_waypoint(0, cells[1])
    assert [c.index for c in uav.waypoints] == [0, 1, 2] and not uav.inserted
