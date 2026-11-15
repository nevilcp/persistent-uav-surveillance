"""Turnaround queue and reserve pools of a base."""

from dataclasses import replace

from uav_surveil.depots import Depot, reserve_pool
from uav_surveil.params import MissionParams, Parameters, UAVParams
from uav_surveil.simulation import Simulation
from uav_surveil.uav import UAV, UAVState

TAU = 60.0


def _arrivals(depot: Depot, n: int) -> list[UAV]:
    uavs = [UAV(id=i + 1, position=depot.position, battery=0.3) for i in range(n)]
    for uav in uavs:
        depot.arrive(uav, TAU)
    return uavs


def test_unlimited_slots_swap_everyone_at_once():
    depot = Depot(0, (0.0, 0.0))
    uavs = _arrivals(depot, 5)
    assert all(u.state is UAVState.SWAPPING for u in uavs)
    assert depot.waiting == 0
    depot.advance(TAU, TAU)
    assert all(u.state is UAVState.READY and u.battery == 1.0 for u in uavs)


def test_full_slots_queue_the_rest_first_come_first_served():
    depot = Depot(0, (0.0, 0.0), slots=2)
    first, second, third, fourth = _arrivals(depot, 4)
    assert [u.state for u in (first, second)] == [UAVState.SWAPPING] * 2
    assert [u.state for u in (third, fourth)] == [UAVState.QUEUED] * 2
    assert depot.waiting == 2
    depot.advance(TAU, TAU)
    assert first.state is second.state is UAVState.READY
    assert third.state is fourth.state is UAVState.SWAPPING
    assert third.swap_remaining == TAU  # a freed slot starts the next one in full
    assert depot.waiting == 0
    depot.advance(TAU, TAU)
    assert third.state is fourth.state is UAVState.READY


def test_a_single_slot_serves_in_arrival_order():
    depot = Depot(0, (0.0, 0.0), slots=1)
    uavs = _arrivals(depot, 3)
    ready_after = []
    for _ in range(3):
        depot.advance(TAU, TAU)
        ready_after.append([u.id for u in uavs if u.state is UAVState.READY])
    assert ready_after == [[1], [1, 2], [1, 2, 3]]


def test_close_records_the_time():
    depot = Depot(1, (5.0, 5.0))
    assert depot.is_open and depot.closed_at is None
    depot.close(1800.0)
    assert not depot.is_open and depot.closed_at == 1800.0


def test_reserve_pool_filters_by_base_and_kind():
    uavs = [
        UAV(id=1, position=(0.0, 0.0), home=0),
        UAV(id=2, position=(0.0, 0.0), home=1),
        UAV(id=3, position=(0.0, 0.0), home=1, is_contingency=True),
        UAV(id=4, position=(0.0, 0.0), home=1, state=UAVState.SWAPPING),
    ]
    assert [u.id for u in reserve_pool(uavs, 1, contingency=False)] == [2]
    assert [u.id for u in reserve_pool(uavs, 1, contingency=True)] == [3]
    assert [u.id for u in reserve_pool(uavs, 0, contingency=False)] == [1]


def test_a_one_slot_depot_never_serves_two_uavs_in_the_same_step():
    params = Parameters(
        uav=UAVParams(depot_slots=1), mission=MissionParams(duration=3000.0)
    )
    sim = Simulation(params)
    depot = sim.depots[0]
    for _ in range(3000):
        sim.step()
        assert len(depot.in_service) <= 1
        queued = [u for u in sim.uavs if u.state is UAVState.QUEUED]
        assert len(queued) == depot.waiting
    assert all(u.state is not UAVState.QUEUED or u in depot.queue for u in sim.uavs)


def test_default_depot_has_unlimited_slots():
    sim = Simulation(replace(Parameters(), mission=MissionParams(duration=10.0)))
    assert sim.depots[0].slots == 0 and sim.depots[0].is_open
