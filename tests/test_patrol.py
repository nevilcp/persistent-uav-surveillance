"""Shared-loop simulation: deployment, hand-overs, re-spacing, spares and closure."""

from dataclasses import replace

import pytest

from uav_surveil import LoopSimulation, Parameters, Simulation, make_simulation
from uav_surveil.params import (
    FailureParams,
    MissionParams,
    PatrolParams,
    RegionParams,
    UAVParams,
)
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
    patrol=PatrolParams(mode="shared_loop"),
    mission=MissionParams(duration=1000.0),
)
# one base and no zones keep the hand-over tests quick; T_resp covers the far corner
SMALL = Parameters(
    uav=UAVParams(depot_slots=2),
    patrol=PatrolParams(
        mode="shared_loop", response_time=400.0, respace_gain_time=100.0
    ),
    mission=MissionParams(duration=1500.0),
)


def spacing(sim: LoopSimulation) -> list[float]:
    """Arc gaps between consecutive looping UAVs."""
    arcs = sorted(u.arc for u in sim.uavs if u.state is UAVState.ON_ROUTE)
    length = sim.loop.length
    return [(b - a) % length for a, b in zip(arcs, arcs[1:] + arcs[:1])]


@pytest.fixture(scope="module")
def port() -> LoopSimulation:
    sim = LoopSimulation(PORT)
    sim.run()
    return sim


def test_make_simulation_follows_the_patrol_mode():
    assert isinstance(make_simulation(PORT), LoopSimulation)
    assert type(make_simulation(Parameters())) is Simulation


def test_fleet_matches_the_sizing(port):
    assert len(port.uavs) == port.split.total == 48
    assert port.split.n_fly == 39


def test_flyers_end_up_evenly_spaced_on_the_loop(port):
    gaps = spacing(port)
    assert len(gaps) == 39
    assert all(g == pytest.approx(port.loop.length / 39, abs=1.0) for g in gaps)


def test_loop_covers_every_cell_within_the_threshold_once_deployed(port):
    rows = [r for r in port.monitor.rows if r["time"] >= 450.0]
    assert all(r["max_age"] <= port.params.mission.revisit_threshold for r in rows)
    assert all(r["coverage_pct"] == 100.0 for r in rows)


def test_uavs_stay_clear_of_zones_and_keep_their_battery(port):
    sim = LoopSimulation(PORT)
    for _ in range(600):
        sim.step()
        for uav in sim.uavs:
            assert 0.0 <= uav.battery <= 1.0
            if uav.airborne:
                assert sim.airspace.signed_distance(uav.position) >= -1e-9
    summary = port.monitor.summary(None)
    assert summary["c2_battery"]["violating_steps"] == 0


def test_runs_are_deterministic():
    a, b = LoopSimulation(PORT), LoopSimulation(PORT)
    for _ in range(500):
        a.step()
        b.step()
    assert a.monitor.rows == b.monitor.rows
    assert a.events.rows == b.events.rows


def test_a_relief_takes_over_and_the_incumbent_flies_home():
    sim = LoopSimulation(SMALL)
    sim.run()
    kinds = [e[1] for e in sim.events.rows]
    assert kinds.count("handover") > sim.split.n_fly  # initial slots, then reliefs
    relieved = [e for e in sim.events.rows if e[1] == "return" and e[3] == "relieved"]
    assert relieved
    for time, _, uav_id, _ in relieved:
        # the incumbent leaves in the same step its relief takes over
        assert any(e[1] == "handover" and e[0] == time for e in sim.events.rows), uav_id
    # a UAV never flies below the reserve, relieved or not
    assert sim.monitor.summary(None)["c2_battery"]["violating_steps"] == 0


def test_respacing_never_exceeds_cruise_and_closes_a_hole():
    sim = LoopSimulation(SMALL)
    while sim.time < 700.0:
        sim.step()
    flying = [u for u in sim.uavs if u.state is UAVState.ON_ROUTE]
    n = len(flying)
    assert n == sim.split.n_fly
    flying[
        2
    ].state = UAVState.FAILED  # no alarm: the hole can only be closed by slowing
    target = sim.loop.length / (n - 1)
    for _ in range(700):
        sim.step()
        assert all(s <= SMALL.uav.speed for s in sim._speed.values())
    gaps = spacing(sim)
    assert len(gaps) == n - 1
    assert all(g == pytest.approx(target, rel=0.05) for g in gaps)


def test_a_breakdown_launches_a_spare_that_joins_the_loop():
    params = replace(SMALL, failure=FailureParams(enabled=True, route=3, time=700.0))
    sim = LoopSimulation(params)
    while sim.time < 1100.0:
        sim.step()
    failed = [u for u in sim.uavs if u.state is UAVState.FAILED]
    assert len(failed) == 1
    launches = [e for e in sim.events.rows if e[1] == "launch" and "spare" in e[3]]
    assert len(launches) == 1 and launches[0][0] == 700.0
    spare = sim._by_id[launches[0][2]]
    assert spare.state is UAVState.ON_ROUTE and not spare.is_contingency
    assert len([u for u in sim.uavs if u.state is UAVState.ON_ROUTE]) == sim.split.n_fly


def test_the_victim_is_the_loop_uav_nearest_the_chosen_base():
    params = replace(
        PORT,
        failure=FailureParams(enabled=True, time=600.0, near_depot=1),
    )
    sim = LoopSimulation(params)
    while sim.time < 600.0:
        sim.step()
    nearest = min(
        (u for u in sim.uavs if u.state is UAVState.ON_ROUTE),
        key=lambda u: sim.airspace.depot_distance(u.position, 1),
    )
    sim.step()
    assert nearest.state is UAVState.FAILED
    assert sim.breakdowns == [(600.0, nearest.id)]


def test_closing_a_base_reroutes_the_returns():
    params = replace(
        PORT,
        failure=FailureParams(closed_depot=2, closure_time=500.0),
    )
    sim = LoopSimulation(params)
    returning_home = set()
    while sim.time < 1200.0:
        sim.step()
        if sim.time > 500.0:
            returning_home |= {
                u.home for u in sim.uavs if u.state is UAVState.RETURNING
            }
    assert not sim.depots[2].is_open and sim.depots[2].closed_at == 500.0
    assert 2 not in returning_home
    assert any(e[1] == "closure" for e in sim.events.rows)
    assert any(e[1] == "resize" for e in sim.events.rows)


def test_port_dispatch_safety_over_a_full_mission():
    from scripts.run_port_study import runs

    sim = LoopSimulation(runs()['R4'])
    summary = sim.run()
    c2 = summary['c2_battery']['violating_steps']
    c5 = summary['c5_way_home']['violating_steps']
    assert c2 == c5 == summary['c7_zone_clearance']['violating_steps'] == 0


def test_straight_return_rule_can_exhaust_the_reserve_in_the_pocket():
    from scripts.run_port_study import runs
    from uav_surveil.dispatch import return_threshold

    sim = LoopSimulation(runs()['R0'])
    uav = sim.uavs[0]
    uav.position = (920.0, 760.0)
    _, straight = sim._nearest_base(uav)
    uav.battery = return_threshold(straight, sim.params.uav, sim.params.dispatch)
    sim._send_home(uav, 'pocket return')
    for _ in range(200):
        sim.step()
    assert sim.monitor.summary(None)['c2_battery']['violating_steps'] > 0
