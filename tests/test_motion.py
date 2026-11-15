"""Legs, detours and the choice of base."""

from dataclasses import replace

import pytest

from uav_surveil.params import (
    DispatchParams,
    FleetParams,
    MissionParams,
    Parameters,
    RegionParams,
)
from uav_surveil.simulation import Simulation
from uav_surveil.uav import UAV

WALL = (-300.0, -100.0, -200.0, 100.0)  # hides the cells nearest the depot
NOMINAL = Parameters()
SHORT = MissionParams(duration=900.0)


def test_fly_leg_carries_the_step_over_a_detour_vertex():
    uav = UAV(id=1, position=(0.0, 0.0))
    uav.set_leg((3.0, 4.0), [(3.0, 0.0), (3.0, 4.0)])
    flown, arrived = uav.fly_leg(6.0)
    assert (flown, arrived) == (6.0, False)
    assert uav.position == pytest.approx((3.0, 3.0))
    flown, arrived = uav.fly_leg(6.0)
    assert (flown, arrived) == (1.0, True)
    assert uav.position == (3.0, 4.0)


def test_launch_and_clear_route_reset_the_leg():
    uav = UAV(id=1, position=(0.0, 0.0))
    uav.set_leg((1.0, 1.0), [(1.0, 1.0)])
    uav.clear_route()
    assert uav.leg == [] and uav.leg_goal is None


def test_ferry_detours_around_a_zone_and_costs_more_battery():
    region = replace(NOMINAL.region, zones=(WALL,))
    walled = Simulation(replace(NOMINAL, region=region, mission=SHORT))
    open_sky = Simulation(replace(NOMINAL, mission=SHORT))
    for sim in (walled, open_sky):
        for _ in range(240):
            sim.step()
    first = {sim: sim.uavs[0] for sim in (walled, open_sky)}
    assert first[walled].battery < first[open_sky].battery
    assert walled.airspace.signed_distance(first[walled].position) >= 0.0


def test_no_uav_enters_a_zone_over_a_whole_run():
    region = replace(NOMINAL.region, zones=(WALL,))
    sim = Simulation(replace(NOMINAL, region=region, mission=SHORT))
    for _ in range(900):
        sim.step()
        for uav in sim.uavs:
            if uav.airborne:
                assert sim.airspace.signed_distance(uav.position) >= -1e-6


def test_returning_uav_remembers_the_base_it_flies_to():
    sim = Simulation(replace(NOMINAL, mission=MissionParams(duration=2400.0)))
    sim.run()
    returned = [e for e in sim.events.rows if e[1] == "return"]
    assert returned
    assert all(u.home == 0 for u in sim.uavs)


def test_metric_picks_the_base_by_the_configured_distance():
    region = RegionParams(
        width=1200.0,
        length=1200.0,
        depots=((1240.0, 600.0), (600.0, 1240.0)),  # east, north
        zones=((960.0, 640.0, 1040.0, 1120.0), (680.0, 640.0, 960.0, 720.0)),
    )
    port = replace(
        NOMINAL,
        region=region,
        fleet=FleetParams(size=120, batch_size=27, reserve_fraction=0.05),
        mission=SHORT,
    )
    flyable = Simulation(port)
    straight = Simulation(replace(port, dispatch=DispatchParams(metric="straight")))
    drone_a = (920.0, 760.0)
    assert flyable._nearest_base(drone_a)[0] == 1  # north: 577 m against 922 m east
    assert straight._nearest_base(drone_a) == (0, pytest.approx(358.0, abs=1.0))


