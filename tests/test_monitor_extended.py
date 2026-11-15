"""Clauses C5-C7, per-breakdown C4 and the load figures of the monitor."""

import math
from dataclasses import replace

import pytest

from uav_surveil import LoopSimulation, Parameters, Simulation
from uav_surveil.airspace import Airspace
from uav_surveil.coverage import CoverageMap
from uav_surveil.depots import Depot
from uav_surveil.grid import Grid
from uav_surveil.monitor import COLUMNS, EXTENDED_COLUMNS, Monitor
from uav_surveil.params import MissionParams, PatrolParams, RegionParams, UAVParams
from uav_surveil.uav import UAV, UAVState

REGION = RegionParams(
    width=1200.0,
    length=1200.0,
    depots=((-40.0, 600.0), (1240.0, 600.0), (600.0, 1240.0)),
    zones=((200.0, 520.0, 520.0, 680.0),),
)
PARAMS = Parameters(
    region=REGION,
    uav=UAVParams(depot_slots=1),
    patrol=PatrolParams(mode="shared_loop"),
    mission=MissionParams(duration=600.0),
)


def bench() -> tuple[Monitor, list[Depot], CoverageMap]:
    air = Airspace.from_region(Grid(REGION), REGION)
    depots = [Depot(m, pos, 1) for m, pos in enumerate(air.depots)]
    return Monitor(PARAMS, [], air, depots), depots, CoverageMap(900)


def test_paper_runs_keep_their_columns_and_summary():
    sim = Simulation(replace(Parameters(), mission=MissionParams(duration=60.0)))
    summary = sim.run()
    assert not sim.monitor.extended
    assert set(sim.monitor.rows[0]) == set(COLUMNS)
    assert "passed" not in summary and "c5_way_home" not in summary


def test_a_loop_run_reports_every_extended_clause():
    sim = LoopSimulation(PARAMS)
    summary = sim.run()
    assert set(sim.monitor.rows[0]) == set(EXTENDED_COLUMNS)
    assert summary["c5_way_home"]["violating_steps"] == 0
    assert summary["c7_zone_clearance"]["violating_steps"] == 0
    assert summary["c7_zone_clearance"]["min_clearance_m"] >= 0.0
    assert 0.0 < summary["transit_share"] < 1.0
    assert len(summary["busiest_moment"]) == 3
    assert summary["passed"] is (summary["c6_base_queue"]["violating_steps"] == 0)


def test_c5_flags_a_uav_that_cannot_reach_a_base():
    monitor, _, coverage = bench()
    low = UAV(id=1, position=(600.0, 300.0), battery=0.12, state=UAVState.RETURNING)
    monitor.record(1.0, [low], coverage)
    assert monitor.rows[0]["way_home_margin"] < 0.0
    assert monitor.summary(None)["c5_way_home"]["violating_steps"] == 1
    assert monitor.summary(None)["passed"] is False


def test_c5_measures_the_way_home_around_a_zone():
    monitor, _, coverage = bench()
    uav = UAV(id=1, position=(600.0, 600.0), battery=1.0, state=UAVState.ON_ROUTE)
    monitor.record(1.0, [uav], coverage)
    # the west base is 640 m away in a straight line, which the zone does not block here
    expected = 1.0 - 640.0 / PARAMS.uav.range - PARAMS.uav.sigma_min
    assert monitor.rows[0]["way_home_margin"] == pytest.approx(expected, abs=0.01)


def test_c7_is_negative_inside_a_zone():
    monitor, _, coverage = bench()
    inside = UAV(id=1, position=(300.0, 600.0), state=UAVState.ON_ROUTE)
    monitor.record(1.0, [inside], coverage)
    assert monitor.rows[0]["zone_clearance"] == pytest.approx(-80.0)
    assert monitor.summary(None)["c7_zone_clearance"]["violating_steps"] == 1


def test_c6_counts_the_uavs_waiting_beyond_the_slots():
    monitor, depots, coverage = bench()
    swappers = [UAV(id=i, position=depots[0].position) for i in (1, 2, 3)]
    for uav in swappers:
        depots[0].arrive(uav, 60.0)
    monitor.record(1.0, swappers, coverage)
    assert monitor.rows[0]["max_base_queue"] == 1  # two wait for one slot
    for t in range(2, 62):
        depots[0].advance(1.0, 60.0)
        monitor.record(float(t), swappers, coverage)
    summary = monitor.summary(None)
    assert summary["c6_base_queue"]["max_excess"] == 1
    assert summary["c6_base_queue"]["min_margin"] == -1
    assert summary["longest_queue_wait_s"] >= 60.0
    assert summary["busiest_moment"][0]["present"] == 3
    assert summary["busiest_moment"][0]["max_waiting"] == 2


def test_clauses_without_a_subject_are_blank():
    monitor, _, coverage = bench()
    monitor.record(1.0, [], coverage)
    row = monitor.rows[0]
    assert math.isnan(row["way_home_margin"]) and math.isnan(row["zone_clearance"])
    assert monitor.summary(None)["c5_way_home"]["min_margin"] is None


def test_c4_per_breakdown_runs_to_the_next_breakdown():
    monitor, _, _ = bench()
    # stale from 100 s to 249 s, fresh again from 250 s; second breakdown never recovers
    ages = {t: (300.0 if 100 <= t < 250 else 100.0) for t in range(600)}
    for t in range(450, 600):
        ages[t] = 300.0
    monitor.rows = [{"time": float(t), "max_age": a} for t, a in ages.items()]
    first, second = monitor._recoveries([(100.0, 7), (450.0, 9)])
    assert first["recovery_time_s"] == 150.0 and first["satisfied"] is True
    assert second["recovery_time_s"] is None and second["satisfied"] is False


def test_c4_is_zero_when_the_slot_is_refilled_before_anything_goes_stale():
    monitor, _, _ = bench()
    monitor.rows = [{"time": float(t), "max_age": 160.0} for t in range(300)]
    assert monitor._recoveries([(100.0, 7)])[0]["recovery_time_s"] == 0.0


def test_c4_reports_first_restoration_even_if_coverage_later_drops():
    monitor, _, _ = bench()
    monitor.rows = [
        {"time": 100.0, "max_age": 300.0},
        {"time": 150.0, "max_age": 160.0},
        {"time": 200.0, "max_age": 300.0},
    ]
    assert monitor._recoveries([(100.0, 7)])[0]['recovery_time_s'] == 50.0
