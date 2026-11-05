"""Whole-loop invariants of the Ground Station Monitor."""

import math
from dataclasses import replace

import pytest

from uav_surveil.coverage import CoverageMap
from uav_surveil.monitor import Alarm, Monitor
from uav_surveil.params import FailureParams, Parameters
from uav_surveil.routing import loop_distance
from uav_surveil.simulation import Simulation
from uav_surveil.uav import UAVState

NOMINAL = Parameters()
FAILURE = replace(NOMINAL, failure=FailureParams(enabled=True, route=10, time=1800.0))


@pytest.fixture(scope="module")
def nominal() -> Simulation:
    sim = Simulation(NOMINAL)
    sim.run()
    return sim


@pytest.fixture(scope="module")
def failure() -> Simulation:
    sim = Simulation(FAILURE)
    sim.run()
    return sim


def test_freshness_uses_the_revisit_threshold():
    coverage = CoverageMap(3)
    coverage.observe(0, 100.0)
    coverage.observe(1, 40.0)  # age 160 s at t = 200
    assert coverage.n_fresh(200.0, 180.0) == 2  # never-observed cell 2 is stale
    assert coverage.n_fresh(200.0, 120.0) == 1


def test_logged_coverage_matches_the_revisit_threshold(nominal):
    sim = nominal
    fresh = sim.coverage.n_fresh(sim.time, sim.params.mission.revisit_threshold)
    assert sim.monitor.rows[-1]["coverage_pct"] == pytest.approx(100.0 * fresh / 144)
    assert sim.params.mission.revisit_threshold == 180.0


def test_every_step_keeps_the_fleet_state_consistent():
    sim = Simulation(NOMINAL)
    depot = NOMINAL.region.depot
    for _ in range(2400):
        sim.step()
        assert len(sim.uavs) == 21
        for uav in sim.uavs:
            assert 0.0 <= uav.battery <= 1.0
            if uav.state is UAVState.ON_ROUTE:
                assert uav.route is not None and 0 <= uav.target < len(uav.waypoints)
            if uav.state in (UAVState.READY, UAVState.SWAPPING) and uav.route is None:
                assert uav.position == depot and not uav.waypoints


def test_battery_floor_is_never_violated(nominal):
    """C2: every airborne UAV stays above sigma_min at every step."""
    margins = [r["battery_margin"] for r in nominal.monitor.rows]
    margins = [m for m in margins if not math.isnan(m)]
    assert min(margins) >= 0.0
    summary = nominal.monitor.summary(None)
    assert summary["c2_battery"]["violating_steps"] == 0


def test_returning_uavs_reach_the_depot_with_the_designed_margin(nominal):
    arrivals = [row for row in nominal.events.rows if row[1] == "arrive"]
    assert arrivals
    for _, _, _, detail in arrivals:
        battery = float(detail.split()[-1])
        assert battery >= nominal.params.uav.sigma_min


def test_every_cell_is_eventually_observed(nominal):
    assert math.isfinite(nominal.coverage.max_age(nominal.time))


def test_replacements_inherit_the_unmodified_route(nominal):
    """A UAV on route k flies exactly its cell list."""
    holders: dict[int, int] = {}
    for uav in nominal.uavs:
        if uav.state in (UAVState.ON_ROUTE, UAVState.RETURNING):
            assert uav.route is nominal.routes[uav.route.id - 1]
            assert uav.waypoints == list(uav.route.cells)  # no stray insertions
            holders[uav.route.id] = holders.get(uav.route.id, 0) + 1
    assert max(holders.values()) <= 2  # at most incumbent + replacement


def test_no_route_is_flown_by_two_uavs_for_long(nominal):
    """A route has one holder except during the brief replacement overlap."""
    mean_overlap = sum(
        r["n_on_route"] + r["n_returning"] for r in nominal.monitor.rows
    ) / len(nominal.monitor.rows)
    assert mean_overlap < 16 + 4  # n_fly plus at most the n_rotate replacements


def test_alarms_are_answered_in_order_with_nonnegative_latency(nominal):
    launched = [a for a in nominal.monitor.alarms if a.launch_time is not None]
    assert launched and all(a.latency(nominal.time) >= 0.0 for a in launched)
    assert [a.launch_time for a in launched] == sorted(a.launch_time for a in launched)


def test_simulation_is_deterministic():
    a = Simulation(
        replace(NOMINAL, mission=replace(NOMINAL.mission, duration=1500.0))
    ).run()
    b = Simulation(
        replace(NOMINAL, mission=replace(NOMINAL.mission, duration=1500.0))
    ).run()
    assert a == b


def test_c3_monitor_reports_unanswered_alarms():
    """A single rotation UAV for 15 routes runs the pool dry, so C3 must fire."""
    starved = replace(
        NOMINAL,
        fleet=replace(NOMINAL.fleet, size=17, batch_size=1),
        mission=replace(NOMINAL.mission, duration=3000.0),
    )
    summary = Simulation(starved).run()
    assert summary["c3_spare_launch"]["violations"] > 0
    assert summary["c3_spare_launch"]["max_latency_s"] > starved.uav.turnaround


def test_alarm_latency_definition():
    answered = Alarm(time=100.0, route=1, kind="rotation", launch_time=130.0)
    pending = Alarm(time=100.0, route=2, kind="rotation")
    assert answered.latency(now=500.0) == 30.0
    assert pending.latency(now=500.0) == 400.0


def test_monitor_summary_counts_violations_against_tau():
    monitor = Monitor(NOMINAL, routes=[])
    monitor.rows.append(
        {
            "time": 1.0,
            "coverage_pct": 50.0,
            "n_overdue": 72,
            "max_age": 300.0,
            "battery_margin": 0.2,
        }
    )
    monitor.alarms = [
        Alarm(0.0, 1, "rotation", launch_time=10.0),
        Alarm(0.0, 2, "rotation", launch_time=61.0),  # exceeds tau = 60 s
        Alarm(0.0, 3, "rotation"),  # still open at t = 1 s: latency 1 s
    ]
    c3 = monitor.summary(None)["c3_spare_launch"]
    assert (c3["alarms"], c3["violations"], c3["max_latency_s"]) == (
        3,
        1,
        61.0,
    )


def test_failure_sequence(failure):
    mgr, fail_time = failure.failure, 1800.0
    assert mgr.failed_uav.state is UAVState.FAILED
    assert mgr.t_fail == fail_time

    # contingency UAV launches in the same step, so the alarm latency is 0
    alarm = next(a for a in failure.monitor.alarms if a.kind == "failure")
    assert alarm.time == fail_time and alarm.latency(failure.time) == 0.0
    takeover = [
        e
        for e in failure.events.rows
        if e[1] == "launch" and e[3] == "route 10 (failure)"
    ]
    assert takeover == [(fail_time, "launch", mgr.contingency.id, "route 10 (failure)")]
    assert mgr.contingency.id == 21  # the only member of U_spare

    assert mgr.t_handover is not None and mgr.t_handover > fail_time
    assert all(not u.inserted for u in failure.uavs)
    assert all(
        u.waypoints == list(u.route.cells)
        for u in failure.uavs
        if u.route and u.airborne
    )


def test_failed_uav_stays_where_it_failed():
    sim = Simulation(FAILURE)
    for _ in range(1801):
        sim.step()
    victim = sim.failure.failed_uav
    position_at_failure = victim.position
    for _ in range(600):
        sim.step()
    assert victim.state is UAVState.FAILED and victim.position == position_at_failure


def test_orphan_cells_are_recovered_and_c4_is_evaluated(failure):
    c4 = failure.monitor.summary(failure.failure)["c4_recovery"]
    assert c4["recovery_time_s"] is not None and c4["recovery_time_s"] > 0.0
    assert c4["satisfied"] == (c4["recovery_time_s"] <= 180.0 + 120.0)


def test_failure_does_not_violate_the_battery_floor(failure):
    assert (
        failure.monitor.summary(failure.failure)["c2_battery"]["violating_steps"] == 0
    )


def test_bridging_respects_the_airborne_constraint():
    """Inserted cells never push a route past the usable endurance."""
    sim = Simulation(FAILURE)
    limit = sim.params.uav.usable_endurance
    bridged = False
    for _ in range(2400):
        sim.step()
        for uav in sim.uavs:
            if uav.state is UAVState.ON_ROUTE:
                bridged |= bool(uav.inserted)
                t_airborne = (
                    2.0 * sim.ferry_time
                    + loop_distance(uav.waypoints) / NOMINAL.uav.speed
                )
                assert t_airborne <= limit
    assert bridged  # guards against a vacuous pass


def test_bridging_uses_surviving_uavs_on_other_routes_only():
    """Bridging never uses the failed route's own holder."""
    sim = Simulation(FAILURE)
    failed_route = FAILURE.failure.route
    assignees: set[int] = set()
    for _ in range(2400):
        sim.step()
        for uav in sim.uavs:
            if uav.inserted:
                assert uav.route.id != failed_route
                assignees.add(uav.id)
    assert assignees  # guards against a vacuous pass
    assert sim.failure.contingency.id not in assignees


def test_contingency_uav_inherits_the_full_failed_route():
    """The contingency UAV carries the full failed route throughout."""
    sim = Simulation(FAILURE)
    for _ in range(2400):
        sim.step()
        mgr = sim.failure
        spare = mgr.contingency
        if spare is None or spare.route is None:
            continue
        assert spare.route is sim.routes[FAILURE.failure.route - 1]
        assert {c.index for c in mgr.route.cells} <= {c.index for c in spare.waypoints}
        if mgr.t_handover is not None and spare.state is UAVState.ON_ROUTE:
            assert spare.waypoints == list(spare.route.cells)
    assert sim.failure.t_handover is not None
