"""Tests for WP3: route phase preservation across spare launches and swaps
(architecture.md §4.2 / §2.1; thesis Alg. 6)."""

from uav_surveil.config import load_scenario
from uav_surveil.gss.simulation import GSSSimulation
from uav_surveil.core.uav import UAVState


def _initialized_sim():
    cfg = load_scenario("baseline")
    sim = GSSSimulation(config=cfg)
    assert sim.initialize() is True
    return sim


def _run_until_progress(sim, uav, min_waypoints=2, max_ticks=600):
    for _ in range(max_ticks):
        sim.step()
        if getattr(uav, "_waypoint_idx", 0) >= min_waypoints:
            return
    raise AssertionError(f"{uav.id} never reached {min_waypoints} waypoints")


class TestSpareInheritsPhase:
    def test_spare_inherits_phase_offset_and_route(self):
        sim = _initialized_sim()
        parent = next(u for u in sim.uavs if u.id == "01")
        _run_until_progress(sim, parent)

        spare = sim._launch_spare_for(parent)

        assert spare is not None
        assert spare.phase_offset == parent.phase_offset
        assert spare.route_list[0].id == parent.route_list[0].id

    def test_spare_resumes_at_parents_progress(self):
        sim = _initialized_sim()
        parent = next(u for u in sim.uavs if u.id == "01")
        _run_until_progress(sim, parent, min_waypoints=3)

        parent_progress = parent._waypoint_idx
        spare = sim._launch_spare_for(parent)

        assert spare is not None
        assert spare._waypoint_idx == parent_progress


class TestRelaunchIsPhaseCongruent:
    def test_relaunch_time_congruent_to_phase_mod_t_cyc(self):
        sim = _initialized_sim()
        parent = next(u for u in sim.uavs if u.id == "01")
        _run_until_progress(sim, parent)

        # Free up a rotation spare slot and force the parent through a swap.
        sim._launch_spare_for(parent)
        parent.state = UAVState.SWAPPING
        parent.swap_timer = 1.0
        sim.step()

        assert parent.state == UAVState.IDLE
        assert parent.phase_offset is not None
        assert sim._t_cyc > 0
        diff = (parent.launch_time - parent.phase_offset) % sim._t_cyc
        assert diff == 0.0 or abs(diff - sim._t_cyc) < 1e-6

    def test_swap_does_not_relaunch_immediately_when_phase_is_later(self):
        sim = _initialized_sim()
        parent = next(u for u in sim.uavs if u.id == "01")
        _run_until_progress(sim, parent)

        sim._launch_spare_for(parent)
        parent.state = UAVState.SWAPPING
        parent.swap_timer = 1.0
        now_before = sim.metrics.current_time
        sim.step()

        # The parent must not jump straight back onto the route it just
        # handed to a spare -- it should wait for its own phase slot.
        assert parent.state == UAVState.IDLE
        if parent.launch_time not in (float("inf"),):
            assert parent.launch_time >= now_before
