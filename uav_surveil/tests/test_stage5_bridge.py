"""Tests for WP4: Stage 5 bridging K cap, best-index insertion, and the
detour-ratio geometry guard (architecture.md §4.4)."""

from math import hypot

from uav_surveil.config import get_test_config
from uav_surveil.core.cell import Cell
from uav_surveil.core.route import Route
from uav_surveil.core.uav import UAV, UAVState
from uav_surveil.stage5_failure import BridgeManager


class _FakeMetrics:
    def __init__(self, t: float = 100.0):
        self.current_time = t
        self.bridge_inserts_count = 0


class _FakeSim:
    """Minimal duck-typed stand-in exposing just what BridgeManager needs."""

    def __init__(self):
        self.config = get_test_config()
        self.config.stl.max_revisit_gap = 120.0
        self.metrics = _FakeMetrics()
        self._claimed_cells = set()
        self._cell_claim_times = {}
        self.uavs = []
        self.cell_lookup = {}

    def _usable_range_m(self) -> float:
        return self.config.uav.cruise_speed * self.config.battery.usable_endurance


def _make_failed(sim, cell_ids):
    route = Route(id="f_route", cell_sequence=cell_ids, loop_time=200.0)
    failed = UAV(
        id="F", x=0.0, y=0.0, soc=0.5, route_list=[route], state=UAVState.FAILED
    )
    failed.is_failed = True
    failed._waypoint_idx = 0
    failed.tail_index_at_failure = 0
    sim.uavs.append(failed)
    return failed


def _make_active(sim, uid, x, y, waypoints, loop_time=100.0):
    route = Route(
        id=f"{uid}_route",
        cell_sequence=[f"{uid}{i}" for i in range(2)],
        loop_time=loop_time,
    )
    uav = UAV(id=uid, x=x, y=y, soc=1.0, route_list=[route], state=UAVState.ON_MISSION)
    uav._waypoints = list(waypoints)
    uav._waypoint_idx = 0
    sim.uavs.append(uav)
    return uav


class TestBridgeKCap:
    def test_at_most_k_inserts_per_tick(self):
        sim = _FakeSim()
        sim.config.failure.bridge_policy.realloc_k = 2

        cells = [Cell(id=f"c{i}", x=100.0 * i, y=0.0) for i in range(1, 6)]
        for c in cells:
            sim.cell_lookup[c.id] = c
        _make_failed(sim, [c.id for c in cells])

        # Two active UAVs that can each reach any of the cells cheaply
        # (max_inserts_per_uav=1 by default, so a single UAV alone would be
        # capped before K), so K is the only thing limiting total inserts.
        _make_active(
            sim,
            "A",
            x=0.0,
            y=0.0,
            waypoints=[
                (100.0, 0.0),
                (200.0, 0.0),
                (300.0, 0.0),
                (400.0, 0.0),
                (500.0, 0.0),
            ],
            loop_time=1000.0,
        )
        _make_active(
            sim,
            "B",
            x=0.0,
            y=0.0,
            waypoints=[
                (100.0, 0.0),
                (200.0, 0.0),
                (300.0, 0.0),
                (400.0, 0.0),
                (500.0, 0.0),
            ],
            loop_time=1000.0,
        )

        bm = BridgeManager(sim)
        bm.enable("F")
        bm.tick()

        assert len(bm._assigned_cells["F"]) == 2
        assert sim.metrics.bridge_inserts_count == 2


class TestBridgeBestIndex:
    def test_chosen_index_minimises_delta_t(self):
        sim = _FakeSim()
        sim.config.failure.bridge_policy.realloc_k = 1

        target = Cell(id="target", x=250.0, y=50.0)
        sim.cell_lookup["target"] = target
        _make_failed(sim, ["target"])

        waypoints = [(100.0, 0.0), (200.0, 0.0), (300.0, 0.0), (400.0, 0.0)]
        a = _make_active(sim, "A", x=0.0, y=0.0, waypoints=waypoints, loop_time=500.0)

        bm = BridgeManager(sim)
        bm.enable("F")
        bm.tick()

        # Brute-force the expected minimum-cost insertion index.
        speed = sim.config.uav.cruise_speed
        best_j, best_cost = None, None
        for j in range(len(waypoints)):
            prev = (0.0, 0.0) if j == 0 else waypoints[j - 1]
            nxt = waypoints[j]
            d_pn = hypot(prev[0] - nxt[0], prev[1] - nxt[1])
            d_pc = hypot(prev[0] - target.x, prev[1] - target.y)
            d_cn = hypot(target.x - nxt[0], target.y - nxt[1])
            cost = max(0.0, d_pc + d_cn - d_pn) / speed
            if best_cost is None or cost < best_cost:
                best_j, best_cost = j, cost

        assert a._waypoints[best_j] == (target.x, target.y)
        assert len(a._waypoints) == len(waypoints) + 1


class TestBridgeDetourGuard:
    def test_detour_ratio_guard_rejects_far_cell(self):
        sim = _FakeSim()
        sim.config.failure.bridge_policy.realloc_k = 5
        sim.config.failure.bridge_policy.max_detour_ratio = 0.2

        near = Cell(id="near", x=150.0, y=5.0)
        far = Cell(id="far", x=1200.0, y=0.0)
        sim.cell_lookup["near"] = near
        sim.cell_lookup["far"] = far
        _make_failed(sim, ["far", "near"])

        # Modest loop_time -> tight detour budget (0.2 * 100s = 20s ~ 120m),
        # well inside the UAV's SoC/usable-range budget either way, so the
        # rejection is attributable to the geometry guard specifically.
        a = _make_active(
            sim,
            "A",
            x=100.0,
            y=0.0,
            waypoints=[(200.0, 0.0), (300.0, 0.0)],
            loop_time=100.0,
        )

        bm = BridgeManager(sim)
        bm.enable("F")
        bm.tick()

        assert "near" in bm._assigned_cells["F"]
        assert "far" not in bm._assigned_cells["F"]
        assert (near.x, near.y) in a._waypoints
        assert (far.x, far.y) not in a._waypoints
