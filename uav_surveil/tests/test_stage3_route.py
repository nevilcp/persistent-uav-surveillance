from uav_surveil.config import load_scenario
from uav_surveil.stage1_grid import build_grid_from_config
from uav_surveil.stage3_route import generate_routes_from_config
from uav_surveil.stage3_route_roundrobin import (
    generate_routes_roundrobin,
    _serpentine_order,
    _pick_seeds,
    _route_time,
)


def test_route_generation_counts():
    cfg = load_scenario("baseline")
    cells = build_grid_from_config(cfg)
    n_launch = 4

    routes, summary = generate_routes_from_config(cfg, cells, n_launch)

    assert len(routes) == n_launch
    assert summary.n_routes == n_launch
    # Check that every cell appears exactly once across all routes
    cell_ids = {c.id for c in cells}
    routed_ids = {cid for r in routes for cid in r.cell_sequence}
    assert cell_ids == routed_ids


class TestRoundRobinSerpentine:
    """Tests for the serpentine sweep + seeded round-robin + smoothing
    Stage 3A implementation (architecture.md §4.1 / thesis Alg. 5)."""

    def _baseline(self):
        cfg = load_scenario("baseline")
        cells = build_grid_from_config(cfg)
        depot = (cfg.mission.depot_x, cfg.mission.depot_y)
        return cfg, cells, depot

    def test_route_counts_balanced(self):
        """No route is empty or a micro-route; every route stays within a
        reasonable band of the mean cell count (smoothing trades some
        count-balance for loop-time balance, but shouldn't produce
        degenerate routes)."""
        cfg, cells, depot = self._baseline()
        n_launch = 16
        routes, _ = generate_routes_roundrobin(
            cells,
            n_launch,
            cfg.uav.cruise_speed,
            depot,
            False,
            cfg.battery.usable_endurance,
        )
        assert len(routes) == n_launch
        lengths = [len(r.cell_sequence) for r in routes]
        avg = len(cells) / n_launch
        assert min(lengths) >= max(2, int(0.5 * avg))
        assert max(lengths) <= int(2 * avg) + 2

    def test_every_cell_assigned_exactly_once(self):
        cfg, cells, depot = self._baseline()
        n_launch = 16
        routes, _ = generate_routes_roundrobin(
            cells,
            n_launch,
            cfg.uav.cruise_speed,
            depot,
            True,
            cfg.battery.usable_endurance,
        )
        cell_ids = [c.id for c in cells]
        routed_ids = [cid for r in routes for cid in r.cell_sequence]
        assert sorted(routed_ids) == sorted(cell_ids)

    def test_smoothing_reduces_loop_time_std(self):
        cfg, cells, depot = self._baseline()
        n_launch = 16
        speed = cfg.uav.cruise_speed
        buse = cfg.battery.usable_endurance

        # Reconstruct the pre-smoothing (raw round-robin) assignment using
        # the same best-of-(sweep, nearest-neighbour) timing the smoothed
        # result is judged by, for an apples-to-apples comparison.
        sweep = _serpentine_order(cells, depot)
        offset = _pick_seeds(sweep, n_launch, depot, True)
        raw = [[] for _ in range(n_launch)]
        for i, cell in enumerate(sweep):
            raw[(i + offset) % n_launch].append(cell)
        unsmoothed_times = [_route_time(r, depot, speed) for r in raw]
        unsmoothed_std = (
            sum((t - sum(unsmoothed_times) / n_launch) ** 2 for t in unsmoothed_times)
            / n_launch
        ) ** 0.5

        routes, summary = generate_routes_roundrobin(
            cells, n_launch, speed, depot, True, buse
        )
        assert summary["loop_time_std"] <= unsmoothed_std + 1e-9

    def test_deterministic(self):
        cfg, cells, depot = self._baseline()
        n_launch = 16
        args = (
            cells,
            n_launch,
            cfg.uav.cruise_speed,
            depot,
            True,
            cfg.battery.usable_endurance,
        )

        routes_a, _ = generate_routes_roundrobin(*args)
        routes_b, _ = generate_routes_roundrobin(*args)

        assert [r.cell_sequence for r in routes_a] == [
            r.cell_sequence for r in routes_b
        ]

    def test_all_routes_within_usable_endurance(self):
        cfg, cells, depot = self._baseline()
        n_launch = 16
        buse = cfg.battery.usable_endurance
        routes, _ = generate_routes_roundrobin(
            cells, n_launch, cfg.uav.cruise_speed, depot, True, buse
        )
        for route in routes:
            assert route.loop_time <= buse
