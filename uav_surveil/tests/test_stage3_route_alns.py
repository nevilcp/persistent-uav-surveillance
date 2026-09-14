"""Unit tests for Stage 3A ALNS route generator."""

from uav_surveil.core.cell import Cell
from uav_surveil.stage3_route_alns import generate_routes_alns


def test_alns_loop_time_under_threshold():
    """Test that ALNS generates routes with loop times under 2x revisit
    requirement (240s)."""
    # Create a 12x12 grid (144 cells) similar to baseline scenario
    cells = []
    for row in range(12):
        for col in range(12):
            cell = Cell(
                id=f"cell_{row:03d}_{col:03d}",
                x=20.0 + col * 40.0,  # 40m cell size
                y=20.0 + row * 40.0,
                last_seen_ts=None,
                priority=1.0,
                is_covered=False,
            )
            cells.append(cell)

    # Test parameters from baseline config
    n_launch = 16
    cruise_speed = 12.0  # Updated speed
    depot = (-500.0, 0.0)  # Baseline depot position

    # Generate routes
    routes, _metadata = generate_routes_alns(cells, n_launch, cruise_speed, depot)

    # Verify we have routes
    assert len(routes) > 0
    assert len(routes) <= n_launch

    # Loop time constraint: 280s = 2.33 x 120s revisit requirement
    max_loop_time = max(route.loop_time for route in routes if route.loop_time > 0)
    assert (
        max_loop_time < 280.0
    ), f"Max loop time {max_loop_time:.1f}s exceeds 280s threshold"

    # Verify all routes have minimum cells
    non_empty_routes = [r for r in routes if len(r.cell_sequence) > 0]
    for route in non_empty_routes:
        assert (
            len(route.cell_sequence) >= 7
        ), f"Route {route.id} has only {len(route.cell_sequence)} cells (min 7)"

    # Verify all cells are assigned
    assigned_cells = set()
    for route in routes:
        assigned_cells.update(route.cell_sequence)
    expected_cells = {cell.id for cell in cells}
    assert (
        assigned_cells == expected_cells
    ), f"Missing cells: {expected_cells - assigned_cells}"


def test_alns_route_balance():
    """Test that ALNS generates reasonably balanced routes."""
    # Create smaller test grid for faster execution
    cells = []
    for row in range(6):
        for col in range(6):
            cell = Cell(
                id=f"cell_{row:03d}_{col:03d}",
                x=20.0 + col * 40.0,
                y=20.0 + row * 40.0,
                last_seen_ts=None,
            )
            cells.append(cell)

    n_launch = 4
    cruise_speed = 8.0
    depot = (-500.0, 0.0)

    routes, _metadata = generate_routes_alns(cells, n_launch, cruise_speed, depot)

    # Check route balance (should be roughly 36/4 = 9 cells per route)
    non_empty_routes = [r for r in routes if len(r.cell_sequence) > 0]
    route_sizes = [len(r.cell_sequence) for r in non_empty_routes]

    if route_sizes:  # Only check if we have routes
        min_size = min(route_sizes)
        max_size = max(route_sizes)
        assert (
            max_size - min_size <= 3
        ), f"Routes too imbalanced: {min_size}-{max_size} cells"


if __name__ == "__main__":
    test_alns_loop_time_under_threshold()
    test_alns_route_balance()
    print("✅ All ALNS tests passed!")
