"""Stage 3A: ALNS (Adaptive Large Neighborhood Search) route generator.

Optimizes UAV routes using destroy/repair operators to achieve better coverage
balance and shorter loop times compared to KMNN.
"""

from __future__ import annotations

import time
from collections.abc import Sequence
from math import hypot

import numpy as np

from .core.cell import Cell
from .core.route import Route

try:
    from alns import ALNS
    from alns.accept import SimulatedAnnealing
    from alns.select import RouletteWheel
    from alns.stop import MaxIterations

    ALNS_AVAILABLE = True
except ImportError:
    ALNS_AVAILABLE = False


__all__ = ["generate_routes_alns"]


MIN_CELLS_PER_ROUTE = 7


def _merge_small_routes(route_list: list[Route], n_launch: int) -> list[Route]:
    """Ensure each route has at least MIN_CELLS_PER_ROUTE cells by redistributing."""
    routes = route_list.copy()
    orphan_cells = []
    for r in routes[:]:
        if len(r.cell_sequence) < MIN_CELLS_PER_ROUTE:
            orphan_cells.extend(r.cell_sequence)
            routes.remove(r)
    # attach orphans round-robin to shortest routes
    routes.sort(key=lambda rt: len(rt.cell_sequence))
    for idx, cid in enumerate(orphan_cells):
        routes[idx % len(routes)].cell_sequence.append(cid)
    # if still fewer than n_launch, create empty placeholders
    while len(routes) < n_launch:
        routes.append(
            Route(id=f"alns_empty_{len(routes)}", cell_sequence=[], loop_time=0.0)
        )
    return routes


def generate_routes_alns(
    cells: Sequence[Cell],
    n_launch: int,
    cruise_speed: float,
    depot: tuple[float, float] = (0.0, 0.0),
    max_iterations: int = 2000,
) -> tuple[list[Route], object]:
    """Generate routes using Adaptive Large Neighbourhood Search (ALNS)."""

    if not ALNS_AVAILABLE:
        print("⚠️ ALNS package not available, falling back to KMNN")
        from .stage3_route_kmnn import generate_routes_kmnn

        return generate_routes_kmnn(cells, n_launch, cruise_speed, depot)

    print(f"🔄 ALNS: Optimizing {len(cells)} cells → {n_launch} routes")
    start_time = time.time()

    try:
        # Create initial solution using Round-Robin as starting point (better balance than KMNN)
        from .stage3_route_roundrobin import generate_routes_roundrobin

        initial_routes, _ = generate_routes_roundrobin(
            cells, n_launch, cruise_speed, depot, furthest_first=False
        )
        initial_routes = _merge_small_routes(initial_routes, n_launch)

        # Convert routes to solution format (list of cell indices)
        cell_to_idx = {cell.id: idx for idx, cell in enumerate(cells)}
        initial_solution = []
        for route in initial_routes:
            route_indices = [cell_to_idx[cell_id] for cell_id in route.cell_sequence]
            initial_solution.append(route_indices)

        # Create ALNS state class for our problem
        class ALNSState:
            def __init__(self, routes):
                self.routes = routes

            def copy(self):
                return ALNSState([route.copy() for route in self.routes])

            def objective(self):
                return _calculate_objective(self.routes, cells, cruise_speed, depot)

        # Set up ALNS with random seed
        rng = np.random.default_rng(1234)
        alns = ALNS(rng)

        # Add operators (we have 1 destroy and 1 repair operator)
        alns.add_destroy_operator(_simple_removal)
        alns.add_repair_operator(_simple_insertion)

        # Create initial state
        initial_state = ALNSState(initial_solution)

        # Set up ALNS components with correct parameters
        # RouletteWheel expects (weights, decay, num_destroy, num_repair)
        select = RouletteWheel([25, 5, 1, 0], 0.8, 1, 1)  # 1 destroy, 1 repair operator
        # Use Simulated Annealing for better exploration (start temp = 10% of initial objective)
        initial_obj = initial_state.objective()
        start_temp = max(0.1 * initial_obj, 10.0)  # Minimum temperature of 10
        accept = SimulatedAnnealing(
            start_temp, 1.0, 0.98
        )  # Cool down to temp=1, decay=0.98
        stop = MaxIterations(max_iterations)

        # Run ALNS optimization
        print(f"🔧 ALNS starting with initial state type: {type(initial_state)}")
        print(f"🔧 Initial state has routes: {hasattr(initial_state, 'routes')}")
        result = alns.iterate(initial_state, select, accept, stop)
        print(f"🔧 ALNS result type: {type(result.best_state)}")
        print(
            f"🔧 Result best_state has routes: {hasattr(result.best_state, 'routes')}"
        )

        # Convert optimized solution back to routes
        try:
            # The ALNS library returns the state object we created
            if hasattr(result.best_state, "routes"):
                optimized_solution = result.best_state.routes
                initial_obj = initial_state.objective()
                final_obj = result.best_state.objective()
                improvement = initial_obj - final_obj
            else:
                # If best_state is just the raw solution
                optimized_solution = result.best_state
                initial_obj = initial_state.objective()
                final_obj = _calculate_objective(
                    optimized_solution, cells, cruise_speed, depot
                )
                improvement = initial_obj - final_obj
        except (AttributeError, TypeError) as e:
            # Fallback if result.best_state doesn't have expected attributes
            print(f"⚠️ ALNS result format unexpected ({e}), using initial solution")
            optimized_solution = initial_solution
            improvement = 0.0

        routes = _solution_to_routes(optimized_solution, cells, depot, cruise_speed)
        routes = _merge_small_routes(routes, n_launch)

        elapsed = time.time() - start_time
        print(f"✅ ALNS completed in {elapsed:.1f}s, improved by {improvement:.1f}s")

        return routes, {
            "algorithm": "alns",
            "runtime": elapsed,
            "improvement": improvement,
        }

    except Exception as e:  # noqa: BLE001 - ALNS library errors vary, fall back to KMNN
        print(f"⚠️ ALNS failed ({e}), using KMNN fallback")
        from .stage3_route_kmnn import generate_routes_kmnn

        return generate_routes_kmnn(cells, n_launch, cruise_speed, depot)


def _simple_removal(state, rng):
    """Remove random cells from random routes."""
    destroyed = state.copy()

    # Remove 20-30% of cells
    total_cells = sum(len(route) for route in destroyed.routes)
    remove_count = max(1, int(0.25 * total_cells))

    for _ in range(remove_count):
        # Find non-empty routes
        non_empty = [i for i, route in enumerate(destroyed.routes) if route]
        if not non_empty:
            break

        # Remove random cell from random route
        route_idx = rng.choice(non_empty)
        cell_idx = rng.integers(0, len(destroyed.routes[route_idx]))
        destroyed.routes[route_idx].pop(cell_idx)

    return destroyed


def _simple_insertion(destroyed, rng):
    """Insert removed cells greedily into best positions."""
    repaired = destroyed.copy()

    # Find all cells that should be assigned (0 to max_cell_idx)
    assigned_cells = set()
    for route in repaired.routes:
        assigned_cells.update(route)

    max_cell_idx = max(max(route) if route else -1 for route in repaired.routes)
    if max_cell_idx >= 0:
        all_cells = set(range(max_cell_idx + 1))
        missing_cells = list(all_cells - assigned_cells)

        # Distribute missing cells evenly
        for i, cell_idx in enumerate(missing_cells):
            route_idx = i % len(repaired.routes)
            repaired.routes[route_idx].append(cell_idx)

    return repaired


def _calculate_objective(
    solution: list[list[int]],
    cells: Sequence[Cell],
    cruise_speed: float,
    depot: tuple[float, float] = (-500.0, 0.0),
) -> float:
    """Calculate the objective function (total route time)."""
    total_time = 0.0
    depot_x, depot_y = depot[0], depot[1]  # Use actual depot coordinates

    for route in solution:
        if not route:
            continue

        route_distance = 0.0

        # Distance from depot to first cell
        first_cell = cells[route[0]]
        route_distance += hypot(first_cell.x - depot_x, first_cell.y - depot_y)

        # Distance between consecutive cells
        for i in range(len(route) - 1):
            cell1 = cells[route[i]]
            cell2 = cells[route[i + 1]]
            route_distance += hypot(cell2.x - cell1.x, cell2.y - cell1.y)

        # Distance from last cell back to depot
        last_cell = cells[route[-1]]
        route_distance += hypot(depot_x - last_cell.x, depot_y - last_cell.y)

        total_time += route_distance / cruise_speed

    return total_time


def _solution_to_routes(
    solution: list[list[int]],
    cells: Sequence[Cell],
    depot: tuple[float, float],
    cruise_speed: float,
) -> list[Route]:
    """Convert ALNS solution back to Route objects."""
    routes = []

    for i, route_indices in enumerate(solution):
        if not route_indices:  # Skip empty routes
            continue

        # Convert indices back to cell IDs
        cell_sequence = [cells[idx].id for idx in route_indices]

        # Calculate route time
        route_distance = 0.0

        # From depot to first cell
        if route_indices:
            first_cell = cells[route_indices[0]]
            route_distance += hypot(first_cell.x - depot[0], first_cell.y - depot[1])

            # Between cells
            for j in range(len(route_indices) - 1):
                cell1 = cells[route_indices[j]]
                cell2 = cells[route_indices[j + 1]]
                route_distance += hypot(cell2.x - cell1.x, cell2.y - cell1.y)

            # Back to depot
            last_cell = cells[route_indices[-1]]
            route_distance += hypot(depot[0] - last_cell.x, depot[1] - last_cell.y)

        route_time = route_distance / cruise_speed

        route = Route(
            id=f"alns_route_{i}",
            cell_sequence=cell_sequence,
            loop_time=route_time,
            departure_time=0.0,  # Will be set by scheduler
        )
        routes.append(route)

    # Ensure we have exactly n_launch routes
    if len(routes) < len(solution):
        print(
            f"⚠️ ALNS produced {len(routes)} valid routes from {len(solution)} solutions"
        )

    return routes
