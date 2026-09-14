"""Factory for Stage 3A route generators.

Usage:
    routes, summary = create_route_generator(name)(cells, n_launch, speed)
"""

from collections.abc import Callable, Sequence

from .core.cell import Cell
from .core.route import Route
from .stage3_route import generate_routes_greedy
from .stage3_route_alns import generate_routes_alns
from .stage3_route_balanced import generate_routes_balanced
from .stage3_route_kmnn import generate_routes_kmnn
from .stage3_route_roundrobin import generate_routes_roundrobin

# Type definition for route generator functions
Generator = Callable[[Sequence[Cell], int, float], tuple[list[Route], object]]

# Map algorithm names to generator functions
_MAP = {
    "greedy": generate_routes_greedy,
    "kmnn": generate_routes_kmnn,
    "alns": generate_routes_alns,
    "balanced": generate_routes_balanced,
    "roundrobin": generate_routes_roundrobin,
}


def get_route_generator(name: str) -> Generator:
    if name not in _MAP:
        raise ValueError(f"Unknown route generator '{name}'. Available: {list(_MAP)}")
    return _MAP[name]


__all__ = ["get_route_generator"]
