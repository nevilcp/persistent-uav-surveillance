"""Planar geometry shared by every stage."""

from __future__ import annotations

import math
from collections.abc import Sequence

from .params import Point


def distance(p: Point, q: Point) -> float:
    """Euclidean distance between two points."""
    return math.hypot(q[0] - p[0], q[1] - p[1])


def path_length(points: Sequence[Point]) -> float:
    """Length of the open polyline through ``points``."""
    return sum(distance(a, b) for a, b in zip(points, points[1:]))


def step_towards(p: Point, target: Point, max_step: float) -> tuple[Point, float, bool]:
    """New position, distance flown and arrival flag after moving at most ``max_step``."""
    # on arrival the rest of the step is discarded, costing under one step per waypoint
    remaining = distance(p, target)
    if remaining <= max_step:
        return target, remaining, True
    scale = max_step / remaining
    new_p = (p[0] + (target[0] - p[0]) * scale, p[1] + (target[1] - p[1]) * scale)
    return new_p, max_step, False
