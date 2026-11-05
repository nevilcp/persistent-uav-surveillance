"""Stage 3: staggered initial launches."""

from __future__ import annotations

from dataclasses import replace

from .params import FleetParams, ScheduleParams
from .routing import Route


def assign_departures(
    routes: list[Route], fleet: FleetParams, schedule: ScheduleParams
) -> list[Route]:
    """Routes with their departure times filled in."""
    return [
        replace(
            route,
            departure=(i // fleet.batch_size) * schedule.batch_period
            + (i % fleet.batch_size) * schedule.launch_gap,
        )
        for i, route in enumerate(routes)
    ]
