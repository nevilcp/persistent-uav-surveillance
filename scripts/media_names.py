"""Shared names for surveillance GIF and MP4 pairs."""

import re

PORT_SCENES = {
    "R0": ("straight_return_control", "Straight-line dispatch", "Straight return metric"),
    "R1_west": ("single_depot_west", "West Depot", "Choset + Round-Robin"),
    "R1_east": ("single_depot_east", "East Depot", "Choset + Round-Robin"),
    "R1_north": ("single_depot_north", "North Depot", "Choset + Round-Robin"),
    "R2": ("multiple_depots", "No NFZs", "No obstacles"),
    "R3": ("no_fly_zones_choset_round_robin", "Nominal", "Choset + Round-Robin"),
    "R4": ("no_fly_zones_single_failure", "Single failure", "Choset + Round-Robin"),
    "R5": ("no_fly_zones_repeated_failures", "Repeated failures", "Choset + Round-Robin"),
    "R6": ("no_fly_zones_depot_closure", "Depot closure", "Choset + Round-Robin"),
}


def media_stem(title, prefix=""):
    return re.sub(r"[^a-z0-9]+", "-", f"{prefix} {title}".lower()).strip("-") or "scenario"
