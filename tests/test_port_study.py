"""The port config, the study's run definitions and its random maps."""

import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

from run_port_study import (
    PORT,
    random_zones,
    runs,
    single_base,
)

from uav_surveil import Simulation
from uav_surveil.airspace import Airspace
from uav_surveil.fleet import split_fleet
from uav_surveil.grid import Grid


def test_port_config_is_the_worked_out_map():
    region = PORT.region
    air = Airspace.from_region(Grid(region), region)
    assert len(region.bases) == 3 and len(region.zones) == 3
    assert len(air.free_cells) == 830  # 900 cells less the 70 under the zones
    assert PORT.patrol.mode == "round_robin"
    assert PORT.uav.depot_slots == 2
    assert PORT.mission.revisit_threshold == 180.0
    assert PORT.mission.duration == 4800.0


@pytest.mark.parametrize("position", PORT.region.bases)
def test_port_depots_are_500m_from_the_nearest_region_boundary(position):
    x, y = position
    region = PORT.region
    assert x < 0 or x > region.width or y < 0 or y > region.length
    nearest = (min(max(x, 0.0), region.width), min(max(y, 0.0), region.length))
    assert math.dist(position, nearest) == pytest.approx(500.0)


def test_the_runs_differ_from_the_main_run_only_where_the_doc_says():
    r = runs()
    assert r["R3"] == PORT
    assert r["R4"].failure.enabled and r["R4"].failure.near_depot == 1
    assert r["R0"].dispatch.metric == "straight"
    assert r["R2"].region.zones == ()
    assert r["R5"].failure.near_depot == 1
    assert (r["R5"].failure.time, *r["R5"].failure.later_times) == (
        1800.0,
        2100.0,
        2400.0,
        2700.0,
    )
    assert r["R6"].failure.closed_depot == 2 and r["R6"].failure.closure_time == 1800.0
    assert r["R3"].patrol.mode == "round_robin"
    split = split_fleet(r["R3"].fleet)
    assert (split.n_fly, split.n_rotate, split.n_spare) == (87, 27, 6)


@pytest.mark.parametrize("index", [0, 1, 2])
def test_a_single_base_gets_every_service_slot_and_keeps_the_fleet(index):
    params = single_base(index)
    assert params.region.bases == (PORT.region.bases[index],)
    assert params.uav.depot_slots == 6
    assert Simulation(params).split.total == 120


def test_random_maps_are_reproducible_and_differ_by_seed():
    assert random_zones(3) == random_zones(3)
    assert random_zones(3) != random_zones(4)


@pytest.mark.parametrize("seed", range(5))
def test_random_maps_cover_about_eight_percent_and_stay_flyable(seed):
    zones = random_zones(seed)
    region = PORT.region
    size = region.cell_size
    cells = set()
    for x0, y0, x1, y1 in zones:
        assert all(v % size == 0 for v in (x0, y0, x1, y1))
        assert 0 <= x0 < x1 <= region.width and 0 <= y0 < y1 <= region.length
        new = {
            (r, c)
            for r in range(int(y0 // size), int(y1 // size))
            for c in range(int(x0 // size), int(x1 // size))
        }
        assert not new & cells  # zones do not overlap
        cells |= new
    assert 72 <= len(cells) <= 83
    params = replace(PORT, region=replace(region, zones=zones))
    air = Airspace.from_region(
        Grid(params.region), params.region
    )  # connected, bases clear
    assert len(air.free_cells) == 900 - len(cells)
