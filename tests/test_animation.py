"""Smoke tests of scripts/make_animation.py."""

import importlib.util
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("matplotlib")
pytest.importorskip("PIL")

from uav_surveil import Parameters, Simulation

SCRIPTS = Path(__file__).parent.parent / "scripts"
SCRIPT = SCRIPTS / "make_animation.py"


@pytest.fixture(scope="module")
def animation():
    sys.path.insert(0, str(SCRIPTS))  # make_animation imports run_scenarios
    spec = importlib.util.spec_from_file_location("make_animation", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclass annotations resolve through sys.modules
    spec.loader.exec_module(module)
    return module


def test_frame_schedule_eases_into_half_speed_at_the_failure(animation):
    schedule = animation.frame_times(0.0, 2700.0, 1800.0, 150.0, 25)
    normal = 150.0 / 25
    assert schedule[0][0] == 0.0 and schedule[-1][0] == 2700.0
    steps = {time: step for time, step, _ in schedule}
    assert steps[600.0 if 600.0 in steps else min(steps, key=lambda t: abs(t - 600))] == normal
    slow = [step for time, step, flag in schedule if flag]
    assert slow and min(slow) == pytest.approx(normal / 2)
    mid = [step for _, step, flag in schedule if not flag and step < normal]
    assert mid  # the speed ramps, it does not jump


def test_schedule_without_a_failure_has_one_speed(animation):
    schedule = animation.frame_times(0.0, 600.0, None, 100.0, 25)
    assert {step for _, step, _ in schedule} == {4.0}
    assert not any(slow for *_, slow in schedule)


def test_cell_colours_keep_overdue_loudest_and_unseen_hollow(animation):
    theme = animation.LIGHT
    ages = np.array([0.0, 90.0, 180.0, 400.0, np.inf])
    for banded in (True, False):
        faces, edges = animation.cell_colors(ages, 180.0, theme, banded)
        assert faces[4, 3] == 0 and edges[4, 3] > 0  # unseen: no fill, an outline
        assert (faces[:4, 3] == 1).all()
        assert faces[3, 0] > faces[2, 0]  # past the limit the red channel jumps


def test_chapters_follow_the_failure(animation):
    base = Parameters()
    params = replace(
        base, failure=replace(base.failure, enabled=True), mission=replace(base.mission, duration=2400.0)
    )
    sim = Simulation(params)
    sim.run()
    chapters = animation.chapter_plan(sim, 2400.0, animation.LIGHT)
    assert [c.name for c in chapters] == ["Deploy", "Patrol", "Failure", "Bridging", "Takeover", "Recovered"]
    assert chapters[0].start == 0.0 and chapters[-1].end == 2400.0
    assert all(a.end == b.start for a, b in zip(chapters, chapters[1:]))


def test_gif_is_written_with_one_frame_per_snapshot(animation, tmp_path):
    base = Parameters()
    params = replace(
        base, failure=replace(base.failure, enabled=True), mission=replace(base.mission, duration=120.0)
    )
    sim = Simulation(params)
    trails = animation.Trails()
    snapshots = []
    for target in (30.0, 60.0, 90.0):
        while sim.time < target:
            sim.step()
        snapshots.append(animation.capture(sim, 30.0, trails))
    th = animation.LIGHT
    renderer = animation.Renderer(
        sim, 120.0, "test", 25, 0.5, th, animation.LAYOUTS["hero"], banded=True
    )
    frames = [renderer.draw(s) for s in snapshots]

    colours = 128
    palette = animation.build_palette(frames, th, colours)
    indexed = (animation.quantise(f, palette, colours - 1) for f in frames)
    out = tmp_path / "demo.gif"
    animation.encode_gif(indexed, palette, [50] * len(frames), out, colours)

    from PIL import Image

    with Image.open(out) as gif:
        assert gif.n_frames == len(frames) and gif.size == frames[0].size
