"""Frame export across baseline, obstacle and depot-closure scenarios."""

import sys
from dataclasses import replace
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from export_frames import dump, scenario


def test_nominal_export_keeps_the_original_schema():
    params = scenario('nominal')
    data = dump('nominal', replace(params, mission=replace(params.mission, duration=2.0)))
    assert 'extent' not in data and 'depots' not in data
    assert len(data['cells']) == len(data['frames'][0]['fresh']) == 144
    assert data['meta'] == 'Baseline  |  Algorithm: Round-Robin  |  Fleet: 21 UAVs'


def test_port_export_maps_freshness_to_free_cells_and_records_closure():
    params = scenario('R6')
    params = replace(params, mission=replace(params.mission, duration=2.0),
                     failure=replace(params.failure, closure_time=1.0))
    data = dump('R6', params)
    assert len(data['cells']) == len(data['frames'][0]['fresh']) == 830
    assert len(data['depots']) == 3 and len(data['zones']) == 3
    assert data['loop_polyline']
    assert data['frames'][0]['closed_depots'] == []
    assert data['frames'][1]['closed_depots'] == [2]
    x0, x1, y0, y1 = data['extent']
    assert all(x0 < x < x1 and y0 < y < y1 for x, y in data['depots'])


def test_renderer_handles_extended_geometry_and_single_frame(tmp_path):
    pytest.importorskip('skia')
    pytest.importorskip('numpy')
    import json
    import render_animation as renderer
    params = scenario('R4')
    data = dump('R4', replace(params, mission=replace(params.mission, duration=1.0)))
    path = tmp_path / 'frames.json'
    path.write_text(json.dumps(data))
    run = renderer.Run(path)
    assert len(run.rects) == 830
    assert renderer.XLIM == tuple(data['extent'][:2])
    h = renderer.canvas_h(run)
    surface = renderer.skia.Surface(renderer.W, h)
    renderer.draw_frame(surface.getCanvas(), run, 0, h)
    assert surface.makeImageSnapshot().width() == renderer.W
