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
    assert data['monitor_limits'] == {'freshness': 180, 'launch': 60, 'recovery': 300}
    assert data['frames'][0]['queue_margin'] is None
    assert data['frames'][0]['zone_clearance'] is None
    assert data['frames'][0]['routes_due'] == 1
    assert data['frames'][0]['routes_unstaffed'] == 0
    assert data['frames'][0]['failed'] == 0


def test_port_export_maps_freshness_to_free_cells_and_records_closure():
    params = scenario('R6')
    params = replace(params, mission=replace(params.mission, duration=2.0),
                     failure=replace(params.failure, closure_time=1.0))
    data = dump('R6', params)
    assert len(data['cells']) == len(data['frames'][0]['fresh']) == 830
    assert len(data['depots']) == 3 and len(data['zones']) == 3
    assert data['planner'] == 'choset_round_robin'
    assert len(data['routes']) == 87
    assert data['loop_polyline'] == []
    assert data['frames'][0]['closed_depots'] == []
    assert data['frames'][1]['closed_depots'] == [2]
    assert data['events'] == [{'t': 1.0, 'kind': 'closure', 'uav': '-', 'detail': 'base 2'}]
    x0, x1, y0, y1 = data['extent']
    assert all(x0 < x < x1 and y0 < y < y1 for x, y in data['depots'])


def test_choset_export_records_each_failure_and_failed_airframe():
    params = scenario('R5')
    params = replace(params, mission=replace(params.mission, duration=510.0),
                     failure=replace(params.failure, time=500.0, later_times=(502.0, 504.0, 506.0)))
    data = dump('R5', params)
    assert [e['t'] for e in data['events']] == [500.0, 502.0, 504.0, 506.0]
    assert all(e['kind'] == 'failure' for e in data['events'])
    assert data['frames'][499]['failed_uavs'] == []
    assert len(data['frames'][-1]['failed_uavs']) == 4
    assert data['frames'][-1]['failed'] == 4
    assert set(data['frames'][-1]['failed_uavs']) == {f"{e['uav']:02d}" for e in data['events']}
    assert data['failure_depots'] == {uid: 1 for uid in data['frames'][-1]['failed_uavs']}
    assert [entry['t_fail_s'] for entry in data['recoveries']] == [500, 502, 504, 506]


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
    assert run.study_id == 'R4'
    assert all(renderer.XLIM[0] < x < renderer.XLIM[1] and
               renderer.YLIM[0] < y < renderer.YLIM[1] for x, y in run.depots + run.cells)
    h = renderer.canvas_h(run)
    surface = renderer.skia.Surface(renderer.W, h)
    renderer.draw_frame(surface.getCanvas(), run, 0, h)
    assert surface.makeImageSnapshot().width() == renderer.W

    video_height = renderer.video_height(run)
    panel = renderer.MonitorPanel(run, video_height)
    assert panel.time_ticks() == [1.0]
    assert [track.unit for track in panel.tracks] == ['%', ' pp', ' Alarms', ' s', ' pp', ' UAVs', ' m']
    assert all(track.ticks[0] < track.ticks[-1] for track in panel.tracks)
    assert all(field[0].endswith(':') for track in panel.tracks for field in track.field_layout)
    surface = renderer.skia.Surface(renderer.VIDEO_W, video_height)
    renderer.draw_video_frame(surface.getCanvas(), run, 0, panel)
    assert surface.makeImageSnapshot().width() == renderer.VIDEO_W

    depot = run.depots[1]
    a, b, c, d = renderer.failure_crosses(depot, 4)
    assert renderer.failure_crosses(depot, 3) == [a, b, c]
    assert a[1] == b[1] and c[1] == d[1]
    assert a[0] == c[0] and b[0] == d[0]
    assert abs(a[0] - b[0]) == abs(a[1] - c[1]) > 0
