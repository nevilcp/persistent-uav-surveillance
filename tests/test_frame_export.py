"""Frame export across baseline, obstacle and depot-closure scenarios."""

import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

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
    assert data['frames'][0]['c3_pending'] == data['frames'][0]['c3_overdue'] == 0


@pytest.mark.parametrize(('name', 'fleet_size', 'duration', 'title'), [
    ('R3_under_provisioned', 90, 4800.0, 'Under-Provisioned'),
    ('R3_over_provisioned', 150, 4800.0, 'Over-Provisioned'),
    ('R3_long_horizon', 120, 9600.0, 'Long Horizon'),
])
def test_r3_report_variant_export(name, fleet_size, duration, title):
    nominal = scenario('R3')
    params = scenario(name)
    assert params == replace(nominal, fleet=replace(nominal.fleet, size=fleet_size),
                             mission=replace(nominal.mission, duration=duration))

    data = dump(name, replace(params, mission=replace(params.mission, duration=1.0)))
    assert data['title'] == title
    assert len(data['frames'][0]['uavs']) == fleet_size
    assert len(data['depots']) == 3
    assert len(data['cells']) == 830


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
    dashboard = renderer.R3Dashboard(run)
    assert [track.unit for track in dashboard.tracks] == ['%', ' pp', ' Alarms', ' s', ' pp', ' UAVs', ' m']
    assert all(track.ticks[0] < track.ticks[-1] for track in dashboard.tracks)
    assert [len(paths) for paths in dashboard.paths] == [1, 1, 2, 2, 1, 1, 1]
    surface = renderer.skia.Surface(dashboard.width, dashboard.height)
    dashboard.draw(surface.getCanvas(), 0)
    frame = surface.makeImageSnapshot().toarray()
    assert frame[47:78, 1400:dashboard.chart_right, :3].min() == 255

    depot = run.depots[1]
    a, b, c, d = renderer.failure_crosses(depot, 4)
    assert renderer.failure_crosses(depot, 3) == [a, b, c]
    assert a[1] == b[1] and c[1] == d[1]
    assert a[0] == c[0] and b[0] == d[0]
    assert abs(a[0] - b[0]) == abs(a[1] - c[1]) > 0


def test_live_monitor_readouts_clear_when_conditions_resolve():
    pytest.importorskip('skia')
    pytest.importorskip('numpy')
    from render_animation import monitor_tracks

    frames = [
        {"t": t, "cov": 100, "battery_margin": battery,
         "current_violations": {"battery_margin": count},
         "c3_pending": pending, "c3_overdue": overdue}
        for t, battery, count, pending, overdue in ((1, -0.01, 3, 2, 0), (2, 0, 0, 1, 1),
                                                   (3, None, 0, 1, 1), (4, 0.1, 0, 0, 0))
    ]
    run = SimpleNamespace(fr=frames, cells=[(0, 0)], d={
        "rolling_window": 240,
        "monitor_limits": {"freshness": 180, "launch": 60, "recovery": 2},
        "recoveries": [{"t_fail_s": 1, "recovery_time_s": 3, "satisfied": False}],
    })
    _, battery, launch, recovery, *_ = monitor_tracks(run)

    assert dict(battery.fields)["Violations"] == ["3", "0", "—", "0"]
    assert dict(launch.fields)["Late"] == ["0", "1", "1", "0"]
    assert dict(launch.fields)["Waiting"] == ["2", "1", "1", "0"]
    assert dict(recovery.fields) == {
        "Active": ["1", "1", "1", "0"],
        "Elapsed": ["0 s", "1 s", "2 s", "—"],
        "Deadline": ["2 s"] * 4,
        "Overdue": ["0", "0", "1", "0"],
    }
    for frame in frames:
        del frame["c3_pending"], frame["c3_overdue"]
    assert dict(monitor_tracks(run)[2].fields)["Waiting"] == ["—"] * 4


def test_dashboard_draws_measured_traces_only_through_the_playhead():
    pytest.importorskip('skia')
    pytest.importorskip('numpy')
    import render_animation as renderer

    dashboard = renderer.R3Dashboard.__new__(renderer.R3Dashboard)
    measured, comparison = renderer.skia.Path(), renderer.skia.Path()
    for path, y in ((measured, 16), (comparison, 32)):
        path.moveTo(0, y)
        path.lineTo(100, y)
    dashboard.paths = [[], [], [measured, comparison], [measured, comparison]]
    dashboard.violations = {}
    surface = renderer.skia.Surface(100, 48)
    canvas = surface.getCanvas()
    area = renderer.skia.Rect.MakeWH(100, 48)

    for row in (2, 3):
        canvas.clear(renderer.color(renderer.CARD))
        dashboard.draw_paths(canvas, row, area, cursor=40)
        pixels = surface.makeImageSnapshot().toarray()
        assert pixels[14:18, :35, :3].min() < 255
        assert pixels[14:18, 45:, :3].min() == 255
        assert pixels[30:34, :35, :3].min() < 255
        comparison_ahead = pixels[30:34, 45:, :3]
        if row == 2:
            assert comparison_ahead.min() == 255
        else:
            assert comparison_ahead.min() < 255


def test_current_violation_counts_include_each_affected_uav_and_depot(monkeypatch):
    import uav_surveil.monitor as monitor_module

    monkeypatch.setattr(monitor_module, 'nearest_open_base', lambda _air, _depots, position, _mode: (0, position[0]))
    params = SimpleNamespace(uav=SimpleNamespace(range=100, sigma_min=0.2),
                             region=SimpleNamespace(zone_margin=10))
    air = SimpleNamespace(signed_distance=lambda position: position[1])
    depots = [SimpleNamespace(waiting=3, slots=1), SimpleNamespace(waiting=4, slots=2),
              SimpleNamespace(waiting=1, slots=1)]
    monitor = monitor_module.Monitor(params, [], airspace=None, depots=depots)
    monitor._airspace = air
    uavs = [SimpleNamespace(airborne=True, battery=battery, position=position)
            for battery, position in [(0.1, (0, 5)), (0.1, (0, 6)), (0.5, (50, 20))]]
    monitor.extended_sample(uavs)
    assert monitor.current_violations == {
        'battery_margin': 2, 'way_home_margin': 3,
        'queue_margin': 2, 'zone_clearance': 2,
    }


def test_dashboard_metrics_keep_positions_when_values_change(monkeypatch):
    pytest.importorskip('skia')
    import render_animation as renderer

    dashboard = renderer.R3Dashboard.__new__(renderer.R3Dashboard)
    dashboard.tracks = [
        SimpleNamespace(fields=[('Fresh', ['9.0%', '100.0%']),
                                ('Overdue', ['99', '1']),
                                ('Age Limit', ['180 s', '180 s'])]),
        SimpleNamespace(fields=[('Margin', ['−0.1 pp', '+10.0 pp']),
                                ('Violations', ['12', '0'])]),
    ]
    dashboard.metric_widths = [[130, 110, 140], [155, 105]]
    dashboard.monitor_keys = {}
    dashboard.violation = '#ff0000'
    surface = renderer.skia.Surface(800, 100)
    original = renderer.text
    calls = []

    def capture(cv, value, x, y, style, *args, **kwargs):
        width = original(cv, value, x, y, style, *args, **kwargs)
        calls.append((x, width))
        return width

    monkeypatch.setattr(renderer, 'text', capture)
    for row, widths in enumerate(dashboard.metric_widths):
        calls.clear()
        dashboard.draw_monitor_metrics(surface.getCanvas(), row, 0, 700, 50)
        assert calls[0][0] == 700 - sum(widths) + dashboard.metric_padding
        for label, dot, value in zip(calls[::3], calls[1::3], calls[2::3]):
            assert dot[0] - sum(label) == value[0] - sum(dot) == dashboard.metric_gap
        first = [x for x, _ in calls]
        calls.clear()
        dashboard.draw_monitor_metrics(surface.getCanvas(), row, 1, 700, 50)
        assert [x for x, _ in calls] == first
