from uav_surveil.config import load_scenario
from uav_surveil.stage1_grid import build_grid_from_config
from uav_surveil.stage3_route import generate_routes_from_config
from uav_surveil.stage3_schedule import schedule_from_config


def test_schedule_spacing():
    cfg = load_scenario("baseline")
    cells = build_grid_from_config(cfg)
    n_launch = 4
    routes, _ = generate_routes_from_config(cfg, cells, n_launch)

    summary = schedule_from_config(cfg, routes)

    # Ensure β_adapt is in (0,1)
    assert 0 < summary.β_adapt < 1
    # Departure times strictly increasing
    departures = [r.departure_time for r in routes]
    assert departures == sorted(departures)
