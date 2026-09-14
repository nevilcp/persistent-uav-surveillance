from uav_surveil.config import load_scenario
from uav_surveil.stage1_grid import build_grid_from_config, validate_grid_coverage


def test_baseline_grid_dimensions():
    """Baseline scenario should produce expected number of cells and pass validation."""
    cfg = load_scenario("baseline")
    cells = build_grid_from_config(cfg)

    expected_cols = int(cfg.mission.area_width / cfg.grid.cell_size)
    expected_rows = int(cfg.mission.area_length / cfg.grid.cell_size)
    assert len(cells) == expected_cols * expected_rows

    # Simple coverage check should return True
    assert validate_grid_coverage(
        cells, cfg.mission.area_width, cfg.mission.area_length
    )
