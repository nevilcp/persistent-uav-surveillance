import json
from dataclasses import replace

import pytest

from uav_surveil.params import FailureParams, Parameters, PatrolParams, RegionParams

TANK = (200.0, 520.0, 520.0, 680.0)


def test_bases_default_to_the_single_depot():
    assert Parameters().region.bases == ((-500.0, 0.0),)


def test_bases_prefer_depots_when_given():
    region = RegionParams(depots=((-40.0, 600.0), (1240.0, 600.0)))
    assert region.bases == region.depots


def test_json_lists_become_nested_tuples(tmp_path):
    path = tmp_path / "port.json"
    path.write_text(
        json.dumps(
            {
                "region": {"depots": [[-40, 600], [1240, 600]], "zones": [list(TANK)]},
                "failure": {"near_depot": 1, "later_times": [1800, 2100]},
                "patrol": {"mode": "round_robin"},
            }
        )
    )
    params = Parameters.from_json(path)
    assert params.region.depots == ((-40, 600), (1240, 600))
    assert params.region.zones == (TANK,)
    assert params.failure.later_times == (1800, 2100)
    assert params.failure.near_depot == 1
    assert params.patrol.mode == "round_robin"


def test_new_defaults_keep_the_paper_behaviour():
    params = Parameters()
    assert params.region.zones == ()
    assert params.uav.depot_slots == 0
    assert params.patrol.mode == "round_robin"
    assert params.dispatch.metric == "flyable"
    assert params.failure.closed_depot is None


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        ({"region": RegionParams(zones=((5.0, 0.0, 1.0, 2.0),))}, "zone"),
        ({"region": RegionParams(zone_margin=-1.0)}, "zone_margin"),
        ({"failure": FailureParams(near_depot=1)}, "near_depot"),
        ({"failure": FailureParams(closed_depot=0)}, "at least two bases"),
        ({"patrol": PatrolParams(mode="loop")}, "patrol mode"),
        ({"patrol": PatrolParams(mode="shared_loop")}, "shared_loop was removed"),
    ],
)
def test_airspace_and_patrol_validation(kwargs, match):
    with pytest.raises(ValueError, match=match):
        Parameters(**kwargs)


def test_unknown_metric_is_rejected():
    params = Parameters()
    with pytest.raises(ValueError, match="metric"):
        Parameters(dispatch=replace(params.dispatch, metric="manhattan"))
