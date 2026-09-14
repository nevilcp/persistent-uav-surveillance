"""Unit tests for Stage 4 baseline policy (apply_policy)."""

from uav_surveil.config.parameters import SystemParameters
from uav_surveil.core.uav import UAV, UAVState
from uav_surveil.stage4_policy import apply_policy


def test_low_soc_triggers_return():
    """UAV with SoC below threshold should be flagged to fly home."""
    cfg = SystemParameters()  # default baseline config

    # SoC < threshold (default threshold 0.2)
    uav = UAV(
        id="UAV_TEST",
        x=0.0,
        y=0.0,
        soc=0.15,
        route_list=[],
        launch_time=0.0,
        is_active=True,
        state=UAVState.ON_MISSION,
    )
    uavs = [uav]

    apply_policy(uavs, current_time=0.0, config=cfg)

    assert (
        getattr(uav, "_fly_home", False) is True
    ), "UAV should be flagged for return when SoC low"


def test_soc_above_threshold_no_return():
    """UAV with sufficient SoC should not be flagged to fly home."""
    cfg = SystemParameters()

    # SoC > threshold
    uav = UAV(
        id="UAV_TEST2",
        x=0.0,
        y=0.0,
        soc=0.5,
        route_list=[],
        launch_time=0.0,
        is_active=True,
        state=UAVState.ON_MISSION,
    )
    uavs = [uav]

    apply_policy(uavs, current_time=0.0, config=cfg)

    assert (
        getattr(uav, "_fly_home", False) is False
    ), "UAV should not be flagged when SoC sufficient"
