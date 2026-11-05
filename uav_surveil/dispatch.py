"""Stage 4: battery-aware dispatch rules."""

from __future__ import annotations

from .params import DispatchParams, UAVParams


def return_cost(distance_to_depot: float, uav: UAVParams) -> float:
    """Battery fraction needed to fly ``distance_to_depot``."""
    return distance_to_depot / uav.range


def return_threshold(
    distance_to_depot: float, uav: UAVParams, dispatch: DispatchParams
) -> float:
    """Battery level at which a UAV must head home."""
    return uav.sigma_min + return_cost(distance_to_depot, uav) + dispatch.return_buffer


def predicted_arrival_battery(
    battery: float, distance_to_depot: float, uav: UAVParams
) -> float:
    """Battery level on reaching the depot by the direct route."""
    return battery - return_cost(distance_to_depot, uav)


def must_return(
    battery: float, distance_to_depot: float, uav: UAVParams, dispatch: DispatchParams
) -> bool:
    """Whether the UAV must return to base."""
    return battery <= return_threshold(distance_to_depot, uav, dispatch)


def needs_replacement(
    battery: float, distance_to_depot: float, uav: UAVParams, dispatch: DispatchParams
) -> bool:
    """Whether a replacement UAV should be launched."""
    b_hat = predicted_arrival_battery(battery, distance_to_depot, uav)
    return b_hat <= uav.sigma_min + dispatch.prelaunch_margin
