"""Stage 4: Boolean π-Policy (baseline).

A very lightweight rule engine that decides per-tick UAV actions.
For the *baseline* architecture it only enforces:

1. Battery return rule – if the State-of-Charge (SoC) of an ON_MISSION UAV
   falls below ``soc_return_threshold`` from the config, transition it to
   ``UAVState.RTB``.  The *simulation* loop will then route the UAV back to
   the depot and trigger a battery swap.

2. Spare-floor rule – ensure at least one SPARE is ready on the pad.  If the
   spare floor is violated we *could* immediately launch a standby UAV, but in
   the baseline version we just increment a violation counter (Stage-6 will
   log it).

The policy mutates the UAV objects *in-place* – it does not return a separate
list of actions, which keeps integration with the existing simulation simple.
This can be refactored later once the action/event bus is introduced.
"""

from __future__ import annotations

from .config.parameters import SystemParameters
from .core.uav import UAV, UAVState

__all__ = [
    "apply_policy",
    "get_distance_aware_threshold",
]


def get_distance_aware_threshold(
    uav_x: float,
    uav_y: float,
    depot_x: float,
    depot_y: float,
    base_threshold: float = 0.25,
) -> float:
    """Calculate distance-aware battery return threshold.

    Mathematical Enhancement:
    -------------------------
    Original: θ_return = 0.25 (fixed)
    Enhanced: θ_return = θ_base + Δθ(d_depot)

    Where:
        d_depot = √[(x_uav - x_depot)² + (y_uav - y_depot)²]
        Δθ(d) = { +0.05  if d > 400m (distant: return at 30%)
                { 0.00   if 200 ≤ d ≤ 400m (mid-range: return at 25%)
                { -0.05  if d < 200m (near: return at 20%)

    This risk-adjusted policy ensures distant UAVs have extra battery margin
    for the longer return journey, while nearby UAVs can operate longer.

    Args:
        uav_x, uav_y: Current UAV position
        depot_x, depot_y: Depot position
        base_threshold: Base SoC return threshold (typically 0.25)

    Returns:
        Adjusted SoC threshold for this UAV's current position
    """
    import math

    # Calculate Euclidean distance to depot
    distance = math.sqrt((uav_x - depot_x) ** 2 + (uav_y - depot_y) ** 2)

    # Apply distance-based adjustment
    if distance > 400.0:
        # Distant cells: return earlier (higher threshold)
        return base_threshold + 0.05
    elif distance < 200.0:
        # Near cells: return later (lower threshold)
        return base_threshold - 0.05
    else:
        # Mid-range: use base threshold
        return base_threshold


def apply_policy(
    uavs: list[UAV], current_time: float, config: SystemParameters
) -> None:
    """Apply enhanced rule-based Stage-4 policy with distance-aware battery threshold.

    Args:
        uavs:      Mutable list of *UAV* objects managed by the simulation.
        current_time: Simulation time (s)
        config:    Loaded *SystemParameters* containing thresholds.
    """

    # -------------------------------------------------------------------------
    # 1) Enhanced battery return rule – distance-aware SoC threshold
    # -------------------------------------------------------------------------
    base_threshold = config.battery.soc_return_threshold
    depot_x = getattr(config.mission, "depot_x", -500.0)
    depot_y = getattr(config.mission, "depot_y", 0.0)

    for uav in uavs:
        if uav.state == UAVState.ON_MISSION:
            # Calculate distance-aware threshold
            soc_threshold = get_distance_aware_threshold(
                uav.x, uav.y, depot_x, depot_y, base_threshold
            )
            # Log once the first time the UAV hits its distance-aware threshold
            import math

            distance_to_depot = math.hypot(uav.x - depot_x, uav.y - depot_y)
            if uav.soc <= soc_threshold:
                if not getattr(uav, "_threshold_logged", False):
                    print(
                        f"🔋 {uav.id} SoC {uav.soc:.2f} ≤ θ_return "
                        f"{soc_threshold:.2f} at d={distance_to_depot:.0f}m → RTB"
                    )
                    uav._threshold_logged = True
                # Transition to RTB; the simulation movement logic will act
                # on this state and navigate home.
                uav.state = UAVState.RTB

    # -------------------------------------------------------------------------
    # 2) Spare-floor rule – ensure at least one spare on pad (baseline)
    # -------------------------------------------------------------------------
    # Count spare UAVs ready on pad (launch_time == inf) – launch decisions
    # are still handled inside *gss.simulation* so we only detect for now.
    spare_ready = sum(
        1
        for uav in uavs
        if uav.state == UAVState.SPARE and uav.launch_time == float("inf")
    )
    if spare_ready == 0:
        # Future: trigger immediate launch or quick patch. For baseline we do
        # nothing; Stage-6 compliance checker will increment the C3 counter.
        pass
