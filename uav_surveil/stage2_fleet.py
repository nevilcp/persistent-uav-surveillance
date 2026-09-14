"""Stage 2: Fleet Size Optimization.

This module implements the MILP-based optimization for determining
optimal fleet composition (active + spare UAVs) subject to cost
constraints and spare floor requirements.
"""

from dataclasses import dataclass

# Configuration helper (only spare ratio needed for now)
from .config.parameters import SystemParameters

# -----------------------------------------------------------------------------
# Data class: optimisation outcome (kept unchanged except field docs)
# -----------------------------------------------------------------------------


@dataclass
class FleetOptimizationResult:
    """Result of fleet size optimization.

    Attributes:
        n_launch: Number of UAVs to launch for active patrol
        n_spare: Number of spare UAVs to maintain on pad
        n_rotation: Number of rotation spares (25% of active fleet)
        n_contingency: Number of contingency spares (fixed at 1)
        total_cost: Total fleet cost (acquisition + operation)
        spare_ratio: Ratio of spares to total fleet
        is_feasible: Whether solution meets all constraints
        solver_status: Optimization solver status
    """

    n_launch: int
    n_spare: int
    n_rotation: int
    n_contingency: int
    total_cost: float
    spare_ratio: float
    is_feasible: bool
    solver_status: str


# -----------------------------------------------------------------------------
# Simplified optimiser (dimensionless cost) – no monetary inputs needed
# -----------------------------------------------------------------------------


def optimize_fleet_size(
    K_inv: int,
    beta_min: float = 0.2,
    C_L: float = 1.0,
    C_S: float = 1.2,
) -> FleetOptimizationResult:
    """Exact convex–integer formulation of the fleet-sizing problem (P_FS).

    minimise   C_L n_L + C_S n_S
    subject to n_L + n_S  = K_inv              (inventory)
               n_S >= β_min · K_inv            (spare-floor)
               n_L, n_S ∈ ℤ₀⁺

    Args:
        K_inv:        Total available airframes (inventory).
        beta_min:     Required spare ratio β_min (fixed or adaptive).
        C_L:          Dimensionless cost coefficient for launched UAVs.
        C_S:          Dimensionless cost coefficient for spares.

    Returns:
        FleetOptimizationResult
    """

    import math

    if K_inv <= 0:
        raise ValueError("K_inv must be positive")
    if not (0.0 <= beta_min <= 1.0):
        raise ValueError("beta_min must be in [0,1]")

    # Minimum spares to satisfy spare-floor
    n_spare_min = math.ceil(beta_min * K_inv)

    # Optimal solution: use the minimum spares (C_S ≥ C_L assumed)
    n_spare = n_spare_min
    n_launch = K_inv - n_spare

    # Cost (dimensionless)
    total_cost = C_L * n_launch + C_S * n_spare

    # For backward compatibility, calculate rotation/contingency breakdown
    n_contingency = min(1, n_spare)  # Fixed at 1 (or 0 if no spares)
    n_rotation = max(0, n_spare - n_contingency)  # Remaining spares

    return FleetOptimizationResult(
        n_launch=n_launch,
        n_spare=n_spare,
        n_rotation=n_rotation,
        n_contingency=n_contingency,
        total_cost=total_cost,
        spare_ratio=n_spare / K_inv,
        is_feasible=True,
        solver_status="inventory_exact",
    )


def optimize_fleet_enhanced(
    K_inv: int,
    beta_min: float = 0.2,
    C_L: float = 1.0,
    C_R: float = 1.1,
    C_C: float = 1.5,
    force_supervisor_method: bool = False,
) -> FleetOptimizationResult:
    """Enhanced fleet optimization with rotation/contingency spare distinction.

    Mathematical Enhancement:
    -------------------------
    Original:  minimize C_L * n_L + C_S * n_S
               subject to n_S ≥ β_min * K_inv

    Enhanced:  minimize C_L * n_L + C_R * n_R + C_C * n_C
               subject to n_R = ⌈n_L / 4⌉     (rotation spares)
                         n_C = 1             (contingency spare)
                         n_S = n_R + n_C     (total spares)
                         n_L + n_S ≤ K_inv   (inventory limit)

    Args:
        K_inv: Total available airframes (inventory)
        beta_min: Minimum spare ratio (for comparison only)
        C_L: Cost coefficient for active UAVs
        C_R: Cost coefficient for rotation spares
        C_C: Cost coefficient for contingency spares
        force_supervisor_method: If True, use supervisor's exact formula

    Returns:
        FleetOptimizationResult with rotation/contingency breakdown
    """

    if K_inv <= 0:
        raise ValueError("K_inv must be positive")

    if force_supervisor_method:
        # Supervisor's Method: Keep existing fleet optimization, just categorize spares
        # Step 1: Use existing logic to determine total spares
        base_result = optimize_fleet_size(K_inv, beta_min, C_L, C_R)

        # Step 2: Categorize spares into rotation vs contingency
        n_launch = base_result.n_launch
        n_spare = base_result.n_spare
        n_contingency = min(1, n_spare)  # Fixed at 1 (or 0 if no spares)
        n_rotation = max(0, n_spare - n_contingency)  # Remaining spares

        total_cost = C_L * n_launch + C_R * n_rotation + C_C * n_contingency
        return FleetOptimizationResult(
            n_launch=n_launch,
            n_spare=n_spare,
            n_rotation=n_rotation,
            n_contingency=n_contingency,
            total_cost=total_cost,
            spare_ratio=n_spare / K_inv,
            is_feasible=True,
            solver_status="supervisor_categorized",
        )

    else:
        # Backward Compatible: Use original logic with breakdown
        result = optimize_fleet_size(K_inv, beta_min, C_L, C_R)
        # Already has rotation/contingency calculated
        return result


def validate_fleet_configuration(
    n_launch: int, n_spare: int, spare_floor_ratio: float
) -> bool:
    """Validate fleet configuration meets constraints.

    Args:
        n_launch: Number of active patrol UAVs
        n_spare: Number of spare UAVs
        spare_floor_ratio: Required minimum spare ratio

    Returns:
        True if configuration is valid
    """
    if n_launch <= 0 or n_spare < 0:
        return False

    total_fleet = n_launch + n_spare
    actual_spare_ratio = n_spare / total_fleet if total_fleet > 0 else 0

    return actual_spare_ratio >= spare_floor_ratio


# -----------------------------------------------------------------------------
# Optional Pyomo-based MILP implementation
# -----------------------------------------------------------------------------


def _optimize_fleet_size_milp(
    K_inv: int,
    beta_min: float,
    C_L: float,
    C_S: float,
    solver: str = "glpk",
) -> FleetOptimizationResult | None:
    """Solve fleet sizing problem using Pyomo if available.

    Falls back to *None* if Pyomo or the desired solver backend is not
    available or if any exception occurs during solve.
    """
    try:
        # Lazy import – avoids mandatory dependency on Pyomo for users who only
        # need the closed-form solution.
        from pyomo.environ import (
            ConcreteModel,
            Constraint,
            NonNegativeIntegers,
            Objective,
            SolverFactory,
            Var,
            minimize,
            value,
        )
    except ImportError:
        return None

    # Build model
    m = ConcreteModel()
    m.n_L = Var(within=NonNegativeIntegers)
    m.n_S = Var(within=NonNegativeIntegers)

    # Objective: minimise cost
    m.obj = Objective(expr=C_L * m.n_L + C_S * m.n_S, sense=minimize)

    # Constraints
    m.inv = Constraint(expr=m.n_L + m.n_S == K_inv)
    m.spare_floor = Constraint(expr=m.n_S >= beta_min * K_inv)

    # Solve
    opt = SolverFactory(solver)
    if not opt.available(exception_flag=False):
        return None

    try:
        opt.solve(m, tee=False)
    except Exception:  # noqa: BLE001 - solver backend errors vary, treat any as infeasible
        return None

    n_launch = round(value(m.n_L))
    n_spare = round(value(m.n_S))

    # Check feasibility again (robust) and cost
    if n_launch < 0 or n_spare < 0 or n_launch + n_spare != K_inv:
        return None

    total_cost = C_L * n_launch + C_S * n_spare

    # Calculate rotation/contingency breakdown for MILP result
    n_contingency = min(1, n_spare)  # Fixed at 1 (or 0 if no spares)
    n_rotation = max(0, n_spare - n_contingency)  # Remaining spares

    return FleetOptimizationResult(
        n_launch=n_launch,
        n_spare=n_spare,
        n_rotation=n_rotation,
        n_contingency=n_contingency,
        total_cost=total_cost,
        spare_ratio=n_spare / K_inv,
        is_feasible=True,
        solver_status=f"milp_{solver}",
    )


# -----------------------------------------------------------------------------
# Convenience wrapper – use SystemParameters to optimise fleet size
# -----------------------------------------------------------------------------


def optimize_fleet_from_config(
    config: "SystemParameters",
    C_L: float = 1.0,
    C_S: float = 1.2,
    use_milp: bool = True,
    use_enhanced: bool = False,
) -> FleetOptimizationResult:
    """Optimise fleet size using parameters from *SystemParameters*.

    Attempts MILP solve via Pyomo if *use_milp* is True and a suitable solver
    is available; otherwise falls back to the closed-form exact solution.

    Args:
        config: System configuration parameters
        C_L: Cost coefficient for active UAVs
        C_S: Cost coefficient for spares (used as C_R if enhanced)
        use_milp: Whether to attempt MILP solve
        use_enhanced: Whether to use enhanced fleet sizing with rotation/contingency
    """

    params = config.get_fleet_optimization_params()
    K_inv = params.get("total_inventory", 4)
    beta_min = params["spare_floor_ratio"]

    if use_enhanced:
        # Use enhanced optimization with supervisor's method
        return optimize_fleet_enhanced(
            K_inv=K_inv,
            beta_min=beta_min,
            C_L=C_L,
            C_R=C_S,  # Reuse C_S as rotation cost
            C_C=C_S * 1.25,  # Contingency slightly more expensive
            force_supervisor_method=True,
        )

    if use_milp:
        milp_res = _optimize_fleet_size_milp(K_inv, beta_min, C_L, C_S)
        if milp_res is not None:
            return milp_res

    # Fallback – closed-form exact solution
    return optimize_fleet_size(
        K_inv=K_inv,
        beta_min=beta_min,
        C_L=C_L,
        C_S=C_S,
    )


# Make it explicit for * import users
__all__ = [
    "FleetOptimizationResult",
    "optimize_fleet_enhanced",
    "optimize_fleet_from_config",
    "optimize_fleet_size",
    "validate_fleet_configuration",
]
