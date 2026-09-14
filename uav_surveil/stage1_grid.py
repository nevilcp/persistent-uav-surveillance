"""Stage 1: Grid Builder.

This module implements the discretization of the surveillance area
into a grid of cells for coverage optimization.
"""


from .config.parameters import SystemParameters
from .core.cell import Cell

# -----------------------------------------------------------------------------
# Public helper – build grid directly from a SystemParameters
# -----------------------------------------------------------------------------


def build_grid_from_config(config: "SystemParameters") -> list[Cell]:
    """Convenience wrapper to build a surveillance grid from a *SystemParameters* object.

    Args:
        config: Loaded system configuration

    Returns:
        List of ``Cell`` objects covering the configured area.
    """

    # Extract parameters via the helper already provided by the config class
    params = config.get_grid_build_params()
    return build_surveillance_grid(
        area_width=params["area_width"],
        area_length=params["area_length"],
        cell_size=params["cell_size"],
        origin=params["origin"],
    )


# Re-export for wildcard import
__all__ = [
    "build_grid_from_config",
    "build_surveillance_grid",
    "validate_grid_coverage",
]


def build_surveillance_grid(
    area_width: float,
    area_length: float,
    cell_size: float = 40.0,
    origin: tuple[float, float] = (0.0, 0.0),
) -> list[Cell]:
    """Build a regular grid of surveillance cells over the area.

    Discretizes the rectangular surveillance area into square cells
    of the specified size, creating Cell objects for each grid position.

    Args:
        area_width: Width of surveillance area (m)
        area_length: Length of surveillance area (m)
        cell_size: Side length of each square cell (m)
        origin: Bottom-left corner coordinates (x, y)

    Returns:
        List of Cell objects covering the surveillance area

    Note:
        This is a placeholder implementation. Full grid builder will include:
        - Irregular terrain handling
        - No-fly zone exclusions
        - Cell priority weighting
        - Visibility analysis
    """
    # TODO: Implement full grid discretization algorithm
    # TODO: Add terrain constraints and exclusion zones
    # TODO: Implement cell priority assignment
    # TODO: Add visibility and coverage validation

    cells = []

    # Calculate grid dimensions
    cols = int(area_width / cell_size)
    rows = int(area_length / cell_size)

    # Generate cells with center coordinates
    for row in range(rows):
        for col in range(cols):
            cell_x = origin[0] + (col + 0.5) * cell_size
            cell_y = origin[1] + (row + 0.5) * cell_size
            cell_id = f"cell_{row:03d}_{col:03d}"

            cell = Cell(
                id=cell_id, x=cell_x, y=cell_y, priority=1.0  # Default uniform priority
            )
            cells.append(cell)

    return cells


def validate_grid_coverage(
    cells: list[Cell], area_width: float, area_length: float
) -> bool:
    """Validate that grid cells provide complete area coverage.

    Args:
        cells: List of surveillance cells
        area_width: Expected area width (m)
        area_length: Expected area length (m)

    Returns:
        True if coverage is complete and valid

    Note:
        Placeholder for full coverage validation including overlap analysis.
    """
    # TODO: Implement complete coverage validation
    # TODO: Check for gaps and overlaps
    # TODO: Validate cell density requirements

    return len(cells) > 0
