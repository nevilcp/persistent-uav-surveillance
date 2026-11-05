"""Cell ages and the coverage metric."""

from __future__ import annotations

import math


class CoverageMap:
    """Last-observation time of every cell."""

    def __init__(self, n_cells: int) -> None:
        # never-observed cells have infinite age, so coverage starts at zero
        self._last_seen = [-math.inf] * n_cells

    def observe(self, cell_index: int, time: float) -> None:
        self._last_seen[cell_index] = time

    def age(self, cell_index: int, time: float) -> float:
        """Time since the cell was last observed."""
        return time - self._last_seen[cell_index]

    def max_age(self, time: float) -> float:
        """Age of the stalest cell."""
        return time - min(self._last_seen)

    def n_fresh(self, time: float, threshold: float) -> int:
        """Number of cells no older than ``threshold``."""
        return sum(1 for seen in self._last_seen if time - seen <= threshold)

    def __len__(self) -> int:
        return len(self._last_seen)
