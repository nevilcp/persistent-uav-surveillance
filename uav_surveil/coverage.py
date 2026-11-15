"""Cell ages and the coverage metric."""

from __future__ import annotations

import math
from collections.abc import Iterable
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .airspace import Airspace


class CoverageMap:
    """Last-observation time of every cell."""

    def __init__(self, n_cells: int, active: Iterable[int] | None = None) -> None:
        # never-observed cells have infinite age, so coverage starts at zero
        self._last_seen = [-math.inf] * n_cells
        # cells that count towards the metrics; None means all (zone cells are excluded)
        self._active = None if active is None else list(active)

    @classmethod
    def of(cls, airspace: Airspace) -> CoverageMap:
        """Map counting the free cells only (all cells when there are no zones)."""
        free = [c.index for c in airspace.free_cells] if airspace.zones else None
        return cls(len(airspace.grid.cells), free)

    def observe(self, cell_index: int, time: float) -> None:
        self._last_seen[cell_index] = time

    def age(self, cell_index: int, time: float) -> float:
        """Time since the cell was last observed."""
        return time - self._last_seen[cell_index]

    def max_age(self, time: float) -> float:
        """Age of the stalest cell."""
        return time - min(self._seen())

    def n_fresh(self, time: float, threshold: float) -> int:
        """Number of cells no older than ``threshold``."""
        return sum(1 for seen in self._seen() if time - seen <= threshold)

    def _seen(self) -> list[float]:
        if self._active is None:
            return self._last_seen
        return [self._last_seen[i] for i in self._active]

    def __len__(self) -> int:
        return len(self._seen())
