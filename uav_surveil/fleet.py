"""Stage 1: UAV fleet split."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .params import FleetParams


@dataclass(frozen=True)
class FleetSplit:
    n_fly: int
    n_rotate: int
    n_spare: int

    @property
    def total(self) -> int:
        return self.n_fly + self.n_rotate + self.n_spare


def split_fleet(fleet: FleetParams) -> FleetSplit:
    """Optimal flying / rotation / spare split of the inventory."""
    n = fleet.size
    min_spare = math.ceil(round(fleet.reserve_fraction * n, 9))

    best: tuple[tuple[float, int, int], FleetSplit] | None = None
    for n_spare in range(min_spare, n):
        for n_rotate in range(fleet.batch_size, n - n_spare):
            n_fly = n - n_rotate - n_spare
            if n_fly < 1:
                continue
            cost = fleet.fly_cost * n_fly + fleet.reserve_cost * (n_rotate + n_spare)
            # cost ties go to the smaller reserve
            key = (cost, n_rotate + n_spare, n_spare)
            if best is None or key < best[0]:
                best = (key, FleetSplit(n_fly, n_rotate, n_spare))

    if best is None:
        raise ValueError(
            f"fleet of {n} UAVs is infeasible: need N_batch + ceil(rho_min N) <= N - 1 "
            f"with N_batch={fleet.batch_size}, rho_min={fleet.reserve_fraction}"
        )
    return best[1]
