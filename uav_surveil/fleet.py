"""Stage 1: UAV fleet split."""

from __future__ import annotations

import math
from collections.abc import Collection, Sequence
from dataclasses import dataclass
from itertools import combinations

from .airspace import Airspace
from .params import FleetParams, Parameters
from .routing import Loop


@dataclass(frozen=True)
class FleetSplit:
    n_fly: int
    n_rotate: int
    n_spare: int

    @property
    def total(self) -> int:
        return self.n_fly + self.n_rotate + self.n_spare


def apportion(total: int, weights: Sequence[int]) -> list[int]:
    """Integer shares of ``total`` proportional to ``weights`` (largest remainder)."""
    scale = sum(weights)
    quotas = [total * w / scale for w in weights]
    shares = [math.floor(q + 1e-9) for q in quotas]
    by_remainder = sorted(range(len(weights)), key=lambda i: (shares[i] - quotas[i], i))
    for i in by_remainder[: total - sum(shares)]:
        shares[i] += 1
    return shares


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


@dataclass(frozen=True)
class BaseSizing:
    """Loop-fleet figures of one base."""

    index: int
    share: float  # length of the loop stretch served from this base (m)
    n_fly: int  # n_fly,m
    n_rotate: int  # n_rot,m
    spare: bool  # holds a spare UAV
    mean_transit: float  # T-bar_m, mean flight time to its stretch (s)
    arrival_rate: float  # lambda_m, relief requests per second
    utilisation: float  # rho_m = lambda_m tau / kappa_m (0 for unlimited slots)


@dataclass(frozen=True)
class LoopFleet:
    """Fleet needed by the shared loop."""

    n_fly: int
    bases: tuple[BaseSizing, ...]

    @property
    def n_rotate(self) -> int:
        return sum(b.n_rotate for b in self.bases)

    @property
    def n_spare(self) -> int:
        return sum(b.spare for b in self.bases)

    @property
    def total(self) -> int:
        return self.n_fly + self.n_rotate + self.n_spare


def assign_vertices(
    loop: Loop, airspace: Airspace, open_bases: Collection[int]
) -> list[int]:
    """Nearest open base, by flyable distance, of every loop vertex."""
    return [airspace.nearest_depot(v, open_bases) for v in loop.vertices]


def size_loop_fleet(
    loop: Loop,
    airspace: Airspace,
    params: Parameters,
    open_bases: Collection[int] | None = None,
) -> LoopFleet:
    """Flyers, rotation pools and spare bases of the shared loop (Little's law)."""
    uav, patrol = params.uav, params.patrol
    bases = sorted(range(len(airspace.depots)) if open_bases is None else open_bases)
    owner = assign_vertices(loop, airspace, bases)
    n = len(owner)
    # each vertex stands for half of the two segments around it
    before = [loop.arcs[-1] - loop.length, *loop.arcs[:-1]]
    after = [*loop.arcs[1:], loop.length]
    weight = [(after[i] - before[i]) / 2 for i in range(n)]
    share = {m: 0.0 for m in bases}
    transit = {m: 0.0 for m in bases}
    for i, m in enumerate(owner):
        share[m] += weight[i]
        flight = airspace.depot_distance(loop.vertices[i], m) / uav.speed
        transit[m] += weight[i] * flight

    spacing = uav.speed * params.mission.revisit_threshold
    n_fly = math.ceil(loop.length / spacing - 1e-9) + patrol.failures_absorbed
    weights = [round(share[m] * 1000) for m in bases]
    flyers = dict(zip(bases, apportion(n_fly, weights)))
    spares = _spare_bases(loop, airspace, bases, patrol.response_time, uav.speed)

    sizing = []
    for m in bases:
        mean = transit[m] / share[m] if share[m] else 0.0
        window = uav.usable_endurance - 2.0 * mean - patrol.handover_overlap
        if flyers[m] and window <= 0.0:
            raise ValueError(f"base {m} is too far from its stretch of the loop")
        rate = flyers[m] / window if flyers[m] else 0.0
        stay = 2.0 * mean + uav.turnaround + patrol.handover_overlap
        slots = uav.depot_slots
        sizing.append(
            BaseSizing(
                index=m,
                share=share[m],
                n_fly=flyers[m],
                n_rotate=math.ceil(rate * stay - 1e-9)
                + (patrol.rotation_margin if flyers[m] else 0),
                spare=m in spares,
                mean_transit=mean,
                arrival_rate=rate,
                utilisation=rate * uav.turnaround / slots if slots else 0.0,
            )
        )
    return LoopFleet(n_fly, tuple(sizing))


def _spare_bases(
    loop: Loop,
    airspace: Airspace,
    bases: Sequence[int],
    response_time: float,
    speed: float,
) -> tuple[int, ...]:
    """Smallest set of bases from which a spare reaches every loop vertex in time."""
    reach = [
        [airspace.depot_distance(v, m) / speed for m in bases] for v in loop.vertices
    ]
    best: tuple[int, float, tuple[int, ...]] | None = None
    for size in range(1, len(bases) + 1):
        for subset in combinations(range(len(bases)), size):
            worst = max(min(row[j] for j in subset) for row in reach)
            if worst <= response_time:
                key = (size, worst, tuple(bases[j] for j in subset))
                if best is None or key < best:
                    best = key
        if best is not None:
            return best[2]
    raise ValueError(
        f"no set of bases reaches every loop point within T_resp = {response_time:.0f}"
    )
