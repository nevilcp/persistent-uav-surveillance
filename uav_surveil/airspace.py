"""Flyable distances and paths around rectangular no-fly zones."""

from __future__ import annotations

import heapq
import math
from collections.abc import Collection, Iterable

from .geometry import distance
from .grid import Cell, Grid
from .params import Point, Rect, RegionParams

_EPS = 1e-9  # grazing a zone edge counts as clear, matching clearance >= beta
_ATTACH = 1.5  # radius, in cell sizes, within which a free point links to cell centres
_NEIGHBOURS = [(dr, dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]


class Airspace:
    """Free cells, shortest flyable paths and zone clearance."""

    def __init__(
        self,
        grid: Grid,
        zones: Iterable[Rect],
        margin: float,
        depots: Iterable[Point],
    ) -> None:
        self.grid = grid
        self.zones = tuple(zones)
        self.margin = margin
        self.depots = tuple(depots)
        # grazing is allowed, so the open interior is shrunk by _EPS instead of grown
        grow = margin - _EPS
        self._inflated = tuple(
            (x0 - grow, y0 - grow, x1 + grow, y1 + grow)
            for x0, y0, x1, y1 in self.zones
        )
        for index, depot in enumerate(self.depots):
            if self.blocked(depot):
                raise ValueError(f"base {index} lies inside a no-fly zone")
        self._is_free = [not self.blocked(c.position) for c in grid.cells]
        self.free_cells: list[Cell] = [c for c in grid.cells if self._is_free[c.index]]
        self.free_index = {c.index: i for i, c in enumerate(self.free_cells)}
        self._adjacent: list[list[tuple[int, float]]] = []
        self._from_cell: dict[int, tuple[list[float], list[int]]] = {}
        self._depot_tables: list[tuple[list[float], list[int]]] = []
        if self.zones:
            self._build_graph()

    @classmethod
    def from_region(cls, grid: Grid, region: RegionParams) -> Airspace:
        """Airspace described by the region parameters."""
        return cls(grid, region.zones, region.zone_margin, region.bases)

    def blocked(self, p: Point) -> bool:
        """Whether ``p`` lies inside a zone or its margin."""
        return any(
            x0 < p[0] < x1 and y0 < p[1] < y1 for x0, y0, x1, y1 in self._inflated
        )

    def clear(self, p: Point, q: Point) -> bool:
        """Whether the straight segment from ``p`` to ``q`` avoids every zone."""
        return not any(_crosses(p, q, rect) for rect in self._inflated)

    def signed_distance(self, p: Point) -> float:
        """Distance to the nearest zone, negative inside one (inf without zones)."""
        return min((_rect_distance(p, z) for z in self.zones), default=math.inf)

    def distance(self, p: Point, q: Point) -> float:
        """Length of the shortest flyable path from ``p`` to ``q``."""
        if self.clear(p, q):
            return distance(p, q)
        return self._route(p, q)[0]

    def path(self, p: Point, q: Point) -> list[Point]:
        """Shortest flyable polyline from ``p`` to ``q``, both ends included."""
        if self.clear(p, q):
            return [p, q]
        _, a, b = self._route(p, q)
        _, prev = self._cell_table(a)
        return _dedupe([p, *self._chain(prev, b)[::-1], q])

    def depot_distance(self, p: Point, base: int) -> float:
        """Flyable distance from ``p`` to a base."""
        depot = self.depots[base]
        if self.clear(p, depot):
            return distance(p, depot)
        return self._depot_route(p, base)[0]

    def depot_path(self, p: Point, base: int) -> list[Point]:
        """Shortest flyable polyline from ``p`` to a base, both ends included."""
        depot = self.depots[base]
        if self.clear(p, depot):
            return [p, depot]
        _, cell = self._depot_route(p, base)
        _, prev = self._depot_tables[base]
        return _dedupe([p, *self._chain(prev, cell), depot])

    def nearest_depot(self, p: Point, open_bases: Collection[int] | None = None) -> int:
        """Index of the closest open base by flyable distance."""
        candidates = range(len(self.depots)) if open_bases is None else open_bases
        if not candidates:
            raise ValueError("no open base")
        return min(candidates, key=lambda m: (self.depot_distance(p, m), m))

    def _build_graph(self) -> None:
        grid = self.grid
        self._adjacent = [[] for _ in grid.cells]
        for cell in self.free_cells:
            for dr, dc in _NEIGHBOURS:
                row, col = cell.row + dr, cell.col + dc
                if not (0 <= row < grid.n_rows and 0 <= col < grid.n_cols):
                    continue
                other = grid.cells[row * grid.n_cols + col]
                if self._is_free[other.index] and self.clear(
                    cell.position, other.position
                ):
                    self._adjacent[cell.index].append(
                        (other.index, distance(cell.position, other.position))
                    )
        for index, depot in enumerate(self.depots):
            seeds = {
                c.index: distance(depot, c.position)
                for c in self.free_cells
                if self.clear(depot, c.position)
            }
            table = self._dijkstra(seeds)
            if any(math.isinf(table[0][c.index]) for c in self.free_cells):
                raise ValueError(f"free space is not connected to base {index}")
            self._depot_tables.append(table)

    def _dijkstra(self, seeds: dict[int, float]) -> tuple[list[float], list[int]]:
        dist = [math.inf] * len(self.grid.cells)
        prev = [-1] * len(self.grid.cells)
        for node, d in seeds.items():
            dist[node] = d
        heap = [(d, node) for node, d in seeds.items()]
        heapq.heapify(heap)
        while heap:
            d, node = heapq.heappop(heap)
            if d > dist[node]:
                continue
            for other, weight in self._adjacent[node]:
                if d + weight < dist[other]:
                    dist[other] = d + weight
                    prev[other] = node
                    heapq.heappush(heap, (dist[other], other))
        return dist, prev

    def _cell_table(self, source: int) -> tuple[list[float], list[int]]:
        if source not in self._from_cell:
            self._from_cell[source] = self._dijkstra({source: 0.0})
        return self._from_cell[source]

    def _attach(self, p: Point) -> list[tuple[int, float]]:
        """Free cells visible from ``p`` with their distances, nearest ones first."""
        near = self.grid.cells_within(p, _ATTACH * self.grid.cell_size)
        for pool in (near, self.free_cells):
            linked = [
                (c.index, distance(p, c.position))
                for c in pool
                if self._is_free[c.index] and self.clear(p, c.position)
            ]
            if linked:
                return linked
        raise ValueError(f"no flyable link from {p}: point lies inside a no-fly zone")

    def _route(self, p: Point, q: Point) -> tuple[float, int, int]:
        """Length and the cell pair (a, b) the shortest path goes through."""
        best = (math.inf, -1, -1)
        targets = self._attach(q)
        for a, d_pa in self._attach(p):
            dist, _ = self._cell_table(a)
            for b, d_bq in targets:
                total = d_pa + dist[b] + d_bq
                if total < best[0]:
                    best = (total, a, b)
        return best

    def _depot_route(self, p: Point, base: int) -> tuple[float, int]:
        dist, _ = self._depot_tables[base]
        return min((d + dist[a], a) for a, d in self._attach(p))

    def _chain(self, prev: list[int], node: int) -> list[Point]:
        """Cell centres from ``node`` back along ``prev`` to the table's root."""
        cells = self.grid.cells
        points = []
        while node != -1:
            points.append(cells[node].position)
            node = prev[node]
        return points


def _crosses(p: Point, q: Point, rect: Rect) -> bool:
    """Whether segment pq meets the open interior of ``rect`` (slab test)."""
    lo, hi = 0.0, 1.0
    x0, y0, x1, y1 = rect
    for a, b, low, high in ((p[0], q[0], x0, x1), (p[1], q[1], y0, y1)):
        step = b - a
        if step == 0.0:
            if not low < a < high:
                return False
            continue
        t0, t1 = (low - a) / step, (high - a) / step
        lo = max(lo, min(t0, t1))
        hi = min(hi, max(t0, t1))
    return lo < hi


def _rect_distance(p: Point, rect: Rect) -> float:
    """Distance from ``p`` to ``rect``, negative inside."""
    x0, y0, x1, y1 = rect
    dx = max(x0 - p[0], 0.0, p[0] - x1)
    dy = max(y0 - p[1], 0.0, p[1] - y1)
    if dx or dy:
        return math.hypot(dx, dy)
    return -min(p[0] - x0, x1 - p[0], p[1] - y0, y1 - p[1])


def _dedupe(points: list[Point]) -> list[Point]:
    """``points`` without consecutive repeats."""
    return [pt for i, pt in enumerate(points) if i == 0 or pt != points[i - 1]]
