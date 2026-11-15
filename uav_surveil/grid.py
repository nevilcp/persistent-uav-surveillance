"""Discretisation of the surveillance region into square cells."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .geometry import distance
from .params import Point, RegionParams


@dataclass(frozen=True)
class Cell:
    """Surveillance cell with its centroid."""

    index: int  # row-major position in Grid.cells
    row: int
    col: int
    x: float
    y: float

    @property
    def position(self) -> Point:
        return (self.x, self.y)


class Grid:
    """Uniform grid anchored at the origin."""

    def __init__(self, region: RegionParams) -> None:
        self.cell_size = region.cell_size
        # whole cells only: a 500 m region with 40 m cells surveys 480 m
        self.n_cols = math.floor(region.width / region.cell_size + 1e-9)
        self.n_rows = math.floor(region.length / region.cell_size + 1e-9)
        self.cells: list[Cell] = [
            Cell(
                index=row * self.n_cols + col,
                row=row,
                col=col,
                x=(col + 0.5) * region.cell_size,
                y=(row + 0.5) * region.cell_size,
            )
            for row in range(self.n_rows)
            for col in range(self.n_cols)
        ]

    @property
    def extent(self) -> Point:
        """Upper-right corner of the surveyed block."""
        return (self.n_cols * self.cell_size, self.n_rows * self.cell_size)

    def cell_at(self, position: Point) -> Cell | None:
        """Cell whose footprint contains ``position`` (None outside A)."""
        col = math.floor(position[0] / self.cell_size)
        row = math.floor(position[1] / self.cell_size)
        if 0 <= col < self.n_cols and 0 <= row < self.n_rows:
            return self.cells[row * self.n_cols + col]
        return None

    def cells_within(self, position: Point, radius: float) -> list[Cell]:
        """Cells whose centroid lies within ``radius`` of ``position``."""
        x, y = position
        cols = range(
            max(math.floor((x - radius) / self.cell_size), 0),
            min(math.floor((x + radius) / self.cell_size), self.n_cols - 1) + 1,
        )
        rows = range(
            max(math.floor((y - radius) / self.cell_size), 0),
            min(math.floor((y + radius) / self.cell_size), self.n_rows - 1) + 1,
        )
        covered = []
        for row in rows:
            for col in cols:
                cell = self.cells[row * self.n_cols + col]
                if distance(position, cell.position) <= radius:
                    covered.append(cell)
        return covered

    def ferry_distance(self, depot: Point) -> float:
        """Maximum distance from the depot to the region."""
        # distance to a convex region peaks at a corner
        x_max, y_max = self.extent
        corners = [(0.0, 0.0), (x_max, 0.0), (0.0, y_max), (x_max, y_max)]
        return max(distance(depot, corner) for corner in corners)


def boustrophedon_order(cells: list[Cell]) -> list[Cell]:
    """Back-and-forth row-by-row ordering of the cells."""
    ordered: list[Cell] = []
    for row in sorted({c.row for c in cells}):
        in_row = sorted((c for c in cells if c.row == row), key=lambda c: c.col)
        ordered.extend(in_row if row % 2 == 0 else reversed(in_row))
    return ordered


@dataclass(frozen=True)
class SweepCell:
    """A raster boustrophedon region, bounded by sweep split/merge events."""

    index: int
    cells: tuple[Cell, ...]
    neighbours: tuple[int, ...]


def boustrophedon_decomposition(cells: list[Cell]) -> tuple[SweepCell, ...]:
    """Choset's connectivity-event decomposition on the free-cell raster.

    Sweep upwards through horizontal free intervals. Continue a region only
    across a one-to-one overlap; terminate and create regions at splits,
    merges, births and deaths. Links record the region adjacency graph.
    """
    by_row: dict[int, list[Cell]] = {}
    for cell in cells:
        by_row.setdefault(cell.row, []).append(cell)
    regions: list[list[Cell]] = []
    links: list[set[int]] = []
    previous: list[tuple[list[Cell], int]] = []
    last_row = -2
    for row, row_cells in sorted(by_row.items()):
        if row != last_row + 1:
            previous = []
        intervals: list[list[Cell]] = []
        for cell in sorted(row_cells, key=lambda c: c.col):
            if not intervals or cell.col != intervals[-1][-1].col + 1:
                intervals.append([])
            intervals[-1].append(cell)
        parents = [
            [j for j, (old, _) in enumerate(previous)
             if old[0].col <= run[-1].col and run[0].col <= old[-1].col]
            for run in intervals
        ]
        children = [sum(j in group for group in parents) for j in range(len(previous))]
        current = []
        for run, group in zip(intervals, parents):
            if len(group) == 1 and children[group[0]] == 1:
                index = previous[group[0]][1]
            else:
                index = len(regions)
                regions.append([])
                links.append(set())
                for parent in group:
                    other = previous[parent][1]
                    links[index].add(other)
                    links[other].add(index)
            regions[index].extend(run)
            current.append((run, index))
        previous, last_row = current, row
    return tuple(SweepCell(i, tuple(group), tuple(sorted(links[i])))
                 for i, group in enumerate(regions))


def choset_order(cells: list[Cell]) -> list[Cell]:
    """Exhaust the decomposition graph and sweep each region back and forth.

    Travel between region sweeps (including DFS backtracking) is supplied by
    the flyable-path planner, not by straight-line waypoint jumps.
    """
    regions = boustrophedon_decomposition(cells)
    seen: set[int] = set()
    ordered: list[Cell] = []
    for root in range(len(regions)):
        stack = [root]
        while stack:
            index = stack.pop()
            if index in seen:
                continue
            seen.add(index)
            region = regions[index]
            ordered.extend(boustrophedon_order(list(region.cells)))
            stack.extend(reversed(region.neighbours))
    return ordered
