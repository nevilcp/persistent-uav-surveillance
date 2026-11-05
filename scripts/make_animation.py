"""Render the README GIF and an MP4 of each evaluation scenario.

    python scripts/make_animation.py                          # hero GIF, assets/demo.gif
    python scripts/make_animation.py --scenario all --format mp4
    python scripts/make_animation.py --compare under_provisioned nominal over_provisioned
    python scripts/make_animation.py --still 1932 --out frame.png
"""

from __future__ import annotations

import argparse
import math
import multiprocessing as mp
import os
import subprocess
import sys
from collections import deque
from collections.abc import Callable, Iterator
from dataclasses import dataclass, replace
from functools import cache
from pathlib import Path
from typing import NamedTuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection, PatchCollection, PolyCollection
from matplotlib.colors import to_rgb, to_rgba
from matplotlib.patches import Arc, Circle, FancyBboxPatch, Rectangle
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT), str(ROOT / "scripts")]

from run_scenarios import SCENARIOS  # noqa: E402
from uav_surveil import Simulation  # noqa: E402
from uav_surveil.uav import UAVState  # noqa: E402


@dataclass(frozen=True)
class Theme:
    background: str
    card: str  # stage and halo colour
    border: str
    ink: str
    muted: str
    grid: str
    unseen: str  # outline of a tile nobody has observed yet
    ramp: tuple[str, ...]  # fresh -> at the revisit limit, one hue
    overdue: tuple[str, str]  # just past the limit -> far past it
    pulse: str  # flash of a tile that was just observed
    failed: str
    contingency: str
    series: str
    series_fill: str
    recovery_fill: str
    uav: str
    depot: str
    lost: str
    compare: tuple[str, str, str]
    shadow: float  # alpha of the glyph shadow
    dim: float  # how far tiles outside the story fade during a failure


# calm sequential ramp so that only overdue cells are loud
LIGHT = Theme(
    background="#f5f3ee",
    card="#fdfcfa",
    border="#e3dfd6",
    ink="#17212e",
    muted="#667085",
    grid="#e3dfd6",
    unseen="#d3cec3",
    ramp=("#d2e8dc", "#b2d7c5", "#8abfa9", "#5ea48e", "#2f8472"),
    overdue=("#f0a23b", "#d9383a"),
    pulse="#12a37f",
    failed="#7c3aed",
    contingency="#1a6fe8",
    series="#2f4358",
    series_fill="#e4e8ee",
    recovery_fill="#e6dcfb",
    uav="#17212e",
    depot="#3d4a5c",
    lost="#98a0ab",
    compare=("#e08a1e", "#2f8472", "#1a6fe8"),
    shadow=0.2,
    dim=0.55,
)


@dataclass(frozen=True)
class Layout:
    """Pixel positions for a canvas; type sizes are given for 1920 px and scaled by ``k``."""

    size: tuple[int, int]
    k: float
    margin: float
    title: tuple[float, float]
    caption_y: float
    pill_y: float
    stage: tuple[float, float, float, float]  # left, top, width, height
    kpi_x: tuple[float, float, float]
    kpi_y: float  # baseline of the labels
    charts: tuple[tuple[float, float, float, float], tuple[float, float, float, float]] | None
    chart_heads: tuple[float, float]
    badge: tuple[float, float, str]
    legend_y: float
    chapters_y: float


LAYOUTS = {
    "landscape": Layout(
        size=(1920, 1080), k=1.0, margin=64, title=(64, 72), caption_y=114, pill_y=72,
        stage=(48, 140, 1824, 800), kpi_x=(96, 280, 464), kpi_y=206,
        charts=((140, 352, 480, 110), (140, 548, 480, 100)), chart_heads=(334, 530),
        badge=(1856, 114, "right"), legend_y=970, chapters_y=1020,
    ),
    "hero": Layout(
        size=(960, 608), k=0.75, margin=32, title=(32, 42), caption_y=70, pill_y=42,
        stage=(24, 84, 912, 440), kpi_x=(44, 184, 324), kpi_y=116,
        charts=None, chart_heads=(0, 0),
        badge=(44, 226, "left"), legend_y=543, chapters_y=581,
    ),
}

DISPLAY_FONT = ["Inter Display", "Inter", "DejaVu Sans"]
FONT = ["Inter", "DejaVu Sans"]  # Inter when installed, matplotlib's bundled face otherwise
DEFAULT_FPS = {"gif": 20, "mp4": 60}  # GIF delays are whole centiseconds
SPEED = {"gif": 40.0, "mp4": 20.0}  # x real time
SLOW_WINDOW = (-10.0, 240.0)  # half speed, seconds relative to the failure
SLOW_EASE = 40.0  # seconds to ease into and out of the slow window
SLOW_FACTOR = 0.5
TRAIL = 12.0  # seconds of flight path drawn behind each UAV
PULSE = 16.0  # seconds a freshly observed tile keeps its outline ping
BATTERY_ALERT = 0.3  # charge below which a UAV gets a battery arc
TAKEOVER_SHOWN = 100.0  # seconds the taken-over route stays outlined
FAILURE_FLASH = 20.0  # length of the "Failure" chapter
TITLE_HOLD, OUTRO_HOLD, FADE = 3.0, 4.0, 0.6
SHOCKWAVES = 2
GLYPH = np.array([(1.0, 0.0), (-0.85, 0.72), (-0.4, 0.0), (-0.85, -0.72)])  # chevron, nose +x

SCENARIO_LABELS = {
    "under_provisioned": "Under-Provisioned Fleet",
    "nominal": "Nominal Fleet",
    "over_provisioned": "Over-Provisioned Fleet",
    "long_horizon": "Long Horizon",
    "single_failure": "Single UAV Failure",
}


class UavView(NamedTuple):
    id: int
    x: float
    y: float
    state: UAVState
    spare: bool  # the contingency UAV
    battery: float  # 0..1
    heading: float  # radians, from the recent path


@dataclass
class Snapshot:
    """Everything the renderer needs from one instant of the simulation."""

    time: float
    ages: np.ndarray
    uavs: list[UavView]
    trails: list[tuple[list[tuple[float, float]], bool, float]]  # segment, spare, 0 new .. 1 old
    bridges: list[tuple[tuple[float, float], tuple[float, float]]]  # UAV -> bridged cell
    bridged: set[int]
    orphans: set[int]
    failed: tuple[int, float, float] | None
    depot_rotation: int
    depot_spare: int
    depot_turnaround: int
    step: float  # simulated seconds this frame advances
    slow: bool


def smooth(x: float) -> float:
    """Smoothstep of ``x`` clamped to 0..1."""
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def ease_out(x: float) -> float:
    """Cubic ease-out of ``x`` clamped to 0..1."""
    return 1 - (1 - min(max(x, 0.0), 1.0)) ** 3


def slowness(time: float, t_fail: float | None) -> float:
    """0 at full speed, 1 inside the slow window, eased in between."""
    if t_fail is None:
        return 0.0
    a, b = t_fail + SLOW_WINDOW[0], t_fail + SLOW_WINDOW[1]
    return min(smooth((time - (a - SLOW_EASE)) / SLOW_EASE), smooth((b + SLOW_EASE - time) / SLOW_EASE))


def frame_times(
    start: float, end: float, t_fail: float | None, speed: float, fps: float
) -> list[tuple[float, float, bool]]:
    """(time, step, slow) per frame; the step eases down to half speed around the failure."""
    normal = speed / fps  # simulated seconds per frame
    times, t = [(start, normal, False)], start
    while t < end:
        w = slowness(t, t_fail)
        step = normal * (1 - SLOW_FACTOR * w)
        t += step
        times.append((min(t, end), step, w > 0.5))
    return times


class Trails:
    """Recent path and heading of every airborne UAV; lives outside the renderer so frames draw in any order."""

    def __init__(self) -> None:
        self.history: dict[int, list[tuple[float, float, float]]] = {}  # id -> (t, x, y)
        self.facing: dict[int, float] = {}

    def update(
        self, time: float, uavs: list[UavView]
    ) -> list[tuple[list[tuple[float, float]], bool, float]]:
        for u in uavs:
            path = self.history.setdefault(u.id, [])
            path.append((time, u.x, u.y))
            while path[0][0] < time - TRAIL:
                path.pop(0)
            if len(path) > 1:
                dx, dy = path[-1][1] - path[-2][1], path[-1][2] - path[-2][2]
                if math.hypot(dx, dy) > 0.5:
                    self.facing[u.id] = math.atan2(dy, dx)
        alive = {u.id for u in uavs}
        for uid in set(self.history) - alive:  # landed: the next launch starts afresh
            del self.history[uid]
            self.facing.pop(uid, None)
        spares = {u.id for u in uavs if u.spare}
        return [
            ([(x0, y0), (x1, y1)], uid in spares, (time - t1) / TRAIL)
            for uid, path in self.history.items()
            for (_, x0, y0), (t1, x1, y1) in zip(path, path[1:])
        ]


def capture(
    sim: Simulation, step: float, trails: Trails, slow: bool = False
) -> Snapshot:
    """Snapshot of the simulation at its current instant."""
    grid, mgr, t = sim.grid, sim.failure, sim.time
    bridges, bridged = [], set()
    for u in sim.uavs:
        if u.state is UAVState.ON_ROUTE:
            for index in u.inserted:
                bridges.append((u.position, grid.cells[index].position))
                bridged.add(index)
    handed_over = mgr is not None and mgr.t_handover is not None
    orphans = set()
    if mgr is not None and mgr.t_fail is not None and not handed_over:
        orphans = {c.index for c in mgr.route.cells}
    failed = None
    if mgr is not None and mgr.failed_uav is not None:
        v = mgr.failed_uav
        failed = (v.id, *v.position)
    at_depot = [u for u in sim.uavs if u.state is UAVState.READY and u.route is None]
    flying = [
        UavView(u.id, *u.position, u.state, u is (mgr.contingency if mgr else None), u.battery, 0.0)
        for u in sim.uavs
        if u.airborne
    ]
    segments = trails.update(t, flying)
    uavs = [u._replace(heading=trails.facing.get(u.id, 0.0)) for u in flying]
    return Snapshot(
        time=t,
        ages=np.array([sim.coverage.age(i, t) for i in range(len(grid.cells))]),
        uavs=uavs,
        trails=segments,
        bridges=bridges,
        bridged=bridged,
        orphans=orphans,
        failed=failed,
        depot_rotation=sum(not u.is_contingency for u in at_depot),
        depot_spare=sum(u.is_contingency for u in at_depot),
        depot_turnaround=sum(u.state is UAVState.SWAPPING for u in sim.uavs),
        step=step,
        slow=slow,
    )


def phase_text(sim: Simulation, snap: Snapshot) -> str:
    """One-line caption for the current phase, judged from the frame's time."""
    mgr, t = sim.failure, snap.time
    if mgr is not None and mgr.t_handover is not None and t > mgr.t_handover:
        return f"Contingency UAV {mgr.contingency.id} takes over route {mgr.route.id}"
    if mgr is not None and mgr.t_fail is not None and t > mgr.t_fail:
        return (
            f"UAV {mgr.failed_uav.id} fails  ·  neighbours bridge its urgent cells "
            "until the contingency UAV arrives"
        )
    if t <= max(r.departure for r in sim.routes):
        return "Staggered deployment from the depot"
    return "Continuous patrol  ·  low-battery UAVs are relieved from the depot"


@cache
def arrays(theme: Theme) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Ramp, overdue, card and pulse colours as RGB arrays."""
    ramp = np.array([to_rgb(c) for c in theme.ramp])
    hot = np.array([to_rgb(c) for c in theme.overdue])
    return ramp, hot, np.array(to_rgb(theme.card)), np.array(to_rgb(theme.pulse))


def cell_colors(
    ages: np.ndarray, theta: float, theme: Theme, banded: bool, dim: np.ndarray | None = None
) -> tuple[np.ndarray, np.ndarray]:
    """Tile faces and outlines: a calm ramp to the limit, amber to red past it, hollow while unobserved."""
    ramp, hot, card, pulse = arrays(theme)
    seen = np.isfinite(ages)
    x = np.where(seen, ages, 0.0) / theta
    past = np.clip(x - 1, 0, 1)
    if banded:  # a handful of steps keeps the GIF palette and frame deltas small
        base = ramp[np.minimum((x * len(ramp)).astype(int), len(ramp) - 1)]
        over = hot[(past >= 0.5).astype(int)]
    else:
        base = np.stack([np.interp(x, np.linspace(0, 1, len(ramp)), ramp[:, c]) for c in range(3)], 1)
        over = hot[0] + (hot[1] - hot[0]) * past[:, None]
    face = np.where((x > 1)[:, None], over, base)
    flash = np.clip(1 - np.where(seen, ages, np.inf) / PULSE, 0, 1)
    if banded:
        flash = np.round(flash * 3) / 3
    if dim is not None:
        face = face + (card - face) * dim[:, None]
    faces = np.concatenate([face, seen[:, None].astype(float)], 1)
    edges = np.zeros((len(ages), 4))
    edges[~seen] = to_rgba(theme.unseen)
    edges[:, :3] = np.where(seen[:, None], pulse, edges[:, :3])  # a just-observed tile pings with an outline
    edges[:, 3] = np.where(seen, 0.6 * flash, edges[:, 3])
    return faces, edges


def rolling_mean(values: list[float], window: int) -> list[float]:
    """Trailing mean over ``window`` samples."""
    out, acc = [], 0.0
    for i, v in enumerate(values):
        acc += v
        if i >= window:
            acc -= values[i - window]
        out.append(acc / min(i + 1, window))
    return out


class Chapter(NamedTuple):
    name: str
    start: float
    end: float
    colour: str


def chapter_plan(sim: Simulation, end: float, theme: Theme) -> list[Chapter]:
    """Story stages of the run, as equal-width segments of the progress strip."""
    mgr, th = sim.failure, theme
    deploy = max(r.departure for r in sim.routes)
    if mgr is not None and mgr.t_fail is not None:
        t_f = mgr.t_fail
        t_h = mgr.t_handover if mgr.t_handover is not None else end
        t_r = t_f + mgr.recovery_time if mgr.recovery_time is not None else end
        names = ["Deploy", "Patrol", "Failure", "Bridging", "Takeover", "Recovered"]
        colours = [th.series, th.series, th.failed, th.failed, th.contingency, th.ramp[-1]]
        marks = [0.0, deploy, t_f, t_f + FAILURE_FLASH, t_h, t_r, end]
    else:
        names = ["Deploy", "Build-up", "Steady state"]
        colours = [th.series] * 3
        marks = [0.0, deploy, sim.params.mission.steady_state_start, end]
    marks = [min(m, end) for m in np.maximum.accumulate(marks)]
    return [Chapter(n, a, b, c) for n, a, b, c in zip(names, marks, marks[1:], colours)]


class Canvas:
    """A figure addressed in layout pixels, with type sizes scaled by the layout."""

    def __init__(self, layout: Layout, theme: Theme, scale: float) -> None:
        self.layout, self.theme, self.scale, self.k = layout, theme, scale, layout.k
        self.w, self.h = layout.size
        plt.rcParams.update({"font.family": FONT, "text.color": theme.ink})
        self.fig = plt.figure(
            figsize=(self.w / 100, self.h / 100), dpi=100 * scale, facecolor=theme.background
        )

    def fx(self, x: float) -> float:
        return x / self.w

    def fy(self, y: float) -> float:
        return 1 - y / self.h

    def box(self, left: float, top: float, w: float, h: float) -> list[float]:
        return [left / self.w, 1 - (top + h) / self.h, w / self.w, h / self.h]

    def pt(self, px: float) -> float:
        return px * self.k * 0.72

    def text(self, x: float, y: float, s: str, size: float, **kw):
        kw.setdefault("color", self.theme.ink)
        kw.setdefault("va", "baseline")
        return self.fig.text(self.fx(x), self.fy(y), s, fontsize=self.pt(size), **kw)

    def overlay(self, z: float = 0):
        """Full-figure axes in layout pixels, y down."""
        ax = self.fig.add_axes([0, 0, 1, 1], facecolor="none", zorder=z)
        ax.set_xlim(0, self.w)
        ax.set_ylim(self.h, 0)
        ax.axis("off")
        return ax

    def width_of(self, text) -> float:
        return text.get_window_extent(self.fig.canvas.get_renderer()).width / self.scale

    def grab(self) -> Image.Image:
        self.fig.canvas.draw()
        return Image.fromarray(np.asarray(self.fig.canvas.buffer_rgba())).convert("RGB")


class Counter:
    """Number drawn digit by digit on a fixed pitch, so a live counter does not shimmer."""

    def __init__(self, canvas: Canvas, x: float, y: float, size: float, unit: str, unit_size: float, slots: int = 4, **kw):
        self.x, self.canvas = x, canvas
        self.pitch = 0.62 * size * canvas.k
        self.gap = 6 * canvas.k
        self.digits = [
            canvas.text(x + (i + 0.5) * self.pitch, y, "", size, ha="center", **kw)
            for i in range(slots)
        ]
        self.unit = canvas.text(x, y, unit, unit_size, color=canvas.theme.muted)
        self.colour = None

    def set(self, value: str, colour: str | None = None) -> None:
        for digit, char in zip(self.digits, value.ljust(len(self.digits))):
            digit.set_text(char.strip())
        self.unit.set_x(self.canvas.fx(self.x + len(value) * self.pitch + self.gap))
        if colour != self.colour:  # recolouring every frame would be wasted work
            for digit in self.digits:
                digit.set_color(colour or self.canvas.theme.ink)
            self.colour = colour


def curve(p0, p1, bow: float = 0.14, n: int = 18) -> np.ndarray:
    """Quadratic arc between two points, bowed to one side."""
    a, b = np.array(p0, float), np.array(p1, float)
    mid = (a + b) / 2
    d = b - a
    ctrl = mid + np.array([-d[1], d[0]]) * bow
    t = np.linspace(0, 1, n)[:, None]
    return (1 - t) ** 2 * a + 2 * (1 - t) * t * ctrl + t**2 * b


def chevrons(uavs: list[UavView], radii: list[float], shift=(0.0, 0.0)) -> list[np.ndarray]:
    """UAV glyph polygons rotated to each heading."""
    polys = []
    for u, r in zip(uavs, radii):
        c, s = math.cos(u.heading), math.sin(u.heading)
        rot = np.array([[c, -s], [s, c]])
        polys.append((GLYPH - (0.1, 0)) @ rot.T * r + (u.x + shift[0], u.y + shift[1]))
    return polys


class MapView:
    """One scenario's grid, UAVs, depot and failure effects, drawn into an axes of the canvas."""

    def __init__(
        self,
        canvas: Canvas,
        sim: Simulation,
        rect: tuple[float, float, float, float],
        limits: tuple[float, float, float, float],
        *,
        depot: bool = True,
        banded: bool = False,
        fade: bool = True,
    ) -> None:
        th, k = canvas.theme, canvas.k
        self.canvas, self.sim, self.th = canvas, sim, th
        self.banded, self.fade = banded, fade
        left, top, w, h = rect
        xlo, xhi, ylo, yhi = limits
        self.s = w / (xhi - xlo)  # layout px per metre
        u = self.u = 1 / self.s
        self.theta = sim.params.mission.revisit_threshold
        self.r_uav = 0.24 * sim.grid.cell_size * self.s  # px
        self.xhi = xhi

        canvas.overlay(0).add_patch(
            FancyBboxPatch(
                (left, top), w, h, boxstyle=f"round,pad=0,rounding_size={22 * k}",
                fc=th.card, ec=th.border, lw=canvas.pt(1.6),
            )
        )
        ax = self.ax = canvas.fig.add_axes(canvas.box(*rect), zorder=1, facecolor="none")
        ax.set_xlim(xlo, xhi)
        ax.set_ylim(ylo, yhi)
        ax.axis("off")

        mgr = sim.failure
        n = len(sim.grid.cells)
        self.route_mask = np.zeros(n, bool)
        self.t_spot = None
        if mgr is not None and mgr.route is not None:
            self.route_mask[[c.index for c in mgr.route.cells]] = True
            self.t_spot = (mgr.t_fail, mgr.t_fail + (mgr.recovery_time or 250.0))

        half = sim.grid.cell_size / 2 * 0.92  # a thin gap between tiles
        tiles = [
            FancyBboxPatch(
                (c.x - half, c.y - half), 2 * half, 2 * half,
                boxstyle=f"round,pad=0,rounding_size={5 * k * u}",
            )
            for c in sim.grid.cells
        ]
        self.cells = PatchCollection(tiles, linewidths=canvas.pt(1.8), zorder=1)
        ax.add_collection(self.cells)
        self.route_outline = PatchCollection(
            tiles, facecolors="none", edgecolors="none", linewidths=canvas.pt(3.2), zorder=3
        )
        ax.add_collection(self.route_outline)
        self.bridge_outline = [
            PatchCollection(tiles, facecolors="none", edgecolors="none", linewidths=canvas.pt(lw), zorder=z)
            for lw, z in ((6.4, 4), (3.4, 4.1))
        ]
        for artist in self.bridge_outline:
            ax.add_collection(artist)
        self.dash = (3.0, 2.4)  # on, off, in units of the line width
        self.bridge_halo = LineCollection([], colors=th.card, linewidths=canvas.pt(6), zorder=4.2)
        self.bridge_line = LineCollection(
            [], colors=th.contingency, linewidths=canvas.pt(2.8), zorder=4.3, linestyles=[(0, self.dash)]
        )
        ax.add_collection(self.bridge_halo)
        ax.add_collection(self.bridge_line)
        self.trails = LineCollection([], linewidths=canvas.pt(3), capstyle="round", zorder=5)
        ax.add_collection(self.trails)
        self.rings = [
            Circle((0, 0), 1, fill=False, ec=th.failed, lw=0, alpha=0, zorder=4.5)
            for _ in range(SHOCKWAVES)
        ]
        for ring in self.rings:
            ax.add_patch(ring)
        self.battery_halo = LineCollection([], colors=th.card, linewidths=canvas.pt(6), zorder=5.5)
        self.battery_arcs = LineCollection([], linewidths=canvas.pt(3), capstyle="round", zorder=5.6)
        ax.add_collection(self.battery_halo)
        ax.add_collection(self.battery_arcs)
        self.spare_ring = ax.scatter(
            [], [], facecolors="none", edgecolors=th.contingency, linewidths=canvas.pt(2.6), alpha=0.5, zorder=5.8
        )
        self.shadow = PolyCollection([], facecolors=(0, 0, 0, th.shadow), edgecolors="none", zorder=5.9)
        self.glyphs = PolyCollection([], zorder=6)
        ax.add_collection(self.shadow)
        ax.add_collection(self.glyphs)
        self.failed_scatter = ax.scatter(
            [], [], marker="X", s=(0.55 * self.s * sim.grid.cell_size * 0.72) ** 2, color=th.failed,
            edgecolors=th.card, linewidths=canvas.pt(2), zorder=7,
        )
        label_box = {"boxstyle": "round,pad=0.3,rounding_size=0.5", "fc": th.card, "ec": th.border, "lw": 1}
        self.failed_label = ax.text(
            0, 0, "", fontsize=canvas.pt(19), color=th.failed, fontweight="semibold", ha="left", va="bottom",
            zorder=8, bbox=label_box,
        )
        self.spare_label = ax.annotate(
            "", (0, 0), xytext=(0, 0), fontsize=canvas.pt(19), color=th.ink, fontweight="semibold",
            ha="left", va="center", zorder=8, bbox=label_box,
            arrowprops={"arrowstyle": "-", "color": th.ink, "lw": canvas.pt(1.4), "shrinkA": 0, "shrinkB": 10},
        )
        self._build_axes()
        self.depot = depot
        if depot:
            self._build_depot()

    def _build_axes(self) -> None:
        """Metre axes along the bottom and left edges of the region."""
        c, th = self.canvas, self.th
        ax, (xe, ye) = self.ax, self.sim.grid.extent
        size, lw = c.pt(15), c.pt(1.6)
        ax.plot([0, xe], [-4, -4], color=th.muted, lw=lw, solid_capstyle="butt", zorder=1)
        ax.plot([-4, -4], [0, ye], color=th.muted, lw=lw, solid_capstyle="butt", zorder=1)
        for v in range(0, int(xe) + 1, 100):
            ax.plot([v, v], [-4, -10], color=th.muted, lw=lw, zorder=1)
            ax.text(v, -13, f"{v}", fontsize=size, color=th.muted, ha="center", va="top")
        for v in range(0, int(ye) + 1, 100):
            ax.plot([-4, -10], [v, v], color=th.muted, lw=lw, zorder=1)
            ax.text(-13, v, f"{v}", fontsize=size, color=th.muted, ha="right", va="center")
        ax.text(xe + 8, -4, "x (m)", fontsize=size, color=th.ink, ha="left", va="center")
        ax.text(-44, ye / 2, "y (m)", fontsize=size, color=th.ink, ha="center", va="center", rotation=90)

    def _build_depot(self) -> None:
        """A landing pad with the reserve UAVs queued above it."""
        c, th, k, sim = self.canvas, self.th, self.canvas.k, self.sim
        ax = self.ax
        xd = sim.params.region.depot[0]
        ax.add_patch(Circle((xd, 0), 26, fc=th.card, ec=th.depot, lw=c.pt(3), zorder=5))
        ax.text(xd, -1, "H", fontsize=c.pt(24), fontweight="bold", color=th.depot, ha="center", va="center", zorder=5.1)
        ax.text(xd - 42, 3, "Depot", fontsize=c.pt(22), fontweight="semibold", color=th.ink, ha="right", va="center")
        n = sim.split.n_rotate + sim.split.n_spare
        pitch = min(32.0, 220.0 / max(n - 1, 1))
        self.slot_xy = [(xd + (i - (n - 1) / 2) * pitch, 66) for i in range(n)]
        self.slots = [Circle(xy, 10.5, zorder=5) for xy in self.slot_xy]
        for slot in self.slots:
            ax.add_patch(slot)
        ax.text(xd, 92, "reserve UAVs", fontsize=c.pt(17), color=th.muted, ha="center", va="bottom")

    def _spot(self, t: float) -> float:
        """Strength of the fade over tiles outside the failure story."""
        if self.t_spot is None:
            return 0.0
        a, b = self.t_spot
        w = smooth((t - a) / 12) * (1 - smooth((t - b) / 40))
        return round(w * 2) / 2 if self.banded else w

    @staticmethod
    def _edges(n: int, mask: np.ndarray, colour: str | None) -> np.ndarray:
        edges = np.zeros((n, 4))
        if colour is not None:
            edges[mask] = to_rgba(colour)
        return edges

    def draw(self, snap: Snapshot) -> None:
        th, mgr = self.th, self.sim.failure
        n = len(snap.ages)
        bridged = np.zeros(n, bool)
        bridged[np.fromiter(snap.bridged, dtype=int)] = True
        dim = self._spot(snap.time) * th.dim * ~(self.route_mask | bridged)
        faces, edges = cell_colors(snap.ages, self.theta, th, self.banded, dim)
        self.cells.set_facecolor(faces)
        self.cells.set_edgecolor(edges)

        route_colour = None
        if snap.orphans:
            route_colour = th.failed
        elif mgr and mgr.t_handover is not None and 0 < snap.time - mgr.t_handover <= TAKEOVER_SHOWN:
            route_colour = th.contingency
        self.route_outline.set_edgecolor(self._edges(n, self.route_mask, route_colour))
        for layer, colour in zip(self.bridge_outline, (th.card, th.contingency)):
            layer.set_edgecolor(self._edges(n, bridged, colour))
        arcs = [curve(a, b) for a, b in snap.bridges]
        self.bridge_halo.set_segments(arcs)
        self.bridge_line.set_segments(arcs)
        period = sum(self.dash)
        self.bridge_line.set_linestyle([((period - 0.12 * snap.time) % period, self.dash)])  # dashes run to the cell

        self._draw_trails(snap)
        self._draw_uavs(snap)
        self._draw_rings(snap)
        if self.depot:
            self._draw_depot(snap)

    def _draw_trails(self, snap: Snapshot) -> None:
        th = self.th
        ink, blue = to_rgb(th.uav), to_rgb(th.contingency)
        # flat tone for the GIF: a per-frame fade would change every trail pixel
        alpha = (lambda f: 0.6 * (1 - f)) if self.fade else (lambda f: 0.4)
        self.trails.set_segments([seg for seg, _, _ in snap.trails])
        self.trails.set_color([(*(blue if spare else ink), alpha(f)) for _, spare, f in snap.trails])

    def _draw_rings(self, snap: Snapshot) -> None:
        mgr = self.sim.failure
        age = snap.time - mgr.t_fail if mgr is not None and mgr.t_fail is not None else None
        for i, ring in enumerate(self.rings):
            p = None if age is None or snap.failed is None else (age - 7.0 * i) / 22.0
            if p is None or not 0 <= p <= 1:
                ring.set_alpha(0)
                continue
            ring.set_center(snap.failed[1:])
            ring.set_radius(14 + 150 * ease_out(p))
            ring.set_alpha(0.9 * (1 - p))
            ring.set_linewidth(self.canvas.pt(5) * (1 - 0.6 * p))

    def _draw_uavs(self, snap: Snapshot) -> None:
        th, c, u = self.th, self.canvas, self.u
        if snap.uavs:
            radii = [self.r_uav * u * (1.45 if v.spare else 1.0) for v in snap.uavs]
            self.glyphs.set_verts(chevrons(snap.uavs, radii))
            self.shadow.set_verts(chevrons(snap.uavs, radii, (1.6 * c.k * u, -2.4 * c.k * u)))
            hollow = [v.state is UAVState.RETURNING for v in snap.uavs]
            self.glyphs.set_facecolor(
                [th.contingency if v.spare else (th.card if h else th.uav) for v, h in zip(snap.uavs, hollow)]
            )
            self.glyphs.set_edgecolor([th.uav if h else th.card for h in hollow])
            self.glyphs.set_linewidth([c.pt(2.4) if h else c.pt(2.2) for h in hollow])
            spares = [(v.x, v.y) for v in snap.uavs if v.spare]
            self.spare_ring.set_offsets(spares or np.empty((0, 2)))
            self.spare_ring.set_sizes([(2.9 * self.r_uav * 0.72) ** 2] * len(spares))
            low = [v for v in snap.uavs if v.battery < BATTERY_ALERT]
            arcs, colours = [], []
            for v in low:
                radius = (self.r_uav * (1.45 if v.spare else 1.0) + 7 * c.k) * u
                angle = np.pi / 2 - np.linspace(0, 2 * np.pi * v.battery, max(int(v.battery * 40), 2))
                arcs.append(np.column_stack([v.x + radius * np.cos(angle), v.y + radius * np.sin(angle)]))
                colours.append(th.overdue[1] if v.battery < 0.2 else th.overdue[0])
            self.battery_halo.set_segments(arcs)
            self.battery_arcs.set_segments(arcs)
            self.battery_arcs.set_color(colours)
        else:
            for artist in (self.glyphs, self.shadow, self.battery_halo, self.battery_arcs):
                artist.set_segments([]) if isinstance(artist, LineCollection) else artist.set_verts([])
            self.spare_ring.set_offsets(np.empty((0, 2)))

        mgr = self.sim.failure
        if snap.failed:
            fid, fx_, fy_ = snap.failed
            self.failed_scatter.set_offsets([(fx_, fy_)])
            # once the route is taken over the wreck is history: grey, unlabelled
            replaced = mgr.t_handover is not None and snap.time > mgr.t_handover
            self.failed_scatter.set_facecolor(th.lost if replaced else th.failed)
            self.failed_label.set_position((fx_ + 20 * c.k * u, fy_ + 14 * c.k * u))
            self.failed_label.set_text("" if replaced else f"UAV {fid} failed")
        else:
            self.failed_scatter.set_offsets(np.empty((0, 2)))
            self.failed_label.set_text("")

        # name the contingency UAV while it is the story; afterwards it is just another UAV
        spare = next((v for v in snap.uavs if v.spare), None)
        if spare and mgr.t_handover is not None and snap.time > mgr.t_handover + TAKEOVER_SHOWN:
            spare = None
        if spare:
            near_edge = spare.x > self.xhi - 210 * c.k * u
            dx, dy = 58 * c.k * u, 46 * c.k * u
            self.spare_label.set_ha("right" if near_edge else "left")
            self.spare_label.xy = (spare.x, spare.y)
            self.spare_label.set_position((spare.x + (-dx if near_edge else dx), spare.y + dy))
            self.spare_label.set_text(f"contingency UAV {spare.id}")
            self.spare_label.arrow_patch.set_visible(True)
        else:
            self.spare_label.set_text("")
            self.spare_label.arrow_patch.set_visible(False)

    def _draw_depot(self, snap: Snapshot) -> None:
        """Reserve slots: filled = ready, ring = in turnaround, faint = away flying."""
        th, n_rotate = self.th, self.sim.split.n_rotate
        for k, slot in enumerate(self.slots):
            if k < n_rotate:
                ready = k < snap.depot_rotation
                turn = not ready and k < snap.depot_rotation + snap.depot_turnaround
                colour = th.depot
            else:
                ready, turn, colour = snap.depot_spare > 0, False, th.contingency
            if ready:
                slot.set_facecolor(colour)
                slot.set_edgecolor(th.card)
            elif turn:
                slot.set_facecolor(th.card)
                slot.set_edgecolor(th.depot)
            else:
                slot.set_facecolor(th.grid)
                slot.set_edgecolor(th.border)
            slot.set_linewidth(self.canvas.pt(2.4))


class Renderer:
    """Figure built once, with only the moving parts redrawn per frame."""

    def __init__(
        self, sim: Simulation, end: float, label: str, fps: float, scale: float,
        theme: Theme = LIGHT, layout: Layout = LAYOUTS["landscape"], banded: bool = False,
    ) -> None:
        self.sim, self.end, self.label, self.fps = sim, end, label, fps
        self.th, self.layout, self.banded = theme, layout, banded
        self.theta = sim.params.mission.revisit_threshold
        self.tick_step = 600 if end <= 2400 else 1200 if end <= 4800 else 2400
        rows = sim.monitor.rows
        self.t = [r["time"] for r in rows]
        self.coverage_now = [r["coverage_pct"] for r in rows]
        self.coverage = rolling_mean(self.coverage_now, 60)
        age = np.array([r["max_age"] for r in rows], dtype=float)
        self.max_age = np.where(np.isfinite(age), age, np.nan)  # inf until the first sweep
        mgr = sim.failure
        self.t_rec = mgr.recovery_time if mgr else None
        self.chapters = chapter_plan(sim, end, theme)
        self.canvas = Canvas(layout, theme, scale)
        self.fig = self.canvas.fig
        self._build()

    def _build(self) -> None:
        c, th, lay, p = self.canvas, self.th, self.layout, self.sim.params
        w, h = lay.size
        c.text(*lay.title, "Persistent UAV Surveillance", 46, fontfamily=DISPLAY_FONT, fontweight="semibold")
        self.caption = c.text(lay.margin, lay.caption_y, "", 25, color=th.muted)
        c.text(
            w - lay.margin, lay.pill_y, f"{self.label}  ·  N = {p.fleet.size}", 22, color=th.muted, ha="right"
        )

        _, top, sw, sh = lay.stage
        left = lay.stage[0]
        x_depot, extent = p.region.depot[0], self.sim.grid.extent
        s = min(sw / (extent[0] + 30 - (x_depot - 95)), sh / (extent[1] + 60))
        limits = (extent[0] + 30 - sw / s, extent[0] + 30, -30.0, -30.0 + sh / s)
        self.view = MapView(c, self.sim, lay.stage, limits, banded=self.banded, fade=not self.banded)

        x0, x1, x2 = lay.kpi_x
        y = lay.kpi_y
        value_y = y + 62 * c.k
        for x, name in ((x0, "Simulated time"), (x1, "Coverage now"), (x2, "Oldest cell")):
            c.text(x, y, name, 20, color=th.muted)
        kw = {"fontfamily": DISPLAY_FONT, "fontweight": "medium"}
        self.clock = Counter(c, x0, value_y, 60, "s", 28, **kw)
        self.percent = Counter(c, x1, value_y, 60, "%", 28, slots=3, **kw)
        self.oldest = Counter(c, x2, value_y, 60, "s", 28, **kw)
        self.speed = c.text(x0, value_y + 30 * c.k, "", 18, color=th.muted)
        self.over = c.text(
            x2 + 126 * c.k, y, "OVER LIMIT", 15, color=th.card, fontweight="bold", va="baseline",
            bbox={"boxstyle": "round,pad=0.35,rounding_size=0.6", "fc": th.overdue[1], "ec": "none"},
        )
        self.over.set_visible(False)

        bx, by, ha = lay.badge
        self.badge = c.text(
            bx, by, "SLOW-MO  ½×", 18, color=th.failed, fontweight="bold", ha=ha,
            bbox={"boxstyle": "round,pad=0.4,rounding_size=0.7", "fc": th.card, "ec": th.failed, "lw": 1.4},
        )
        self.badge.set_visible(False)

        if lay.charts:
            self._build_charts()
        self._build_legend()
        self._build_chapters()

    def _chart(self, rect, top: float, yticks: list[float], labels: bool):
        """Axes styled like the rest of the figure, with the failure marked."""
        c, th = self.canvas, self.th
        a = c.fig.add_axes(c.box(*rect), facecolor="none", zorder=2)
        a.set_xlim(0, self.end)
        a.set_ylim(0, top)
        a.set_xticks(np.arange(0, self.end + 1, self.tick_step))
        a.set_yticks(yticks)
        a.tick_params(labelsize=c.pt(17), colors=th.muted, length=0, pad=6 * c.k, labelbottom=labels)
        a.grid(axis="y", color=th.grid, lw=c.pt(1.4))
        a.set_axisbelow(True)
        for side in ("top", "right", "left"):
            a.spines[side].set_visible(False)
        a.spines["bottom"].set_color(th.grid)
        failure = self.sim.params.failure
        if failure.enabled and failure.time <= self.end:
            a.axvline(failure.time, color=th.failed, lw=c.pt(1.8), ls=(0, (3, 3)), zorder=2)
        return a

    def _build_charts(self) -> None:
        c, th, p, lay = self.canvas, self.th, self.sim.params, self.layout
        (r1, r2), (h1, h2) = lay.charts, lay.chart_heads
        c.text(lay.kpi_x[0], h1, "Coverage now  ·  60 s mean", 20, color=th.muted)
        c.text(lay.kpi_x[0], h2, "Oldest cell vs revisit limit", 20, color=th.muted)
        a = self.chart = self._chart(r1, 100, [0, 50, 100], labels=False)
        a.axhline(90, color=th.muted, lw=c.pt(1.4), ls=(0, (1, 3)))
        a.text(self.end * 0.012, 86, "90 %", fontsize=c.pt(17), color=th.muted, ha="left", va="top")
        if p.failure.enabled and p.failure.time <= self.end:
            a.text(
                p.failure.time - self.end * 0.012, 6, "failure", fontsize=c.pt(17), color=th.failed,
                ha="right", va="bottom",
            )
            self.recovery = a.axvspan(p.failure.time, p.failure.time, color=th.recovery_fill, lw=0, zorder=1.6)
            self.bracket = None
            if self.t_rec is not None:
                t0, t1 = p.failure.time, p.failure.time + self.t_rec
                self.bracket = a.annotate(
                    f"recovered in {self.t_rec:.0f} s", xy=(t0, -0.2), xytext=(t1, -0.2),
                    xycoords=a.get_xaxis_transform(), textcoords=a.get_xaxis_transform(),
                    annotation_clip=False, fontsize=c.pt(17), color=th.failed, ha="left", va="center",
                    arrowprops={"arrowstyle": "|-|,widthA=0.3,widthB=0.3", "color": th.failed,
                                "lw": c.pt(2), "shrinkA": 0, "shrinkB": 0},
                )
                self.bracket.set_visible(False)
                self.bracket.arrow_patch.set_visible(False)
        self.area = PolyCollection([], facecolors=th.series_fill, edgecolors="none", zorder=1.5)
        a.add_collection(self.area)
        (self.line,) = a.plot([], [], color=th.series, lw=c.pt(3), solid_capstyle="round", zorder=3)
        (self.head,) = a.plot([], [], "o", color=th.series, mec=th.card, mew=c.pt(2), ms=c.pt(11), zorder=4, clip_on=False)

        finite = self.max_age[np.isfinite(self.max_age)]
        top = min(max(1.15 * finite.max() if finite.size else 0.0, 1.4 * self.theta), 6 * self.theta)
        b = self.chart2 = self._chart(r2, top, [0, self.theta], labels=True)
        b.axhline(self.theta, color=th.muted, lw=c.pt(1.4), ls=(0, (1, 3)))
        b.text(self.end * 0.012, self.theta + 0.03 * top, f"{self.theta:.0f} s limit", fontsize=c.pt(17),
               color=th.muted, ha="left", va="bottom")
        (self.age_line,) = b.plot([], [], color=th.series, lw=c.pt(3), solid_capstyle="round", zorder=3)
        (self.age_hot,) = b.plot([], [], color=th.overdue[1], lw=c.pt(3), solid_capstyle="round", zorder=3.1)
        (self.age_head,) = b.plot([], [], "o", color=th.series, mec=th.card, mew=c.pt(2), ms=c.pt(11), zorder=4, clip_on=False)

    def _build_legend(self) -> None:
        """Cell colours and UAV symbols, drawn by hand in one row."""
        c, th, lay = self.canvas, self.th, self.layout
        ax = c.overlay(2)
        y, k = lay.legend_y, c.k
        x, limit = lay.margin, lay.size[0] - lay.margin
        sw = 15 * k

        def swatch(sx: float, colour, hollow: bool = False) -> FancyBboxPatch:
            return FancyBboxPatch(
                (sx, y - sw / 2), sw, sw, boxstyle=f"round,pad=0,rounding_size={3 * k}",
                fc="none" if hollow else colour, ec=th.unseen if hollow else "none", lw=c.pt(1.6),
            )

        def glyph(gx: float, face, edge) -> plt.Polygon:
            poly = (GLYPH - (0.1, 0)) * 12 * k * (1, -1) + (gx, y)
            return plt.Polygon(poly, fc=face, ec=edge, lw=c.pt(2))

        def arc(gx: float) -> list:
            return [
                Circle((gx, y), 3 * k, fc=th.uav, lw=0),
                Arc((gx, y), 18 * k, 18 * k, theta1=100, theta2=380, color=th.overdue[0], lw=c.pt(2.6)),
            ]

        entries = [
            (f"cell age 0 → {self.theta:.0f} s", "ramp"),
            ("overdue", "hot"),
            ("not yet observed", "unseen"),
            ("UAV on patrol", "patrol"),
            ("returning", "back"),
            (f"battery below {BATTERY_ALERT * 100:.0f} %", "battery"),
        ]
        if self.sim.params.failure.enabled:
            entries.append(("contingency UAV", "spare"))
        for label, kind in entries:
            if kind == "ramp":
                marks = [swatch(x + i * (sw + 2 * k), col) for i, col in enumerate(th.ramp)]
                span = len(th.ramp) * (sw + 2 * k) + 4 * k
            elif kind == "hot":
                marks = [swatch(x + i * (sw + 2 * k), col) for i, col in enumerate(th.overdue)]
                span = 2 * (sw + 2 * k) + 4 * k
            elif kind == "unseen":
                marks, span = [swatch(x, None, hollow=True)], sw + 8 * k
            elif kind == "battery":
                marks, span = arc(x + 9 * k), 26 * k
            else:
                face, edge = {
                    "patrol": (th.uav, th.card), "back": (th.card, th.uav), "spare": (th.contingency, th.card)
                }[kind]
                marks, span = [glyph(x + 9 * k, face, edge)], 28 * k
            text = c.text(x + span, y + 6.5 * k, label, 18, color=th.muted)
            end = x + span + c.width_of(text)
            if end > limit:  # drop what does not fit rather than crowd it
                text.remove()
                break
            for mark in marks:
                ax.add_patch(mark)
            x = end + 30 * k

    def _build_chapters(self) -> None:
        c, th, lay = self.canvas, self.th, self.layout
        ax = c.overlay(2)
        n, gap = len(self.chapters), 10 * c.k
        left, right = lay.margin, lay.size[0] - lay.margin
        self.seg = (right - left - gap * (n - 1)) / n
        self.fills, self.names = [], []
        bar = 7 * c.k
        for i, ch in enumerate(self.chapters):
            x = left + i * (self.seg + gap)
            ax.add_patch(Rectangle((x, lay.chapters_y + 12 * c.k), self.seg, bar, fc=th.grid, ec="none"))
            fill = Rectangle((x, lay.chapters_y + 12 * c.k), 0, bar, fc=ch.colour, ec="none")
            ax.add_patch(fill)
            self.fills.append(fill)
            self.names.append(c.text(x, lay.chapters_y, ch.name, 19, color=th.muted, alpha=0.55))

    def _update_chapters(self, t: float) -> None:
        for ch, fill, name in zip(self.chapters, self.fills, self.names):
            frac = 1.0 if t >= ch.end else 0.0 if t <= ch.start else (t - ch.start) / (ch.end - ch.start)
            fill.set_width(frac * self.seg)
            current = 0 < frac < 1
            name.set_color(self.th.ink if current else self.th.muted)
            name.set_alpha(1.0 if frac > 0 else 0.55)
            name.set_fontweight("semibold" if current else "normal")

    def draw(self, snap: Snapshot, final: bool = False) -> Image.Image:
        sim, th = self.sim, self.th
        self.view.draw(snap)
        self.caption.set_text(self._summary() if final else phase_text(sim, snap))
        self.badge.set_visible(snap.slow and not final)
        self.clock.set(f"{snap.time:.0f}")
        self.speed.set_text(f"{snap.step * self.fps:.0f}× real time")

        i = min(max(int(snap.time) - 1, 0), len(self.t) - 1)  # row of this instant in the 1 Hz series
        self.percent.set(f"{self.coverage_now[i]:.0f}" if snap.time > 0 else "0")
        age = self.max_age[i] if snap.time > 0 else np.nan
        seen = np.isfinite(age)
        breach = seen and age > self.theta
        self.oldest.set(f"{age:.0f}" if seen else "–", th.overdue[1] if breach else None)
        self.over.set_visible(bool(breach))

        if self.layout.charts:
            t, cov = self.t[: i + 1], self.coverage[: i + 1]
            self.line.set_data(t, cov)
            self.head.set_data([t[-1]], [cov[-1]])
            self.area.set_verts([[(t[0], 0), *zip(t, cov), (t[-1], 0)]])
            ages = self.max_age[: i + 1]
            self.age_line.set_data(t, ages)
            self.age_hot.set_data(t, np.where(ages > self.theta, ages, np.nan))
            self.age_head.set_data([t[-1]] if seen else [], [age] if seen else [])
            self.age_head.set_color(th.overdue[1] if breach else th.series)
            self._draw_recovery(snap)
        self._update_chapters(snap.time)
        return self.canvas.grab()

    def _summary(self) -> str:
        """Caption of the held last frame: the headline number of the run."""
        mgr = self.sim.failure
        if mgr is not None and self.t_rec is not None:
            return f"Route {mgr.route.id} recovered: every orphan cell re-observed {self.t_rec:.0f} s after the failure"
        c = self.sim.monitor.summary(mgr)["coverage"]
        text = f"Mean coverage {c['mean_pct']:.1f} %"
        if c["steady_state_mean_pct"] is not None:  # undefined for runs shorter than t_ss
            text += f"  ·  steady state {c['steady_state_mean_pct']:.1f} %"
        return text

    def _draw_recovery(self, snap: Snapshot) -> None:
        mgr = self.sim.failure
        if mgr is None or mgr.t_fail is None or self.t_rec is None or not self.sim.params.failure.enabled:
            return
        t_end = mgr.t_fail + self.t_rec
        self.recovery.set_width(max(min(snap.time, t_end) - mgr.t_fail, 0.0))
        done = snap.time >= t_end
        if self.bracket is not None:
            self.bracket.set_visible(done)
            self.bracket.arrow_patch.set_visible(done)


class CompareRenderer:
    """Scenarios side by side: one map and one headline number each, one chart overlaying them."""

    def __init__(self, sims: list[Simulation], labels: list[str], end: float, fps: float, scale: float, theme: Theme) -> None:
        self.sims, self.end, self.fps, self.th = sims, end, fps, theme
        lay = replace(LAYOUTS["landscape"], title=(64, 78), caption_y=120)
        c = self.canvas = Canvas(lay, theme, scale)
        self.fig = c.fig
        c.text(*lay.title, "Fleet size, side by side", 46, fontfamily=DISPLAY_FONT, fontweight="semibold")
        self.caption = c.text(64, 120, "Same region, same patrol: how many UAVs does persistent coverage take?", 25, color=theme.muted)
        c.text(1856 - 190, 66, "Simulated time", 20, color=theme.muted, ha="right")
        self.clock = Counter(c, 1856 - 170, 78, 52, "s", 26, fontfamily=DISPLAY_FONT, fontweight="medium")
        side, gap = 560, 40
        left = (lay.size[0] - 3 * side - 2 * gap) / 2
        self.views, self.percents, self.slots = [], [], []
        for i, (sim, label) in enumerate(zip(sims, labels)):
            x = left + i * (side + gap)
            colour = theme.compare[i]
            c.overlay(2).add_patch(Circle((x + 9, 186), 7, fc=colour, ec="none"))
            c.text(x + 26, 193, f"{label}  ·  N = {sim.params.fleet.size}", 24, fontweight="semibold")
            extent = sim.grid.extent
            self.views.append(MapView(c, sim, (x, 212, side, side), (-30, extent[0] + 30, -30, extent[1] + 30), depot=False))
            self.percents.append(Counter(c, x + 8, 872, 60, "%", 28, slots=3, fontfamily=DISPLAY_FONT, fontweight="medium", color=colour))
            c.text(x + 8, 810, "Coverage now", 20, color=theme.muted)
            n = sim.split.n_rotate + sim.split.n_spare
            ov = c.overlay(2)
            slots = [Circle((x + side - 14 - (n - 1 - j) * 26, 848), 9, ec=theme.border, lw=1.6) for j in range(n)]
            for slot in slots:
                ov.add_patch(slot)
            self.slots.append(slots)
            c.text(x + side, 810, "reserve UAVs", 20, color=theme.muted, ha="right")
        a = self.chart = c.fig.add_axes(c.box(left + 60, 938, 3 * side + 2 * gap - 60, 76), facecolor="none", zorder=2)
        a.set_xlim(0, end)
        a.set_ylim(0, 100)
        step = 600 if end <= 2400 else 1200 if end <= 4800 else 2400
        a.set_xticks(np.arange(0, end + 1, step))
        a.set_yticks([0, 50, 100])
        a.tick_params(labelsize=c.pt(17), colors=theme.muted, length=0, pad=6)
        a.grid(axis="y", color=theme.grid, lw=c.pt(1.4))
        a.set_axisbelow(True)
        for side_ in ("top", "right", "left"):
            a.spines[side_].set_visible(False)
        a.spines["bottom"].set_color(theme.grid)
        a.axhline(90, color=theme.muted, lw=c.pt(1.4), ls=(0, (1, 3)))
        c.text(left, 922, "Coverage now · 60 s mean", 20, color=theme.muted)
        self.rows = [[r["coverage_pct"] for r in sim.monitor.rows] for sim in sims]
        self.means = [rolling_mean(r, 60) for r in self.rows]
        self.times = [r["time"] for r in sims[0].monitor.rows]
        self.lines = [a.plot([], [], color=col, lw=c.pt(3.4), solid_capstyle="round", zorder=3)[0] for col in theme.compare]
        self.heads = [a.plot([], [], "o", color=col, mec=theme.card, mew=c.pt(2), ms=c.pt(11), zorder=4, clip_on=False)[0] for col in theme.compare]

    def draw(self, snaps: tuple[Snapshot, ...], final: bool = False) -> Image.Image:
        th = self.th
        t = snaps[0].time
        self.clock.set(f"{t:.0f}")
        i = min(max(int(t) - 1, 0), len(self.times) - 1)
        for k, snap in enumerate(snaps):
            self.views[k].draw(snap)
            self.percents[k].set(f"{self.rows[k][i]:.0f}" if t > 0 else "0")
            n_rotate = self.sims[k].split.n_rotate
            for j, slot in enumerate(self.slots[k]):
                if j < n_rotate:
                    ready = j < snap.depot_rotation
                    turn = not ready and j < snap.depot_rotation + snap.depot_turnaround
                    colour = th.depot
                else:
                    ready, turn, colour = snap.depot_spare > 0, False, th.contingency
                slot.set_facecolor(colour if ready else th.card if turn else th.grid)
                slot.set_edgecolor(th.depot if turn else th.card if ready else th.border)
            self.lines[k].set_data(self.times[: i + 1], self.means[k][: i + 1])
            self.heads[k].set_data([self.times[i]], [self.means[k][i]])
        if final:
            steady = [
                sim.monitor.summary(None)["coverage"]["steady_state_mean_pct"] for sim in self.sims
            ]
            self.caption.set_text(
                "Steady-state coverage  ·  " + "  ·  ".join(
                    f"N = {sim.params.fleet.size}: {pct:.1f} %" for sim, pct in zip(self.sims, steady)
                )
            )
        return self.canvas.grab()


class Card:
    """Full-frame card with a mosaic of tiles behind the text, which lights up as a wave passes."""

    def __init__(self, layout: Layout, theme: Theme, scale: float) -> None:
        # type sizes follow the canvas width, not the layout's own scale
        self.canvas = c = Canvas(replace(layout, k=layout.size[0] / 1920), theme, scale)
        self.th = theme
        w, h = c.w, c.h
        n = 24
        tile = w / n
        rng = np.random.default_rng(7)
        rows = math.ceil(h / tile)
        self.cx = np.array([(i + 0.5) * tile for _ in range(rows) for i in range(n)])
        self.rest = 0.05 + 0.12 * rng.random(n * rows)
        ramp = arrays(theme)[0]
        self.lit = ramp[rng.integers(0, len(ramp), n * rows)]
        tiles = [
            FancyBboxPatch(
                (i * tile + 4, j * tile + 4), tile - 8, tile - 8, boxstyle=f"round,pad=0,rounding_size={tile / 9}"
            )
            for j in range(rows)
            for i in range(n)
        ]
        self.tiles = PatchCollection(tiles, linewidths=0)
        c.overlay(0).add_collection(self.tiles)
        self.strength = 0.5

    def mosaic(self, progress: float) -> None:
        """Tiles lit behind a wave sweeping left to right."""
        w = self.canvas.w
        front = (progress * 1.35 - 0.2) * w
        behind = np.clip(1 - (front - self.cx) / (0.4 * w), 0, 1)
        alpha = np.where(self.cx < front, self.rest + (self.strength - self.rest) * behind, 0.0)
        self.tiles.set_facecolor(np.concatenate([self.lit, alpha[:, None]], 1))

    def fade(self, text, a: float, y: float | None = None) -> None:
        text.set_alpha(a)
        if y is not None:
            text.set_y(self.canvas.fy(y + (1 - a) * 18 * self.canvas.k))


def title_frames(params, label: str, theme: Theme, layout: Layout, scale: float, fps: float) -> Iterator[Image.Image]:
    """Opening card: the title settles, the colour ramp sweeps in, the run's numbers count up."""
    card = Card(layout, theme, scale)
    c, th = card.canvas, theme
    cx, h = c.w / 2, c.h
    title = c.text(cx, h * 0.45, "Persistent UAV Surveillance", 92, fontfamily=DISPLAY_FONT, fontweight="semibold", ha="center")
    sub = c.text(cx, h * 0.45 + 70 * c.k, label, 44, color=th.muted, ha="center")
    ramp = [*th.ramp, th.overdue[1]]
    seg = 70 * c.k
    ov = c.overlay(2)
    bars = [
        ov.add_patch(Rectangle((cx + (k - len(ramp) / 2) * seg, h * 0.45 + 108 * c.k), 0, 7 * c.k, fc=colour, ec="none"))
        for k, colour in enumerate(ramp)
    ]
    stats = [
        (params.fleet.size, "{} UAVs", "Fleet"),
        (params.mission.duration / 60, "{} min", "Simulated Duration"),
        (params.mission.revisit_threshold, "{} s", "Revisit Limit"),
    ]
    if params.failure.enabled:
        stats.append((params.failure.time, "t = {} s", "Failure Injected"))
    values, captions = [], []
    for k, (_, _, caption) in enumerate(stats):
        x = cx + (k - (len(stats) - 1) / 2) * 0.22 * c.w
        values.append(c.text(x, h * 0.45 + 270 * c.k, "", 56, fontfamily=DISPLAY_FONT, fontweight="medium", ha="center"))
        captions.append(c.text(x, h * 0.45 + 318 * c.k, caption, 26, color=th.muted, ha="center"))
    for step in range(round(TITLE_HOLD * fps)):
        t = step / fps
        card.mosaic(t / 1.8)
        card.fade(title, smooth((t - 0.15) / 0.6), h * 0.45)
        card.fade(sub, smooth((t - 0.4) / 0.6), h * 0.45 + 70 * c.k)
        for k, bar in enumerate(bars):
            bar.set_width(smooth((t - 0.7 - 0.08 * k) / 0.4) * seg)
        count = ease_out((t - 1.0) / 1.2)
        for (value, fmt, _), text, caption in zip(stats, values, captions):
            text.set_text(fmt.format(round(value * count)))
            text.set_alpha(smooth((t - 1.0) / 0.4))
            caption.set_alpha(smooth((t - 1.0) / 0.4))
        yield c.grab()


def outro_content(sim: Simulation) -> tuple[float, str, str, list[tuple[str, str]]]:
    """Headline number, its unit and sentence, and three supporting figures."""
    mgr = sim.failure
    s = sim.monitor.summary(mgr)
    cov = s["coverage"]
    if mgr is not None and mgr.recovery_time is not None:
        r = s["c4_recovery"]
        steady = cov["steady_state_mean_pct"]
        return (
            mgr.recovery_time, "s",
            f"from the failure of UAV {mgr.failed_uav.id} to every cell of route {mgr.route.id} observed again",
            [
                (f"{r['t_handover_s'] - r['t_fail_s']:.0f} s", "Contingency hand-over"),
                (f"{r['bridge_inserts']}", "Bridge insertions"),
                (f"{steady:.1f} %" if steady is not None else "n/a", "Steady-state coverage"),
            ],
        )
    steady = cov["steady_state_mean_pct"]
    value = steady if steady is not None else cov["mean_pct"]
    return (
        value, "%", "steady-state mean coverage" if steady is not None else "mean coverage",
        [
            (f"{sim.params.fleet.size} UAVs", "Fleet"),
            (f"{cov['peak_pct']:.1f} %", "Peak coverage"),
            (f"{cov['time_at_or_above_90pct_s']:.0f} s", "Time at or above 90 %"),
        ],
    )


def outro_frames(sim: Simulation, theme: Theme, layout: Layout, scale: float, fps: float) -> Iterator[Image.Image]:
    """Closing card: the run's headline number counts up, with its supporting figures beneath."""
    value, unit, sentence, extras = outro_content(sim)
    card = Card(layout, theme, scale)
    c, th = card.canvas, theme
    cx, h = c.w / 2, c.h
    number = c.text(cx, h * 0.52, "", 230, fontfamily=DISPLAY_FONT, fontweight="semibold", ha="center", color=th.ink)
    unit_text = c.text(cx, h * 0.52, unit, 80, color=th.muted)
    line = c.text(cx, h * 0.52 + 70 * c.k, sentence, 36, color=th.muted, ha="center")
    stats = []
    for k, (big, small) in enumerate(extras):
        x = cx + (k - 1) * 0.26 * c.w
        stats.append(
            (c.text(x, h * 0.52 + 230 * c.k, big, 50, fontfamily=DISPLAY_FONT, fontweight="medium", ha="center"),
             c.text(x, h * 0.52 + 278 * c.k, small, 24, color=th.muted, ha="center"))
        )
    decimals = 1 if unit == "%" else 0
    for step in range(round(OUTRO_HOLD * fps)):
        t = step / fps
        card.mosaic(t / 1.8)
        shown = f"{value * ease_out(t / 1.4):.{decimals}f}"
        number.set_text(shown)
        # right-hand unit sits after the centred number
        unit_text.set_x(c.fx(cx + c.width_of(number) / 2 + 14 * c.k))
        unit_text.set_alpha(smooth(t / 0.4))
        card.fade(line, smooth((t - 0.5) / 0.5), h * 0.52 + 70 * c.k)
        for big, small in stats:
            big.set_alpha(smooth((t - 1.0) / 0.5))
            small.set_alpha(smooth((t - 1.0) / 0.5))
        yield c.grab()


def build_palette(frames: list[Image.Image], theme: Theme, colours: int) -> Image.Image:
    """Shared palette fitted to a spread sample of frames, with the figure's own colours pinned."""
    transparent = colours - 1
    # median-cut alone merges the failure marker into the overdue amber
    hexes = [
        theme.background, theme.card, theme.border, theme.ink, theme.muted, theme.grid, theme.unseen,
        *theme.ramp, *theme.overdue, theme.failed, theme.contingency, theme.series, theme.series_fill,
        theme.recovery_fill, theme.uav, theme.depot, theme.lost,
    ]
    pinned = [tuple(round(255 * v) for v in to_rgb(h)) for h in hexes]
    mosaic = Image.new("RGB", (frames[0].width, frames[0].height * len(frames)))
    for k, frame in enumerate(frames):
        mosaic.paste(frame, (0, k * frames[0].height))
    n_free = transparent - len(pinned)
    shades = mosaic.quantize(colors=n_free, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
    flat = [v for rgb in pinned for v in rgb] + shades.getpalette()[: 3 * n_free]
    flat += list(pinned[0]) * (256 - len(flat) // 3)  # padding, incl. the transparent index
    palette = Image.new("P", (1, 1))
    palette.putpalette(flat)
    return palette


def quantise(frame: Image.Image, palette: Image.Image, transparent: int) -> np.ndarray:
    """Palette indices of a frame, padding entries mapped to the background."""
    indices = np.asarray(frame.quantize(palette=palette, dither=Image.Dither.NONE)).copy()
    indices[indices >= transparent] = 0  # padding entries are background, like index 0
    return indices


def encode_gif(
    indexed: Iterator[np.ndarray], palette: Image.Image, durations: list[int], out: Path, colours: int = 256
) -> None:
    """GIF of palette-index frames, with unchanged pixels left transparent."""
    transparent = colours - 1
    table = palette.getpalette()[: 3 * colours]

    def deltas() -> Iterator[Image.Image]:
        previous = None  # one frame in flight, so a long video does not sit in memory
        for current in indexed:
            delta = current if previous is None else np.where(current == previous, transparent, current).astype(np.uint8)
            previous = current
            image = Image.fromarray(delta, "P")
            image.putpalette(table)
            image.info["transparency"] = transparent
            yield image

    images = deltas()
    first = next(images)
    out.parent.mkdir(parents=True, exist_ok=True)
    first.save(
        out, save_all=True, append_images=images, duration=durations, loop=0,
        optimize=False, disposal=1, transparency=transparent,
    )


# tagged BT.709 so players do not shift the background tone
H264 = [
    "-c:v", "libx264", "-preset", "medium", "-crf", "18", "-tune", "animation",
    "-colorspace", "bt709", "-color_primaries", "bt709", "-color_trc", "bt709",
    "-movflags", "+faststart",
]


def encode_mp4(
    frames: Iterator[Image.Image],
    out: Path,
    fps: float,
    intro: Iterator[Image.Image] | None = None,
    outro: Callable[[], Iterator[Image.Image]] | None = None,
    hold: float = 1.5,
) -> None:
    """H.264 MP4 streamed through ffmpeg, crossfading from the title card and into the closing card."""
    out.parent.mkdir(parents=True, exist_ok=True)
    first = next(frames)
    command = [
        "ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{first.width}x{first.height}", "-r", str(fps), "-i", "-",
        # yuv420p needs even dimensions, which a fractional --scale does not guarantee
        "-vf", "scale=w=trunc(iw/2)*2:h=trunc(ih/2)*2:out_color_matrix=bt709:out_range=tv,format=yuv420p",
        *H264, "-r", str(fps), str(out),
    ]
    fade = round(FADE * fps)

    def crossfade(a: Image.Image, b: Image.Image) -> Iterator[Image.Image]:
        for k in range(fade):
            yield Image.blend(a, b, (k + 1) / (fade + 1))

    with subprocess.Popen(command, stdin=subprocess.PIPE) as ffmpeg:
        def write(img: Image.Image) -> None:
            ffmpeg.stdin.write(img.tobytes())

        if intro is not None:
            card = None
            for card in intro:
                write(card.resize(first.size) if card.size != first.size else card)
            for img in crossfade(card, first):
                write(img)
        write(first)
        last = first
        for last in frames:
            write(last)
        for _ in range(round(hold * fps)):
            write(last)
        if outro is not None:
            cards = outro()
            card = next(cards)
            for img in crossfade(last, card):
                write(img)
            write(card)
            for card in cards:
                write(card)
        ffmpeg.stdin.close()
    if ffmpeg.returncode:
        sys.exit(f"ffmpeg failed ({ffmpeg.returncode})")


# what the frame workers need; set before the pool forks, so they inherit it for free
_JOB: dict = {}


def _frame(k: int) -> Image.Image:
    snapshots = _JOB["snapshots"]
    return _JOB["renderer"].draw(snapshots[k], final=k == len(snapshots) - 1)


def _frame_indexed(k: int) -> np.ndarray:
    return quantise(_frame(k), _JOB["palette"], _JOB["colours"] - 1)


def map_frames(fn: Callable[[int], object], n: int, workers: int) -> Iterator:
    """``fn(k)`` for k in range(n), in order, drawn across forked worker processes."""
    if workers <= 1 or "fork" not in mp.get_all_start_methods():
        yield from map(fn, range(n))
        return
    window = 2 * workers  # frames are megabytes each, so only a few wait for the encoder
    # recycling workers drops the snapshot pages they copy-on-write as they read
    with mp.get_context("fork").Pool(workers, maxtasksperchild=200) as pool:
        pending: deque = deque()
        for k in range(n):
            pending.append(pool.apply_async(fn, (k,)))
            if len(pending) >= window:
                yield pending.popleft().get()
        while pending:
            yield pending.popleft().get()


def default_window(params, fmt: str) -> tuple[float, float]:
    """Seconds of the run to show: the failure story for a GIF, everything for an MP4."""
    if fmt == "mp4":
        return 0.0, params.mission.duration
    if params.failure.enabled:
        return params.failure.time - 120, params.failure.time + 330
    return 0.0, min(params.mission.duration, 1200.0)


def run_sim(params, start: float, end: float, speed: float, fps: float):
    """Run to ``end`` with a snapshot per frame, the trails warmed up before ``start``."""
    sim = Simulation(replace(params, mission=replace(params.mission, duration=end)))
    t_fail = params.failure.time if params.failure.enabled else None
    schedule = frame_times(start, end, t_fail, speed, fps)
    trails = Trails()
    warm = max(start - TRAIL, 0.0)
    while sim.time < warm:
        sim.step()
    while sim.time < start:  # discarded: only fills the trails
        for _ in range(2):
            sim.step()
        capture(sim, 0.0, trails)
    snapshots = []
    for time, step, slow in schedule:
        while sim.time < time:
            sim.step()
        snapshots.append(capture(sim, step, trails, slow))
    return sim, snapshots


def render(
    name: str, start: float | None, end: float | None, speed: float | None, fmt: str, out: Path,
    fps: float, scale: float, titles: bool, workers: int, theme: Theme, layout: Layout, colours: int,
) -> None:
    """One scenario simulated, replayed frame by frame and written as a GIF or MP4."""
    params = SCENARIOS[name]
    d_start, d_end = default_window(params, fmt)
    start = d_start if start is None else start
    end = d_end if end is None else end
    sim, snapshots = run_sim(params, start, end, speed or SPEED[fmt], fps)

    renderer = Renderer(sim, end, SCENARIO_LABELS[name], fps, scale, theme, layout, banded=fmt == "gif")
    n = len(snapshots)
    _JOB.update(renderer=renderer, snapshots=snapshots, palette=None, colours=colours)
    if fmt == "gif":
        spread = sorted({*range(0, n, max(n // 16, 1)), n - 1})
        palette = build_palette([_frame(k) for k in spread], theme, colours)
        _JOB["palette"] = palette
        durations = [round(100 / fps) * 10] * n
        durations[-1] = 3500  # hold on the summary before the loop restarts
        encode_gif(map_frames(_frame_indexed, n, workers), palette, durations, out, colours)
    else:
        intro = outro = None
        if titles:
            intro = title_frames(params, SCENARIO_LABELS[name], theme, layout, scale, fps)
            outro = lambda: outro_frames(sim, theme, layout, scale, fps)  # noqa: E731
        encode_mp4(map_frames(_frame, n, workers), out, fps, intro, outro)
    width, height = renderer.canvas.fig.canvas.get_width_height()
    print(f"{name}: {n} frames, {width}x{height}, {out.stat().st_size / 1e6:.1f} MB -> {out}")
    plt.close(renderer.canvas.fig)


def compare(
    names: list[str], out: Path, end: float | None, speed: float | None, fps: float, scale: float,
    workers: int, theme: Theme,
) -> None:
    """One MP4 with the scenarios side by side, for judging fleet sizes at a glance."""
    end = end or SCENARIOS[names[0]].mission.duration
    sims, series = [], []
    for name in names:
        sim, snaps = run_sim(SCENARIOS[name], 0.0, end, speed or SPEED["mp4"], fps)
        sims.append(sim)
        series.append(snaps)
    renderer = CompareRenderer(sims, [SCENARIO_LABELS[nm] for nm in names], end, fps, scale, theme)
    snapshots = list(zip(*series))
    _JOB.update(renderer=renderer, snapshots=snapshots)
    encode_mp4(map_frames(_frame, len(snapshots), workers), out, fps, hold=3.0)
    print(f"compare: {' | '.join(names)}, {out.stat().st_size / 1e6:.1f} MB -> {out}")
    plt.close(renderer.canvas.fig)


def still(name: str, time: float, out: Path, scale: float, theme: Theme, layout: Layout, fmt: str) -> None:
    """One frame of a scenario at ``time`` as a PNG, for design review."""
    params = SCENARIOS[name]
    sim = Simulation(replace(params, mission=replace(params.mission, duration=time)))
    trails = Trails()
    warm = max(time - TRAIL, 0.0)
    while sim.time < warm:
        sim.step()
    while sim.time < time:
        sim.step()
        if sim.time % 2 == 0:
            capture(sim, 0.0, trails)
    t_fail = params.failure.time if params.failure.enabled else None
    snap = capture(sim, SPEED[fmt] / 30, trails, slowness(time, t_fail) > 0.5)
    renderer = Renderer(sim, params.mission.duration, SCENARIO_LABELS[name], 30, scale, theme, layout, banded=fmt == "gif")
    out.parent.mkdir(parents=True, exist_ok=True)
    renderer.draw(snap).save(out)
    print(f"{name} at t = {time:.0f} s -> {out}")


def out_path(name: str, fmt: str, layout: str) -> Path:
    """Default output file of a scenario."""
    if fmt == "gif" and name == "single_failure" and layout == "hero":
        return Path("assets/demo.gif")
    return Path("assets/videos") / f"{name}.{fmt}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", default="single_failure", choices=[*SCENARIOS, "all"])
    parser.add_argument("--format", default="gif", choices=["gif", "mp4"])
    parser.add_argument("--layout", choices=list(LAYOUTS), help="canvas (default: hero for a GIF, landscape for an MP4)")
    parser.add_argument("--out", type=Path, help="output file (default under assets/)")
    parser.add_argument("--start", type=float, help="first simulated second shown")
    parser.add_argument("--end", type=float, help="last simulated second shown")
    parser.add_argument("--speed", type=float, help="x real time (default 40 for a GIF, 20 for an MP4)")
    parser.add_argument("--fps", type=float, help="frames per second (default 20 for a GIF, 60 for an MP4)")
    parser.add_argument("--scale", type=float, default=1.0, help="output size relative to the layout, e.g. 2 for double")
    parser.add_argument("--colors", type=int, default=128, help="GIF palette size")
    parser.add_argument("--jobs", type=int, default=os.cpu_count() or 1, help="worker processes drawing frames")
    parser.add_argument("--no-title", action="store_true", help="leave the title and closing cards off MP4s")
    parser.add_argument("--still", type=float, metavar="TIME", help="write one PNG of this simulated second and stop")
    parser.add_argument("--compare", nargs="+", choices=list(SCENARIOS), metavar="SCENARIO",
                        help="side-by-side MP4 of these scenarios (default assets/videos/compare.mp4)")
    args = parser.parse_args()

    if args.compare:
        compare(args.compare, args.out or Path("assets/videos/compare.mp4"), args.end, args.speed,
                args.fps or DEFAULT_FPS["mp4"], args.scale, args.jobs, LIGHT)
        return
    names = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    if len(names) > 1 and args.out:
        parser.error("--out names one file; use --scenario NAME or drop --out")
    layout_name = args.layout or ("hero" if args.format == "gif" else "landscape")
    if args.still is not None:
        still(names[0], args.still, args.out or Path("frame.png"), args.scale, LIGHT,
              LAYOUTS[layout_name], args.format)
        return
    for name in names:
        render(
            name, args.start, args.end, args.speed, args.format,
            args.out or out_path(name, args.format, layout_name),
            args.fps or DEFAULT_FPS[args.format], args.scale, not args.no_title, args.jobs,
            LIGHT, LAYOUTS[layout_name], args.colors,
        )


if __name__ == "__main__":
    main()
