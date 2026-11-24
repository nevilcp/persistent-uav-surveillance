"""Patrol videos and the README GIF, drawn with skia-python from export_frames.py output."""

import argparse
import json
import math
import re
import subprocess
import time
from bisect import bisect_right
from collections import deque
from functools import lru_cache
from itertools import pairwise
from pathlib import Path

import numpy as np
import skia

W = 1200
PAD = 14  # card inset from the canvas edge
PLOT = (66, 62, 1010)  # left, top, width of the metric plot
XLIM, YLIM = (-550.0, 550.0), (-50.0, 550.0)
SCALE = PLOT[2] / (XLIM[1] - XLIM[0])
PLOT_H = (YLIM[1] - YLIM[0]) * SCALE
MONITOR_W = 1180
VIDEO_W = W + MONITOR_W
AXIS_GAP = 9
SNAP = 25.0  # metres; a larger jump between ticks is a relaunch or swap, not flight
INK, MUTED, RULE = "#14181f", "#465063", "#d6dce4"
BG, CARD = "#f4f5f7", "#ffffff"
PLOT_BG, GRID = "#fafbfc", "#e1e6ec"
FAIL = "#9a3412"
ZONE = "#1E1B3A"
ZONE_LINE = "#C4B5FD"
ZONE_BORDER = "#A78BFA"
DEPOT = "#e5383b"
PALETTES = {  # red -> green as RdYlGn; stops at 0, .25, .5, .75, 1
    "A": ["#b3262d", "#ea6a3c", "#f3d263", "#86c06c", "#1c7c4d"],
    "B": ["#951c2e", "#d0502c", "#dfa52b", "#5b9f55", "#0f6a3d"],
    "C": ["#7f1527", "#be4528", "#cf9a1e", "#4f9150", "#0b5c35"],
    "D": ["#90162b", "#d64823", "#e8b02d", "#63a749", "#0a6539"],
}
RAMP = [(i / 4, c) for i, c in enumerate(PALETTES["D"])]

# type scale: one family, five sizes; weight carries the hierarchy
TYPE = {
    "title": (26, 600, "Inter Display"),
    "meta": (15, 400, "Inter"),
    "value": (15, 600, "Inter"),
    "heading": (15, 700, "Inter"),
    "label": (14, 500, "Inter"),
    "small": (13, 500, "Inter"),
    "tick": (12, 400, "Inter"),
}


def hexrgb(h):
    h = h.lstrip("#")
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


def color(h, a=255):
    r, g, b = hexrgb(h)
    return skia.Color(r, g, b, a)


def ramp_lut(n=256):
    out = []
    for i in range(n):
        v = i / (n - 1)
        for (a, ca), (b, cb) in pairwise(RAMP):
            if v <= b:
                f = (v - a) / (b - a)
                r, g, bl = (round(x + (y - x) * f) for x, y in zip(hexrgb(ca), hexrgb(cb)))
                out.append(skia.Color(r, g, bl))
                break
    return out


LUT = ramp_lut()


def set_palette(name):
    global RAMP, LUT
    RAMP = [(i / 4, c) for i, c in enumerate(PALETTES[name])]
    LUT = ramp_lut()


LEVELS = None  # set for the gif: a stepped ramp keeps the 256-colour palette free of dither


def ramp(v):
    if LEVELS:
        v = round(v * (LEVELS - 1)) / (LEVELS - 1)
    return LUT[min(max(round(v * 255), 0), 255)]


class Fonts:
    def __init__(self):
        self.mgr = skia.FontMgr()
        self.cache = {}

    def get(self, style):
        if style not in self.cache:
            size, weight, family = TYPE[style]
            face = self.mgr.matchFamilyStyle(family, skia.FontStyle(weight, skia.FontStyle.kNormal_Width, skia.FontStyle.kUpright_Slant))
            f = skia.Font(face, size)
            f.setSubpixel(True)
            f.setEdging(skia.Font.Edging.kSubpixelAntiAlias)
            self.cache[style] = f
        return self.cache[style]


FONTS = Fonts()


def fill(c, alpha=255):
    return skia.Paint(AntiAlias=True, Color=color(c, alpha) if isinstance(c, str) else c)


def stroke(c, w, alpha=255):
    p = fill(c, alpha)
    p.setStyle(skia.Paint.kStroke_Style)
    p.setStrokeWidth(w)
    p.setStrokeJoin(skia.Paint.kMiter_Join)
    return p


def advances(f, s, tab):
    """Per-glyph advances; tabular digits are as wide as a zero so changing numbers do not shimmer."""
    zero = f.measureText("0")
    return [zero if tab and ch.isdigit() else f.measureText(ch) for ch in s]


@lru_cache(maxsize=4096)
def style_advances(style, s, tab):
    return tuple(advances(FONTS.get(style), s, tab))


@lru_cache(maxsize=4096)
def measure(style, s, tab=False):
    f = FONTS.get(style)
    return sum(style_advances(style, s, True)) if tab else f.measureText(s)


def text(cv, s, x, y, style, c=INK, align="left", tab=False):
    f = FONTS.get(style)
    w = measure(style, s, tab)
    x -= w if align == "right" else w / 2 if align == "center" else 0
    if not tab:
        cv.drawString(s, x, y, f, fill(c))
        return w
    for ch, adv in zip(s, style_advances(style, s, True)):
        gx = x + (adv - measure(style, ch)) / 2 if ch.isdigit() else x
        cv.drawString(ch, gx, y, f, fill(c))
        x += adv
    return w


def star(cx, cy, r):
    path = skia.Path()
    for i in range(10):
        a = -math.pi / 2 + i * math.pi / 5
        rad = r if i % 2 == 0 else r * 0.45
        (path.moveTo if i == 0 else path.lineTo)(cx + rad * math.cos(a), cy + rad * math.sin(a))
    path.close()
    return path


def triangle(cx, cy, r):
    path = skia.Path()
    path.moveTo(cx, cy - r)
    path.lineTo(cx + r * 0.9, cy + r * 0.75)
    path.lineTo(cx - r * 0.9, cy + r * 0.75)
    path.close()
    return path


def px(x, y):
    return PLOT[0] + (x - XLIM[0]) * SCALE, PLOT[1] + (YLIM[1] - y) * SCALE


# Normalize titles in older frame exports while preserving casing in custom titles.
LEGACY_SCENARIO_TITLES = {
    "Continuous Patrol": "Continuous patrol",
    "Port Surveillance": "Port surveillance",
    "Single Depot: West": "Single depot: west",
    "Single Depot: East": "Single depot: east",
    "Single Depot: North": "Single depot: north",
    "No-Fly Zones": "No-fly zones",
    "Single Failure": "Single failure",
    "Repeated Failures": "Repeated failures",
    "Depot Closure": "Depot closure",
    "Return Metric Control": "Return metric control",
    "Multiple Depots": "Multiple depots",
}


class Run:
    def __init__(self, path, *, compact_baseline=False):
        d = json.loads(Path(path).read_text())
        self.d = d
        fr = d["frames"]
        self.n = len(fr)
        self.t0 = fr[0]["t"]
        self.fresh = np.array([f["fresh"] for f in fr], dtype=np.float32)
        self.ids = [u[0] for u in fr[0]["uavs"]]
        self.uav = np.array([[u[1:] for u in f["uavs"]] for f in fr], dtype=np.float32)  # tick, uav, (x y soc)
        self.fr = fr
        routes = max(frame.get("routes_due") or 0 for frame in fr)
        self.hud_value_width = max(
            measure("value", value, tab=True)
            for value in (format_hud_time(fr[-1]["t"]), f"{routes:,} / {routes:,}", f"{len(self.ids):,}",
                          "100.0%", "None due")
        )
        self.failure = d["failure"]
        self.events = d.get("events", [])
        if not self.events and self.failure:
            self.events = [{"kind": "failure", "t": self.failure["t_fail"], "uav": self.failure["uav"]}]
        self.event_labels = []
        failure_number = 0
        for event in sorted(self.events, key=lambda event: event["t"]):
            if event["kind"] == "failure":
                failure_number += 1
                label = f"Failure {failure_number}"
            elif event["kind"] == "closure":
                label = f"D{int(event['detail'].split()[-1]) + 1} closure"
            else:
                continue
            self.event_labels.append((event["t"], label))
        self.failure_times = sorted(event["t"] for event in self.events if event["kind"] == "failure")
        self.cells = d["cells"]
        self.depot = d["depot"]
        self.depots = d.get("depots", [self.depot])
        self.extended = "extent" in d
        self.compact = compact_baseline and not self.extended
        global XLIM, YLIM, SCALE, PLOT_H
        XLIM, YLIM = (-550.0, 550.0), (-50.0, 550.0)
        if not self.compact:
            cx, cy = (600, 850) if self.extended else (0, 250)
            XLIM, YLIM = (cx - 1135, cx + 1135), (cy - 895, cy + 895)
        SCALE = PLOT[2] / (XLIM[1] - XLIM[0])
        PLOT_H = (YLIM[1] - YLIM[0]) * SCALE
        title = " ".join(d["title"].split())
        self.title = LEGACY_SCENARIO_TITLES.get(title, title)
        study_id = re.match(r"R\d+(?=\b|_)", d.get("name") or d.get("meta", ""), re.IGNORECASE)
        self.study_id = study_id.group().upper() if study_id else ""
        self.zone_path = skia.Path()
        for x0, y0, x1, y1 in d.get("zones", []):
            rect = skia.Path()
            rect.addRect(skia.Rect.MakeLTRB(*px(x0, y1), *px(x1, y0)))
            self.zone_path = skia.Op(self.zone_path, rect, skia.PathOp.kUnion_PathOp)
        strip = skia.Path()
        stroke(ZONE_BORDER, 3).getFillPath(self.zone_path, strip)
        self.zone_border = skia.Op(self.zone_path, strip, skia.PathOp.kDifference_PathOp)

        half = d["cell_size"] / 2
        self.rects = [
            skia.Rect.MakeLTRB(*px(x - half, y + half), *px(x + half, y - half)).makeInset(1.3, 1.3) for x, y in self.cells
        ]
        self.tile_paints = [fill(c) for c in LUT]
        self.failure_depots = d.get("failure_depots", {})

    def at(self, pos):
        """State interpolated at tick position ``pos``."""
        pos = min(max(pos, 0.0), self.n - 1.0)
        if self.n == 1:
            return 0, self.fresh[0], self.uav[0]
        i = min(int(pos), self.n - 2)
        f = pos - i
        fresh = self.fresh[i] + (self.fresh[i + 1] - self.fresh[i]) * f
        a, b = self.uav[i], self.uav[i + 1]
        jump = np.hypot(b[:, 0] - a[:, 0], b[:, 1] - a[:, 1]) > SNAP
        u = a + (b - a) * f
        u[jump] = a[jump]
        return min(int(pos), self.n - 1), fresh, u


def canvas_h(run):
    return round(PLOT[1] + PLOT_H + 48 + PAD)


def draw_axes(cv, run):
    left, top, w = PLOT
    cv.drawRect(skia.Rect.MakeXYWH(left, top, w, PLOT_H), fill(PLOT_BG))
    step = 200 if run.extended else 100
    xs = range(math.ceil(XLIM[0] / step) * step, math.floor(XLIM[1] / step) * step + 1, step) if run.extended else range(-500, 501, 100)
    ys = range(math.ceil(YLIM[0] / step) * step, math.floor(YLIM[1] / step) * step + 1, step) if run.extended else range(0, 501, 100)
    for v in xs:
        x = px(v, 0)[0]
        cv.drawLine(x, top, x, top + PLOT_H, stroke(GRID, 1))
        text(cv, f"{v}".replace("-", "−"), x, top + PLOT_H + AXIS_GAP + 9, "tick", MUTED, "center", tab=True)
    y_tick_width = 0
    for v in ys:
        y = px(0, v)[1]
        cv.drawLine(left, y, left + w, y, stroke(GRID, 1))
        width = text(cv, f"{v}".replace("-", "−"), left - AXIS_GAP, y + 4, "tick", MUTED, "right", tab=True)
        y_tick_width = max(y_tick_width, width)
    text(cv, "X (m)", left + w / 2, top + PLOT_H + 2 * (AXIS_GAP + 9), "tick", MUTED, "center")
    cv.save()
    cv.translate(left - 2 * AXIS_GAP - y_tick_width, top + PLOT_H / 2)
    cv.rotate(-90)
    text(cv, "Y (m)", 0, 0, "tick", MUTED, "center")
    cv.restore()


def marker(cv, x, y):
    path = triangle(x, y, 5.2)
    cv.drawPath(path, fill(CARD))
    cv.drawPath(path, stroke(INK, 1.25))


def chip_box(cv, x, y, w, h, alpha):
    rect = skia.Rect.MakeXYWH(x, y, w, h)
    cv.drawRect(rect, fill(CARD, alpha))
    cv.drawRect(rect, stroke("#b8c1cd", 1))


def format_hud_time(t):
    return f"{int(t)} s"


def draw_hud(cv, run, st, *, compact):
    """Mission state in the current sample, independent of the monitor readouts."""
    now = st["t"]
    unstaffed, due = st.get("routes_unstaffed"), st.get("routes_due")
    if due:
        gaps = f"{unstaffed:,} / {due:,}" if unstaffed is not None else "—"
    else:
        gaps = "None due" if due == 0 else "—"
    failed = st.get("failed")
    if failed is None:
        failed = len(st["failed_uavs"]) if "failed_uavs" in st else bisect_right(run.failure_times, now)

    route_status = ("Unserved", gaps, FAIL if unstaffed else INK)
    ready_spares = ("Reserves", f"{st['spare']:,}", INK)
    failed_uavs = ("Losses", f"{failed:,}", INK)
    if compact:
        metrics = [route_status, ready_spares, failed_uavs, ("Coverage", f"{st['cov']:.1f}%", INK)]
    else:
        metrics = [route_status, ("Patrolling", f"{st['active']:,}", INK), ready_spares, failed_uavs]

    metrics.insert(0, ("Time", format_hud_time(now), INK))
    x, y = PLOT[0] + 12, PLOT[1] + 12
    pad, row_height = 12, 22
    label_width = max(measure("label", label + ": ") for label, _, _ in metrics)
    width = label_width + run.hud_value_width + 2 * pad
    box = skia.Rect.MakeXYWH(x, y, width, len(metrics) * row_height + 2 * pad)
    cv.drawRoundRect(box, 4, 4, fill(CARD))
    cv.drawRoundRect(box, 4, 4, stroke(RULE, 1))
    for row, (label, value, ink) in enumerate(metrics):
        baseline = y + pad + 15 + row * row_height
        text(cv, label + ": ", x + pad, baseline, "label", MUTED)
        text(cv, value, box.right() - pad, baseline, "value", ink, "right", tab=True)


def draw_legend(cv, zones=False):
    x1 = PLOT[0] + PLOT[2] - 12
    y = PLOT[1] + 12
    uav_label_width = measure("label", "UAVs")  # Preserve the existing legend spacing.
    w = 24 + measure("label", "Depot") + 26 + 26 + uav_label_width + 14
    if zones:
        w += 38 + measure("label", "No-fly zone")
    chip_box(cv, x1 - w, y, w, 32, 246)
    x = x1 - w + 14
    cv.drawPath(star(x + 7, y + 16, 8), fill(DEPOT))
    x += 22
    x += text(cv, "Depot", x, y + 21, "label", INK) + 22
    marker(cv, x + 7, y + 16.5)
    text(cv, "UAV", x + 22, y + 21, "label", INK)
    if zones:
        x += 22 + uav_label_width + 18
        box = skia.Rect.MakeXYWH(x, y + 10, 13, 13)
        cv.drawRect(box, fill(ZONE, 224))
        cv.save()
        cv.clipRect(box)
        for offset in (-10, 4, 18):
            cv.drawLine(x + offset, y + 23, x + offset + 13, y + 10, stroke(ZONE_LINE, 1.5, 89))
        cv.restore()
        outline = stroke(ZONE_BORDER, 1)
        outline.setPathEffect(skia.DashPathEffect.Make([3, 2], 0))
        cv.drawRect(box.makeInset(1, 1), outline)
        text(cv, "No-fly zone", x + 20, y + 21, "label", INK)


def draw_colorbar(cv):
    x = PLOT[0] + PLOT[2] + 20
    y, h = PLOT[1], PLOT_H
    bar = skia.Rect.MakeXYWH(x, y, 14, h)
    if LEVELS:
        cv.save()
        cv.clipRect(bar)
        for k in range(LEVELS):
            cv.drawRect(skia.Rect.MakeXYWH(x, y + h * (1 - (k + 1) / LEVELS) - 0.5, 14, h / LEVELS + 1), fill(ramp(k / (LEVELS - 1))))
        cv.restore()
    else:
        shader = skia.GradientShader.MakeLinear(
            [(0, y), (0, y + h)], [skia.Color(*hexrgb(c), 255) for _, c in reversed(RAMP)], [1 - p for p, _ in reversed(RAMP)]
        )
        cv.drawRect(bar, skia.Paint(AntiAlias=True, Shader=shader))
    for k in range(6):
        v = k / 5
        text(cv, f"{v:.0%}", x + 24, y + h * (1 - v) + 4, "tick", MUTED, tab=True)
    cv.save()
    cv.translate(x + 70, y + h / 2)
    cv.rotate(-90)
    text(cv, "Cell freshness", 0, 0, "small", MUTED, "center")
    cv.restore()


def nice_step(span, divisions, minimum=0):
    target = max(span / divisions, minimum)
    order = 10 ** math.floor(math.log10(target))
    return next(order * n for n in (1, 2, 2.5, 5, 10) if order * n >= target)


def axis_ticks(values, *, integer=False):
    low, high = min(0, min(values, default=0)), max(0, max(values, default=0))
    if low == high:
        return (0, 1)
    step = nice_step(high - low, 4, minimum=1 if integer else 0)
    if integer:
        step = math.ceil(step)
    return tuple(i * step for i in range(math.floor(low / step), math.ceil(high / step) + 1))


def rolling_average(values, window):
    recent = deque()
    total = count = 0
    for value in values:
        if len(recent) == window:
            oldest = recent.popleft()
            if oldest is not None:
                total -= oldest
                count -= 1
        recent.append(value)
        if value is None:
            yield None
        else:
            total += value
            count += 1
            average = total / count
            yield 0 if abs(average) < 1e-10 else average


class MonitorTrack:
    def __init__(self, heading, unit, traces, fields, *, ticks=None, integer=False, breaks=()):
        self.heading = heading
        self.unit = unit
        self.traces = traces
        self.fields = fields
        self.breaks = frozenset(breaks)
        self.ticks = ticks or axis_ticks(
            [value for _, series in traces for value in series if value is not None], integer=integer
        )
        # Reserve each field's width over the complete mission, including both
        # extremes of signed margins and any wider standby status.
        self.field_layout = [
            (label + ":", measure("small", label + ": "),
             max(measure("small", value, True) for value in values))
            for label, values in fields
        ]

    def format_tick(self, value):
        return f"{value:g}".replace("-", "−")


def monitor_tracks(run):
    fr = run.fr
    limits = run.d.get("monitor_limits", {})
    deadline = limits.get("recovery")
    window = run.d["rolling_window"]
    sample_step = (fr[-1]["t"] - fr[0]["t"]) / (len(fr) - 1) if len(fr) > 1 else window
    window_samples = max(1, round(window / sample_step))
    rolling_label = f"Rolling ({window} s)"

    def read(key):
        return [f.get(key) for f in fr]

    def constant(value):
        return [value] * len(fr)

    def seconds(value):
        return "—" if value is None else f"{value:g} s"

    def gauge(heading, key, unit, factor=1, integer=False):
        values = [None if value is None else value * factor for value in read(key)]
        margin_format = "+,.0f" if integer else "+,.1f"
        violations = 0
        counts, margins = [], []
        for i, value in enumerate(values):
            violations += value is not None and value < 0
            counts.append(f"{fr[i].get('c2', violations) if key == 'battery_margin' else violations:,}")
            margins.append("—" if value is None else f"{value:{margin_format}}{unit}")
        if key != "battery_margin" and all(value is None for value in values):
            counts = constant("—")
        return MonitorTrack(heading, unit,
                            [("Instant", values), (rolling_label, list(rolling_average(values, window_samples)))],
                            [("Margin", margins), ("Violations", counts)], integer=integer)

    recoveries = run.d.get("recoveries")
    if recoveries is None:
        recoveries = []
        for event in run.events:
            if event["kind"] == "failure":
                failure = run.failure or {}
                restored = failure.get("t_restored") if event["t"] == failure.get("t_fail") else None
                recoveries.append({"t_fail_s": event["t"], "satisfied": restored is not None,
                                   "recovery_time_s": None if restored is None else restored - event["t"]})
    recovery_values, failure_counts, recovery_readouts, misses = [], [], [], []
    started = prior_misses = 0
    for frame in fr:
        now = frame["t"]
        while started < len(recoveries) and recoveries[started]["t_fail_s"] <= now:
            if started:
                prior_misses += not recoveries[started - 1]["satisfied"]
            started += 1
        value, elapsed, missed = None, None, False
        if started:
            recovery = recoveries[started - 1]
            elapsed = now - recovery["t_fail_s"]
            duration = recovery["recovery_time_s"]
            restored = duration is not None and elapsed >= duration
            missed = deadline is not None and elapsed >= deadline and (not restored or duration > deadline)
            if restored:
                value = duration
            elif not missed:
                value = elapsed
        recovery_values.append(value)
        failure_counts.append(f"{started:,} / {len(recoveries):,}")
        recovery_readouts.append(seconds(value if value is not None else elapsed))
        misses.append(f"{prior_misses + missed:,}")

    coverage = read("cov")
    return [
        MonitorTrack("C1: Cell freshness", "%", [("Instant", coverage), (rolling_label, read("roll"))],
                     [("Fresh cells", [f"{value:.1f}%" for value in coverage]),
                      ("Overdue cells", [f"{round(len(run.cells) * (1 - value / 100)):,}" for value in coverage]),
                      ("Age limit", constant(seconds(limits.get("freshness"))))], ticks=tuple(range(0, 101, 20))),
        gauge("C2: Battery reserve margin", "battery_margin", " pp", 100),
        MonitorTrack("C3: Spare launch", " Alarms",
                     [("Late alarms", read("c3_missed")), ("All alarms", read("c3_alarms"))],
                     [("Late", [f"{value:,}" for value in read("c3_missed")]),
                      ("Alarms", [f"{value:,}" for value in read("c3_alarms")]),
                      ("Deadline", constant(seconds(limits.get("launch"))))], integer=True),
        MonitorTrack("C4: Coverage recovery", " s",
                     [("Elapsed", recovery_values), ("Deadline", constant(deadline))],
                     [("Failures", failure_counts), ("Elapsed", recovery_readouts), ("Deadline", constant(seconds(deadline))),
                      ("Missed", misses)], breaks=[entry["t_fail_s"] for entry in recoveries]),
        gauge("C5: Return energy margin", "way_home_margin", " pp", 100),
        gauge("C6: Depot queue margin", "queue_margin", " UAVs", integer=True),
        gauge("C7: No-fly clearance margin", "zone_clearance", " m"),
    ]


class MonitorPanel:
    """Seven synchronized STL timelines with fixed axes and readout positions."""

    PALETTE = ("#146d45", "#245ba3", "#6b4194", "#a64c13", "#086b79", "#9b2e44", "#786000")

    def __init__(self, run, height):
        self.tracks = monitor_tracks(run)
        self.times = [frame["t"] for frame in run.fr]
        self.start, self.end = self.times[0], self.times[-1]
        self.height = height
        self.left, self.right = 95, MONITOR_W - 35
        self.row_step = (height - 80) / len(self.tracks)
        self.plot_height = self.row_step - 32
        self.playhead = stroke(INK, 1.2, 175)
        self.events = [(t, label) for t, label in run.event_labels if self.start <= t <= self.end]

    def x(self, time):
        return self.left + (self.right - self.left) * (time - self.start) / (self.end - self.start or 1)

    def time_ticks(self):
        if self.start == self.end:
            return [self.start]
        step = nice_step(self.end - self.start, 5, minimum=1)
        ticks = [i * step for i in range(math.ceil(self.start / step), math.floor(self.end / step) + 1)]
        return [self.start, *(t for t in ticks if self.start + step / 5 < t < self.end - step / 5), self.end]

    def area(self, row):
        return skia.Rect.MakeLTRB(self.left, 33 + row * self.row_step,
                                  self.right, 33 + row * self.row_step + self.plot_height)

    def y(self, area, track, value):
        low, high = track.ticks[0], track.ticks[-1]
        return area.top() + 4 + (self.plot_height - 8) * (high - value) / (high - low)

    @lru_cache(maxsize=4)
    def background(self, sx, sy):
        surface = skia.Surface(round(MONITOR_W * sx), round(self.height * sy))
        cv = surface.getCanvas()
        cv.clear(color(CARD))
        cv.scale(sx, sy)
        cv.drawLine(0, 0, 0, self.height, stroke(RULE, 2))
        grid = stroke(GRID, 1)
        events = stroke(FAIL, 1, 175)
        events.setPathEffect(skia.DashPathEffect.Make([4, 3], 0))
        ticks = self.time_ticks()
        for row, (track, hue) in enumerate(zip(self.tracks, self.PALETTE)):
            area = self.area(row)
            top, bottom = area.top(), area.bottom()
            heading_y = top - 11
            cv.drawRect(area, fill(PLOT_BG))
            for t in ticks:
                x = self.x(t)
                cv.drawLine(x, top, x, bottom, grid)
            for tick in track.ticks:
                y = self.y(area, track, tick)
                cv.drawLine(self.left, y, self.right, y, grid)
                text(cv, track.format_tick(tick), self.left - AXIS_GAP, y + 4, "tick", MUTED, "right", tab=True)
            cv.save()
            cv.translate(self.left - 56, (top + bottom) / 2)
            cv.rotate(-90)
            text(cv, track.unit.strip(), 0, 0, "tick", MUTED, "center")
            cv.restore()
            cv.drawRect(area, stroke(RULE, 1))
            text(cv, track.heading, self.left, heading_y, "heading")

            for t, _ in self.events:
                x = self.x(t)
                cv.drawLine(x, top, x, bottom, events)
            legend_right = self.right
            for j in reversed(range(len(track.traces))):
                label, values = track.traces[j]
                paint = stroke(hue, 1.65, 175 if j else 255)
                if j:
                    paint.setPathEffect(skia.DashPathEffect.Make([5, 3], 0))
                width = text(cv, label, legend_right, heading_y, "small", MUTED, "right")
                line_end = legend_right - width - 7
                cv.drawLine(line_end - 20, heading_y - 4, line_end, heading_y - 4, paint)
                legend_right = line_end - 36
                path = skia.Path()
                drawing = False
                for t, value in zip(self.times, values):
                    if value is None:
                        drawing = False
                        continue
                    point = self.x(t), self.y(area, track, value)
                    if drawing and (j or t not in track.breaks):
                        path.lineTo(*point)
                    else:
                        path.moveTo(*point)
                    drawing = True
                cv.save()
                cv.clipRect(area)
                cv.drawPath(path, paint)
                cv.restore()
            if all(value is None for _, values in track.traces for value in values):
                text(cv, "No samples", (self.left + self.right) / 2, (top + bottom) / 2 + 4,
                     "small", MUTED, "center")

        bottom = self.area(len(self.tracks) - 1).bottom()
        for i, t in enumerate(ticks):
            label = "0" if i == 0 and t == 1 else f"{t:.0f}".replace("-", "−")
            text(cv, label, self.x(t), bottom + 17, "tick", MUTED, "center", tab=True)
        lane_ends = [self.left, self.left]
        for t, label in self.events:
            width = measure("tick", label)
            x = min(self.x(t) + 4, self.right - width)
            for lane, end in enumerate(lane_ends):
                if x >= end:
                    text(cv, label, x, bottom + 35 + lane * 14, "tick", FAIL)
                    lane_ends[lane] = x + width + 8
                    break
        text(cv, "Time (s)", (self.left + self.right) / 2, bottom + 65, "small", MUTED, "center")
        return surface.makeImageSnapshot()

    def draw(self, cv, index):
        matrix = cv.getTotalMatrix()
        image = self.background(matrix.getScaleX(), matrix.getScaleY())
        cv.drawImageRect(image, skia.Rect.MakeWH(MONITOR_W, self.height))
        cursor = self.x(self.times[index])
        for row, (track, hue) in enumerate(zip(self.tracks, self.PALETTE)):
            area = self.area(row)
            y = area.top() - 11
            x = self.left + 282
            for (label, label_width, value_width), (_, values) in zip(track.field_layout, track.fields):
                text(cv, label + " ", x, y, "small", MUTED)
                text(cv, values[index], x + label_width, y, "small", INK, tab=True)
                x += label_width + value_width + 18
            cv.save()
            cv.clipRect(area)
            cv.drawLine(cursor, area.top(), cursor, area.bottom(), self.playhead)
            for j, (_, values) in enumerate(track.traces):
                value = values[index]
                if value is not None:
                    y = self.y(area, track, value)
                    cv.drawCircle(cursor, y, 3, fill(hue) if j == 0 else fill(CARD))
                    cv.drawCircle(cursor, y, 3, stroke(CARD if j == 0 else hue, 1))
            cv.restore()


@lru_cache(maxsize=4)
def compact_backdrop(h, sx, sy):
    surface = skia.Surface(round(W * sx), round(h * sy))
    cv = surface.getCanvas()
    cv.clear(color(BG))
    cv.scale(sx, sy)
    outer = skia.Rect.MakeXYWH(PAD / 2, PAD / 2, W - PAD, h - PAD)
    shadow = fill("#101828", 10)
    shadow.setMaskFilter(skia.MaskFilter.MakeBlur(skia.kNormal_BlurStyle, 5))
    cv.drawRect(outer.makeOffset(0, 2), shadow)
    cv.drawRect(outer, fill(CARD))
    cv.drawRect(outer, stroke(RULE, 1))
    return surface.makeImageSnapshot()


def failure_crosses(depot, count):
    """Fill a two-column grid beside the depot, preserving earlier positions."""
    x, y = px(*depot)
    left, top, width = PLOT
    right, bottom = left + width, top + PLOT_H
    side = -1 if x - left > right - x else 1
    direction = -1 if bottom - y < 40 else 1
    return [(x + side * (22 + 19 * (slot % 2)), y + direction * 19 * (slot // 2))
            for slot in range(count)]


def draw_frame(cv, run, pos, h, *, compact_hud=False):
    i, fresh, uav = run.at(pos)
    st = run.fr[i]
    if run.compact:
        matrix = cv.getTotalMatrix()
        image = compact_backdrop(h, matrix.getScaleX(), matrix.getScaleY())
        cv.drawImageRect(image, skia.Rect.MakeWH(W, h))
    else:
        cv.clear(color(CARD))

    tw = text(cv, run.title, PLOT[0], 42, "title", INK)
    if run.study_id:
        text(cv, run.study_id, PLOT[0] + tw + 16, 42, "meta", MUTED)

    draw_axes(cv, run)
    levels = fresh.astype(float)
    if LEVELS:
        levels = np.rint(levels * (LEVELS - 1)) / (LEVELS - 1)
    indices = np.rint(levels * 255).astype(np.intp).clip(0, 255)
    for rect, index in zip(run.rects, indices):
        cv.drawRect(rect, run.tile_paints[index])

    if not run.zone_path.isEmpty():
        cv.drawPath(run.zone_path, fill(ZONE, 224))
        cv.save()
        cv.clipPath(run.zone_path, doAntiAlias=True)
        bounds = run.zone_path.getBounds()
        height = bounds.height()
        spacing = 10 * math.sqrt(2)
        start = math.floor((bounds.left() - height) / spacing) * spacing
        x = start
        hatch = stroke(ZONE_LINE, 1.5, 89)
        while x < bounds.right() + height:
            cv.drawLine(x, bounds.bottom(), x + height, bounds.top(), hatch)
            x += spacing
        cv.restore()
        border = stroke(ZONE_BORDER, 2)
        border.setPathEffect(skia.DashPathEffect.Make([6, 4], 0))
        glow = stroke(ZONE_BORDER, 2, 150)
        glow.setPathEffect(skia.DashPathEffect.Make([6, 4], 0))
        glow.setMaskFilter(skia.MaskFilter.MakeBlur(skia.kNormal_BlurStyle, 4))
        cv.drawPath(run.zone_border, glow)
        cv.drawPath(run.zone_border, border)
    points = run.d.get("loop_polyline", [])
    if points:
        path = skia.Path()
        path.moveTo(*px(*points[0]))
        for point in points[1:]:
            path.lineTo(*px(*point))
        path.close()
        cv.drawPath(path, stroke("#475569", 1, 90))
    for base, depot in enumerate(run.depots):
        dx, dy = px(*depot)
        closed = base in st.get("closed_depots", [])
        cv.drawPath(star(dx, dy, 12), stroke(DEPOT, 2) if closed else fill(DEPOT))
        label = f"D{base + 1}" + (" closed" if closed else "") if run.extended else "Depot"
        text(cv, label, dx, dy - 22, "small", INK, "center")

    fly, failures = [], [0] * len(run.depots)
    if "failed_uavs" in st:
        failed = set(st["failed_uavs"])
    else:
        failed = {f"{int(event['uav']):02d}" for event in run.events
                  if event["kind"] == "failure" and event["t"] <= st["t"]}
    for uid, (x, y, _) in zip(run.ids, uav):
        if uid in failed:
            base = run.failure_depots.get(uid)
            if base is None:
                base = min(range(len(run.depots)),
                           key=lambda j: math.dist((x, y), run.depots[j]))
            failures[base] += 1
        elif all(math.hypot(x - dx, y - dy) >= 1.0 for dx, dy in run.depots):
            fly.append((x, y))
    for x, y in sorted(fly, key=lambda p: -p[1]):
        marker(cv, *px(x, y))
    cross = stroke(FAIL, 2.5)
    for depot, count in zip(run.depots, failures):
        for x, y in failure_crosses(depot, count):
            cv.drawLine(x - 6, y - 6, x + 6, y + 6, cross)
            cv.drawLine(x - 6, y + 6, x + 6, y - 6, cross)

    draw_hud(cv, run, st, compact=compact_hud)
    draw_legend(cv, bool(run.d.get("zones")))
    draw_colorbar(cv)
    return i


def video_height(run):
    return max(900 if run.extended else 820, canvas_h(run))


def draw_video_frame(cv, run, pos, panel):
    map_height = canvas_h(run)
    cv.clear(color(CARD))
    cv.save()
    cv.translate(0, (panel.height - map_height) / 2)
    index = draw_frame(cv, run, pos, map_height)
    cv.restore()
    cv.save()
    cv.translate(W, 0)
    panel.draw(cv, index)
    cv.restore()


class Encoder:
    def __init__(self, path, w, h, fps, crf, preset):
        head = ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgba", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-"]
        if Path(path).suffix.lower() == ".gif":  # straight from rgb so no chroma subsampling reaches the palette
            tail = ["-vf", "split[a][b];[a]palettegen=max_colors=256:stats_mode=full[p];[b][p]paletteuse=dither=none:diff_mode=rectangle",
                    "-loop", "0", str(path)]
        else:
            tail = ["-c:v", "libx264", "-crf", str(crf), "-preset", preset, "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)]
        self.p = subprocess.Popen(head + tail, stdin=subprocess.PIPE)

    def write(self, b):
        self.p.stdin.write(b)

    def close(self):
        self.p.stdin.close()
        if self.p.wait():
            raise RuntimeError("ffmpeg failed to encode the animation")


def render(run, positions, out, fps, crf=18, limit=None, scale=1.0, *, preset="medium"):
    gif = Path(out).suffix.lower() == ".gif"
    w0, h0 = (W, canvas_h(run)) if gif else (VIDEO_W, video_height(run))
    panel = None if gif else MonitorPanel(run, h0)
    w, h = round(w0 * scale) // 2 * 2, round(h0 * scale) // 2 * 2  # yuv420p needs even sides
    surface = skia.Surface(w, h)
    cv = surface.getCanvas()
    enc = Encoder(out, w, h, fps, crf, preset)
    t0 = time.time()
    for n, pos in enumerate(positions[:limit]):
        cv.save()
        cv.scale(w / w0, h / h0)
        if panel is None:
            draw_frame(cv, run, pos, h0, compact_hud=True)
        else:
            draw_video_frame(cv, run, pos, panel)
        cv.restore()
        arr = surface.makeImageSnapshot().toarray(colorType=skia.kRGBA_8888_ColorType)
        enc.write(arr.tobytes())
        if n % 1000 == 0:
            print(f"  {n}/{len(positions)} {time.time() - t0:.0f}s", flush=True)
    enc.close()


def video_positions(run, fps, speed=20.0):
    n = int((run.n - 1) * fps / speed) + 1
    return [k * speed / fps for k in range(n)]


def gif_positions(run, n_frames):
    return [k * (run.n - 1) / (n_frames - 1) for k in range(n_frames)]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("data", type=Path, help="frames json from export_frames.py")
    ap.add_argument("out", type=Path, nargs="?", help="output file (default under assets/)")
    ap.add_argument("--mode", choices=["video", "gif", "still"], default="video")
    ap.add_argument("--fps", type=float, help="default 30 for a video, 12 for a gif")
    ap.add_argument("--gif-frames", type=int, default=144)
    ap.add_argument("--speed", type=float, default=20.0, help="simulation seconds per video second")
    ap.add_argument("--limit", type=int)
    ap.add_argument("--stepped", action="store_true")
    ap.add_argument("--scale", type=float, help="layout scale (default 1.6 for GIF, 1.12 for video or still)")
    ap.add_argument("--palette", choices=sorted(PALETTES), default="D")
    ap.add_argument("--at", type=float, default=0.0, help="tick position for --mode still")
    a = ap.parse_args()
    global LEVELS
    set_palette(a.palette)
    run = Run(a.data, compact_baseline=a.mode in {"video", "gif"})
    stem = re.sub(r"[^a-z0-9]+", "-", a.data.stem.lower()).strip("-") or "scenario"
    defaults = {
        "video": Path(f"assets/multi_depot/videos/{stem}.mp4"),
        "gif": Path(f"assets/multi_depot/{stem}-surveillance.gif"),
        "still": Path(f"results/{stem}.png"),
    }
    out = a.out or defaults[a.mode]
    out.parent.mkdir(parents=True, exist_ok=True)
    scale = a.scale if a.scale is not None else (1.6 if a.mode == "gif" else 1.12)
    if a.mode == "still":
        LEVELS = 32 if a.stepped else None
        h = video_height(run)
        surface = skia.Surface(round(VIDEO_W * scale), round(h * scale))
        cv = surface.getCanvas()
        cv.scale(scale, scale)
        draw_video_frame(cv, run, a.at, MonitorPanel(run, h))
        surface.makeImageSnapshot().save(str(out), skia.kPNG)
    elif a.mode == "video":
        render(run, video_positions(run, a.fps or 30, speed=a.speed), out, a.fps or 30,
               limit=a.limit, scale=scale)
    else:
        LEVELS = 32
        render(run, gif_positions(run, a.gif_frames), out, a.fps or 12, crf=10, scale=scale)


if __name__ == "__main__":
    main()
