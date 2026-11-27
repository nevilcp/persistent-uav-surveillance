"""Patrol videos and the README GIF, drawn with skia-python from export_frames.py output."""

import argparse
import json
import math
import re
import subprocess
import time
from bisect import bisect_right
from functools import lru_cache
from itertools import pairwise
from pathlib import Path

import numpy as np
import skia

if __package__:
    from .media_names import PORT_SCENES, media_stem
else:
    from media_names import PORT_SCENES, media_stem

W = 1200
PLOT = (66, 62, 1010)  # left, top, width of the metric plot
WIDE_MAP_HALF_WIDTH = 1135.0  # metres
WIDE_MAP_SCALE = PLOT[2] / (2 * WIDE_MAP_HALF_WIDTH)
XLIM, YLIM = (-550.0, 550.0), (-50.0, 550.0)
SCALE = PLOT[2] / (XLIM[1] - XLIM[0])
PLOT_H = (YLIM[1] - YLIM[0]) * SCALE
AXIS_GAP = 9
SNAP = 25.0  # metres; a larger jump between ticks is a relaunch or swap, not flight
INK, MUTED, RULE = "#14181f", "#465063", "#d6dce4"
CARD = "#ffffff"
PLOT_BG, GRID = "#fafbfc", "#e1e6ec"
MONITOR_SOLID, MONITOR_DASHED = "#231561", "#9D3F58"
FAIL = "#9a3412"
ZONE = "#1E1B3A"
ZONE_LINE = "#C4B5FD"
ZONE_BORDER = "#A78BFA"
DEPOT = "#000000"
PALETTES = {  # red -> green as RdYlGn; stops at 0, .25, .5, .75, 1
    "A": ["#b3262d", "#ea6a3c", "#f3d263", "#86c06c", "#1c7c4d"],
    "B": ["#951c2e", "#d0502c", "#dfa52b", "#5b9f55", "#0f6a3d"],
    "C": ["#7f1527", "#be4528", "#cf9a1e", "#4f9150", "#0b5c35"],
    "D": ["#90162b", "#d64823", "#e8b02d", "#63a749", "#0a6539"],
}
RAMP = [(i / 4, c) for i, c in enumerate(PALETTES["D"])]

TYPE = {
    "tick": (12, 400),
    "dashboard_title": (27, 700),
    "dashboard_map_title": (30, 700),
    "dashboard_heading": (16, 700),
    "dashboard_monitor_heading": (18, 700),
    "dashboard_label": (16, 400),
    "dashboard_tick": (14, 400),
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


class Fonts:
    def __init__(self):
        self.mgr = skia.FontMgr()
        self.cache = {}

    def get(self, style):
        if style not in self.cache:
            size, weight = TYPE[style]
            face = self.mgr.matchFamilyStyle(
                "Inter",
                skia.FontStyle(weight, skia.FontStyle.kNormal_Width, skia.FontStyle.kUpright_Slant),
            )
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
    "Single Depot: West": "West Depot",
    "Single Depot: East": "East Depot",
    "Single Depot: North": "North Depot",
    "Single depot: west": "West Depot",
    "Single depot: east": "East Depot",
    "Single depot: north": "North Depot",
    "No-Fly Zones": "No-fly zones",
    "Single Failure": "Single failure",
    "Repeated Failures": "Repeated failures",
    "Depot Closure": "Depot closure",
    "Return Metric Control": "Return metric control",
    "Multiple Depots": "Multiple depots",
}

LEGACY_PORT_TITLES = {
    ("R0", "Return metric control"): "Straight-line dispatch",
    ("R0", "Port surveillance"): "Straight-line dispatch",
    ("R2", "Multiple depots"): "No NFZs",
    ("R2", "Port surveillance"): "No NFZs",
    ("R3", "No-fly zones"): "Nominal",
    ("R3", "Port surveillance"): "Nominal",
}

BASELINE_TITLES = {
    "under_provisioned": "Under-Provisioned",
    "nominal": "Nominal",
    "over_provisioned": "Over-Provisioned",
    "long_horizon": "Long Horizon",
    "single_failure": "Single Failure",
}


class Run:
    def __init__(self, path, *, compact_baseline=False):
        d = json.loads(Path(path).read_text())
        self.d = d
        fr = d["frames"]
        self.n = len(fr)
        self.fresh = np.array([f["fresh"] for f in fr], dtype=np.float32)
        self.ids = [u[0] for u in fr[0]["uavs"]]
        self.uav = np.array([[u[1:] for u in f["uavs"]] for f in fr], dtype=np.float32)  # tick, uav, (x y soc)
        self.fr = fr
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
                label = f"D{int(event['detail'].split()[-1]) + 1} Closure"
            else:
                continue
            self.event_labels.append((event["t"], label))
        self.failure_times = sorted(event["t"] for event in self.events if event["kind"] == "failure")
        self.cells = d["cells"]
        self.depot = d["depot"]
        self.depots = d.get("depots", [self.depot])
        self.extended = "extent" in d
        global XLIM, YLIM, SCALE, PLOT_H
        XLIM, YLIM = (-550.0, 550.0), (-50.0, 550.0)
        if not compact_baseline or self.extended:
            cx, cy = (600, 850) if self.extended else (0, 250)
            XLIM, YLIM = (cx - WIDE_MAP_HALF_WIDTH, cx + WIDE_MAP_HALF_WIDTH), (cy - 895, cy + 895)
        SCALE = PLOT[2] / (XLIM[1] - XLIM[0])
        PLOT_H = (YLIM[1] - YLIM[0]) * SCALE
        # Keep the triangle proportional to a cell when zooming the baseline map.
        self.uav_marker_scale = SCALE / WIDE_MAP_SCALE
        title = " ".join(d["title"].split())
        self.title = LEGACY_SCENARIO_TITLES.get(title, title)
        study_id = re.match(r"R\d+(?=\b|_)", d.get("name") or d.get("meta", ""), re.IGNORECASE)
        self.study_id = study_id.group().upper() if study_id else ""
        if self.title == "Port surveillance" and d.get("name") in PORT_SCENES:
            self.title = PORT_SCENES[d["name"]][1]
        self.title = LEGACY_PORT_TITLES.get((self.study_id, self.title), self.title)
        baseline_title = BASELINE_TITLES.get(d.get("name") or Path(path).stem)
        if not self.extended and self.title == "Continuous patrol" and baseline_title:
            self.title = baseline_title
        self.display_title = f"{self.study_id} · {self.title}" if self.study_id else self.title
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


def marker(cv, x, y, scale=1.0):
    path = triangle(x, y, 5.2 * scale)
    cv.drawPath(path, fill(CARD))
    cv.drawPath(path, stroke(INK, 1.25 * scale))


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

    def format_tick(self, value):
        return f"{value:g}".replace("-", "−")


def monitor_tracks(run):
    fr = run.fr
    limits = run.d.get("monitor_limits", {})
    deadline = limits.get("recovery")

    def read(key):
        return [f.get(key) for f in fr]

    def constant(value):
        return [value] * len(fr)

    def seconds(value):
        return "—" if value is None else f"{value:g} s"

    def count_labels(values):
        return ["—" if value is None else f"{value:,}" for value in values]

    def gauge(heading, key, unit, factor=1, integer=False):
        values = [None if value is None else value * factor for value in read(key)]
        margin_format = "+,.0f" if integer else "+,.1f"
        counts = [
            frame.get("current_violations", {}).get(key) if value is not None else None
            for frame, value in zip(fr, values)
        ]
        margins = ["—" if value is None else f"{value:{margin_format}}{unit}" for value in values]
        return MonitorTrack(heading, unit, [("Instant", values)],
                            [("Margin", margins), ("Violations", count_labels(counts))],
                            integer=integer)

    recoveries = run.d.get("recoveries")
    if recoveries is None:
        recoveries = []
        for event in run.events:
            if event["kind"] == "failure":
                failure = run.failure or {}
                restored = failure.get("t_restored") if event["t"] == failure.get("t_fail") else None
                recoveries.append({
                    "t_fail_s": event["t"],
                    "recovery_time_s": None if restored is None else restored - event["t"],
                })
    recovery_values, failure_counts, recovery_readouts, misses = [], [], [], []
    started = 0
    for frame in fr:
        now = frame["t"]
        while started < len(recoveries) and recoveries[started]["t_fail_s"] <= now:
            started += 1
        elapsed = None
        if started:
            recovery = recoveries[started - 1]
            duration = recovery["recovery_time_s"]
            age = now - recovery["t_fail_s"]
            if duration is None or age < duration:
                elapsed = age
        active = elapsed is not None
        recovery_values.append(elapsed)
        failure_counts.append(str(int(active)))
        recovery_readouts.append(seconds(elapsed))
        misses.append(str(int(active and deadline is not None and elapsed >= deadline)))

    alarm_traces = [("Late", read("c3_overdue")), ("Waiting", read("c3_pending"))]
    alarm_fields = [(name, count_labels(values)) for name, values in alarm_traces]
    coverage = read("cov")
    return [
        MonitorTrack("C1: Coverage", "%", [("Instant", coverage)],
                     [("Fresh cells", [f"{value:.1f}%" for value in coverage]),
                      ("Overdue cells", [f"{round(len(run.cells) * (1 - value / 100)):,}" for value in coverage]),
                      ("Age limit", constant(seconds(limits.get("freshness"))))], ticks=tuple(range(0, 101, 20))),
        gauge("C2: Battery reserve margin", "battery_margin", " pp", 100),
        MonitorTrack("C3: Spare launch", " Alarms", alarm_traces,
                     [*alarm_fields, ("Deadline", constant(seconds(limits.get("launch"))))], integer=True),
        MonitorTrack("C4: Coverage recovery", " s",
                     [("Elapsed", recovery_values), ("Deadline", constant(deadline))],
                     [("Active", failure_counts), ("Elapsed", recovery_readouts),
                      ("Deadline", constant(seconds(deadline))), ("Overdue", misses)],
                     breaks=[entry["t_fail_s"] for entry in recoveries]),
        gauge("C5: Return energy margin", "way_home_margin", " pp", 100),
        gauge("C6: Depot queue margin", "queue_margin", " UAVs", integer=True),
        gauge("C7: No-fly clearance margin", "zone_clearance", " m"),
    ]


def failure_crosses(depot, count):
    """Fill a two-column grid beside the depot, preserving earlier positions."""
    x, y = px(*depot)
    left, top, width = PLOT
    right, bottom = left + width, top + PLOT_H
    side = -1 if x - left > right - x else 1
    direction = -1 if bottom - y < 40 else 1
    return [(x + side * (22 + 19 * (slot % 2)), y + direction * 19 * (slot // 2))
            for slot in range(count)]


def draw_surveillance_grid(cv, run, fresh):
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


def draw_map_markers(cv, run, st, uav):
    points = run.d.get("loop_polyline", [])
    if points:
        path = skia.Path()
        path.moveTo(*px(*points[0]))
        for point in points[1:]:
            path.lineTo(*px(*point))
        path.close()
        cv.drawPath(path, stroke("#475569", 1, 90))
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
        marker(cv, *px(x, y), scale=run.uav_marker_scale)
    cross = stroke(FAIL, 2.5)
    for depot, count in zip(run.depots, failures):
        for x, y in failure_crosses(depot, count):
            cv.drawLine(x - 6, y - 6, x + 6, y + 6, cross)
            cv.drawLine(x - 6, y + 6, x + 6, y - 6, cross)


class R3Dashboard:
    """A 16:9 map and STL dashboard for the baseline and port studies."""

    width, height = 1920, 1080
    margin = 52
    map_x, map_y, map_scale = 80, 213, 0.84
    title_y = 72
    metrics_bottom = title_y + 85
    chart_left, chart_right = 1100, width - margin - 17
    chart_top, row_gap = 133, 35
    metric_separator, metric_gap, metric_padding = "·", 4, 6
    metric_key_width, metric_key_gap = 12, 5
    title, heading, label, tick = (
        "dashboard_title", "dashboard_heading", "dashboard_label", "dashboard_tick",
    )
    trace, comparison = MONITOR_SOLID, MONITOR_DASHED
    plot_grid, zero_rule, cursor_ink = "#e5e5ed", "#c9c4d6", "#665a79"
    map_depot, map_warning = DEPOT, "#a14022"
    negative_margins = (1, 4, 5, 6)
    field_names = (
        ("Fresh", "Overdue", "Age Limit"),
        ("Margin", "Violations"),
        ("Late", "Waiting", "Deadline"),
        ("Active", "Elapsed", "Deadline", "Overdue"),
        ("Margin", "Violations"),
        ("Margin", "Violations"),
        ("Margin", "Violations"),
    )

    def __init__(self, run):
        self.run = run
        self.violation = RAMP[0][1]
        self.tracks = monitor_tracks(run)
        self.monitor_keys = {
            2: {0: (self.violation, False), 1: (self.comparison, True)},
            3: {1: (self.trace, False), 2: (self.comparison, True)},
        }
        # Reserve space for the widest reading in each field of this table.
        self.metric_widths = [
            [2 * self.metric_padding + measure(self.tick, label)
             + measure(self.tick, self.metric_separator) + 2 * self.metric_gap
             + max(measure(self.heading, self.readout(value), True) for value in set(values))
             + (self.metric_key_width + self.metric_key_gap if col in self.monitor_keys.get(row, {}) else 0)
             for col, (label, (_, values)) in enumerate(zip(self.field_names[row], track.fields))]
            for row, track in enumerate(self.tracks)
        ]
        self.times = [frame["t"] for frame in run.fr]
        self.start = 0 if self.times[0] == 1 else self.times[0]
        self.end = self.times[-1]
        map_height = round(PLOT[1] + PLOT_H + 62)
        self.map_canvas_height = max(900 if run.extended else 820, map_height)
        self.map_offset = (self.map_canvas_height - map_height) / 2
        self.map_layer = lru_cache(maxsize=4)(self.map_layer)
        if run.extended:
            bounds = (-600, 1820, 1800, -100)
            self.map_xticks = range(-600, 1801, 200)
            self.map_yticks = range(0, 1801, 200)
        else:
            bounds = (-600, 730, 600, -230)
            self.map_xticks = range(-600, 601, 100)
            self.map_yticks = range(-200, 701, 100)
            self.map_scale = (0.84 * WIDE_MAP_SCALE * 2400) / (1200 * SCALE)
            left, top = px(*bounds[:2])
            self.map_x = 111 - self.map_scale * left
            self.map_y = 237 - self.map_scale * (top + self.map_offset)
        self.map_area = skia.Rect.MakeLTRB(*self.map_point(*bounds[:2]), *self.map_point(*bounds[2:]))
        xs, ys = zip(*run.cells)
        half = run.d["cell_size"] / 2
        self.grid = skia.Rect.MakeLTRB(*px(min(xs) - half, max(ys) + half),
                                      *px(max(xs) + half, min(ys) - half))
        if not run.zone_path.isEmpty():
            self.grid.join(run.zone_border.getBounds().makeOutset(14, 14))
        self.grid.outset(2, 2)
        self.row_y = (self.map_area.bottom() - self.chart_top + self.row_gap) / len(self.tracks)
        self.axes = [self.axis(row, track) for row, track in enumerate(self.tracks)]
        self.paths = [[self.path(row, series, track.breaks if j == 0 else ())
                       for j, (_, series) in enumerate(track.traces)]
                      for row, track in enumerate(self.tracks)]
        self.violations = {
            row: self.violation_path(row, self.tracks[row].traces[0][1])
            for row in (*self.negative_margins, 2)
        }
        self.time_ticks = self.make_time_ticks()
        self.has_alarm_data = any(v is not None for v in self.tracks[2].traces[1][1])
        self.has_recovery = any(v is not None for v in self.tracks[3].traces[0][1])

    def map_point(self, x, y):
        mx, my = px(x, y)
        return self.map_x + self.map_scale * mx, self.map_y + self.map_scale * (my + self.map_offset)

    def map_transform(self, cv):
        # Use a whole-device-pixel translation, including at alternate output scales.
        matrix = cv.getTotalMatrix()
        sx, sy = matrix.getScaleX(), matrix.getScaleY()
        cv.translate(round(self.map_x * sx) / sx, round(self.map_y * sy) / sy)
        cv.scale(self.map_scale, self.map_scale)
        cv.translate(0, self.map_offset)

    def map_layer(self, sx, sy):
        # Rasterize at the legacy origin. Fractional transforms after translation
        # can shift antialiasing by a pixel even when the translation is integral.
        sx, sy = self.map_scale * sx, self.map_scale * sy
        region = skia.IRect.MakeLTRB(
            math.floor(self.grid.left() * sx), math.floor((self.grid.top() + self.map_offset) * sy),
            math.ceil(self.grid.right() * sx), math.ceil((self.grid.bottom() + self.map_offset) * sy),
        )
        surface = skia.Surface(round(W * sx), round(self.map_canvas_height * sy))
        cv = surface.getCanvas()
        cv.clear(color(CARD))
        cv.scale(sx, sy)
        cv.translate(0, self.map_offset)
        draw_axes(cv, self.run)
        return surface, region, surface.makeImageSnapshot()

    def draw_protected_map(self, cv, st, fresh, uav):
        matrix = cv.getTotalMatrix()
        sx, sy = matrix.getScaleX(), matrix.getScaleY()
        surface, region, base = self.map_layer(sx, sy)
        layer = surface.getCanvas()
        layer.save()
        layer.resetMatrix()
        layer.drawImage(base, 0, 0)
        layer.restore()
        draw_surveillance_grid(layer, self.run, fresh)
        draw_map_markers(layer, self.run, st, uav)
        pixels = surface.makeImageSnapshot().makeSubset(region)
        cv.save()
        cv.resetMatrix()
        cv.drawImage(pixels, round(self.map_x * sx + matrix.getTranslateX()) + region.left(),
                     round(self.map_y * sy + matrix.getTranslateY()) + region.top())
        cv.restore()

    def map_header(self, cv, st):
        title = re.sub(r"\b[a-z]", lambda match: match[0].upper(), self.run.display_title)
        text(cv, title, self.margin, self.title_y, "dashboard_map_title")
        due, unstaffed = st.get("routes_due"), st.get("routes_unstaffed")
        routes = f"{unstaffed:,} / {due:,}" if due and unstaffed is not None else "None Due" if due == 0 else "—"
        losses = st.get("failed")
        if losses is None:
            losses = (len(st["failed_uavs"]) if "failed_uavs" in st
                      else bisect_right(self.run.failure_times, st["t"]))
        fields = (
            ("Time (s)", self.readout(f"{st['t']:g}")),
            ("Unserved Routes", routes),
            ("Patrolling UAVs", f"{st['active']:,}"),
            ("Reserve UAVs", f"{st['spare']:,}"),
            ("UAV Losses", f"{losses:,}"),
        )
        box = skia.Rect.MakeLTRB(self.margin, self.title_y + 20, self.map_area.right(), self.metrics_bottom)
        cv.drawRect(box, fill(PLOT_BG))
        cv.drawRect(box, stroke(RULE, 1))
        for j, (label, value) in enumerate(fields):
            x = box.left() + 15 + j * box.width() / len(fields)
            text(cv, label, x, box.top() + 24, self.tick, MUTED)
            is_status = value in {"None Due", "—"}
            ink = self.map_warning if (j == 1 and unstaffed) or (j == 4 and losses) else INK
            text(cv, value, x, box.top() + 54, self.label if is_status else self.title,
                 MUTED if is_status else ink, tab=True)

        x, y = self.margin + 9, box.bottom() + 29
        cv.drawPath(star(x, y, 9), fill(self.map_depot))
        x += 16 + text(cv, "Depot", x + 16, y + 5, self.label) + 28
        marker(cv, x, y)
        x += 13 + text(cv, "UAV", x + 13, y + 5, self.label) + 23
        if self.run.d.get("zones"):
            swatch = skia.Rect.MakeXYWH(x, y - 8, 19, 16)
            cv.drawRect(swatch, fill(ZONE, 224))
            cv.save()
            cv.clipRect(swatch)
            for offset in (-12, -2, 8, 18):
                cv.drawLine(x + offset, y + 8, x + offset + 16, y - 8, stroke(ZONE_LINE, 1.3, 89))
            cv.restore()
            cv.drawRect(swatch.makeInset(0.5, 0.5), self.pen(ZONE_BORDER, 1.4, (5, 3)))
            text(cv, "No-Fly Zone", x + 27, y + 5, self.label)

    def draw_map(self, cv, index, fresh, uav):
        st = self.run.fr[index]
        self.map_header(cv, st)
        left, top, right, bottom = tuple(self.map_area)
        cv.drawRect(self.map_area, fill(PLOT_BG))
        grid = stroke("#e9edf2", 1)
        for value in self.map_xticks:
            x, _ = self.map_point(value, 0)
            cv.drawLine(x, top, x, bottom, grid)
            text(cv, self.readout(str(value)), x, bottom + 45, self.tick, MUTED, "center", tab=True)
        for value in self.map_yticks:
            _, y = self.map_point(0, value)
            cv.drawLine(left, y, right, y, grid)
            text(cv, str(value), left - 8, y + 5, self.tick, MUTED, "right", tab=True)
        text(cv, "X (m)", (left + right) / 2, bottom + 70, self.label, MUTED, "center")
        self.vertical_label(cv, "Y (m)", left - 48, (top + bottom) / 2)

        cv.save()
        self.map_transform(cv)
        draw_map_markers(cv, self.run, st, uav)
        cv.restore()
        self.draw_protected_map(cv, st, fresh, uav)
        for depot, point in enumerate(self.run.depots):
            x, y = self.map_point(*point)
            closed = depot in st.get("closed_depots", ())
            cv.drawPath(star(x, y, 10), stroke(self.map_depot, 2) if closed else fill(self.map_depot))
            label = f"D{depot + 1}" + (" Closed" if closed else "")
            text(cv, label, x, y - 16, self.heading, self.map_warning if closed else INK, "center")

        self.draw_freshness_scale(cv)

    def draw_freshness_scale(self, cv):
        right = self.map_area.right() - 17
        left = right - 420
        height = 12
        gap_center = (self.metrics_bottom + self.map_area.top()) / 2
        top = gap_center - height / 2
        bar = skia.Rect.MakeXYWH(left, top, right - left, height)
        endcap = 12
        if LEVELS:
            width = bar.width() / LEVELS
            for j in range(LEVELS):
                swatch = skia.Rect.MakeXYWH(left + j * width, top, width, bar.height())
                cv.drawRect(swatch, fill(LUT[round(255 * j / (LEVELS - 1))]))
        else:
            # Clamp the gradient into flat endpoint swatches, so the reference
            # pixels sit away from chroma subsampling at the outside edges.
            shader = skia.GradientShader.MakeLinear(
                [(left + endcap, top), (right - endcap, top)],
                [color(c) for _, c in RAMP], [p for p, _ in RAMP],
            )
            cv.drawRect(bar, skia.Paint(Shader=shader))
        for value in range(0, 101, 20):
            if value == 0:
                x = left + endcap / 2
            elif value == 100:
                x = right - endcap / 2
            else:
                inset = 0 if LEVELS else endcap
                x = left + inset + (bar.width() - 2 * inset) * value / 100
            cv.drawLine(x, bar.bottom(), x, bar.bottom() + 4, stroke(RULE, 1))
            text(cv, str(value), x, bar.bottom() + 19, self.tick, MUTED, "center", tab=True)
        text(cv, "Cell Freshness (%)", left, top - 9, self.label, MUTED)

    def vertical_label(self, cv, label, x, y):
        cv.save()
        cv.translate(x, y)
        cv.rotate(-90)
        text(cv, label, 0, 0, self.label, MUTED, "center")
        cv.restore()

    @staticmethod
    def readout(value):
        return value.replace("-", "−").replace(" ", "\u00a0")

    def make_time_ticks(self):
        # Choose regular elapsed-time ticks. Leave room for endpoint labels.
        if self.start == self.end:
            return [self.start]
        step = nice_step(self.end - self.start, 5, minimum=0.001)
        interior = range(math.ceil(self.start / step), math.floor(self.end / step) + 1)
        return [self.start, *(i * step for i in interior
                              if self.start + step / 4 < i * step < self.end - step / 4), self.end]

    def x(self, timestamp):
        return self.chart_left + (self.chart_right - self.chart_left) * (
            (timestamp - self.start) / (self.end - self.start or 1))

    def area(self, row):
        top = self.chart_top + row * self.row_y
        return skia.Rect.MakeLTRB(self.chart_left, top, self.chart_right, top + self.row_y - self.row_gap)

    @staticmethod
    def axis(row, track):
        ticks = track.ticks
        low, high = ticks[0], ticks[-1]
        pad = (high - low) * 0.12
        if row == 0:
            ticks = (0, 50, 100)
        elif len(ticks) > 3:
            midpoint = (low + high) / 2
            ticks = (low, round(midpoint) if row in (2, 5) else midpoint, high)
        return ticks, low - pad, high + pad

    def y(self, row, value):
        area = self.area(row)
        _, low, high = self.axes[row]
        return area.bottom() - area.height() * (value - low) / (high - low)

    def path(self, row, series, breaks):
        path, drawing = skia.Path(), False
        area = self.area(row)
        _, low, high = self.axes[row]
        for timestamp, value in zip(self.times, series):
            if value is None:
                drawing = False
                continue
            y = area.bottom() - area.height() * (value - low) / (high - low)
            (path.lineTo if drawing and timestamp not in breaks else path.moveTo)(self.x(timestamp), y)
            drawing = True
        return path

    def violation_path(self, row, series):
        path = skia.Path()
        previous_drawn = False
        threshold = max if row == 2 else min
        for i, (a, b) in enumerate(pairwise(series)):
            bad = a is not None and b is not None and self.is_violation(row, threshold(a, b))
            if bad:
                if not previous_drawn:
                    path.moveTo(self.x(self.times[i]), self.y(row, a))
                path.lineTo(self.x(self.times[i + 1]), self.y(row, b))
            previous_drawn = bad
        return path

    def is_violation(self, row, value):
        return value > 0 if row == 2 else row in self.negative_margins and value < 0

    @staticmethod
    def pen(ink, width, dash=()):
        paint = stroke(ink, width)
        if dash:
            paint.setPathEffect(skia.DashPathEffect.Make(dash, 0))
        return paint

    def series_sample(self, cv, x, y, ink, *, dashed=False, width=24):
        cv.drawLine(x, y - 5, x + width, y - 5, self.pen(ink, 2, (5, 4) if dashed else ()))

    def draw_paths(self, cv, row, area, cursor):
        path_count = len(self.paths[row])
        cv.save()
        cv.clipRect(area)
        if row == 3:
            cv.drawPath(self.paths[row][1], self.pen(self.comparison, 2, (5, 4)))
            path_count -= 1
        cv.clipRect(skia.Rect.MakeLTRB(area.left(), area.top(), cursor, area.bottom()))
        for j in reversed(range(path_count)):
            path = self.paths[row][j]
            cv.drawPath(path, self.pen(self.comparison if j else self.trace, 2, (5, 4) if j else ()))
        if row in self.violations:
            zero = self.y(row, 0)
            top, bottom = (area.top(), zero) if row == 2 else (zero, area.bottom())
            cv.save()
            cv.clipRect(skia.Rect.MakeLTRB(area.left(), top, area.right(), bottom))
            cv.drawPath(self.violations[row], self.pen(self.violation, 2))
            cv.restore()
        cv.restore()

    def draw_monitor_metrics(self, cv, row, index, right, baseline):
        fields = self.tracks[row].fields
        widths = self.metric_widths[row]
        left = right - sum(widths)
        box = skia.Rect.MakeLTRB(left, baseline - 19, right, baseline + 7)
        border = stroke(RULE, 1)
        cv.drawRect(box, fill(PLOT_BG))
        cv.drawRect(box, border)
        for col, (width, label, (_, series)) in enumerate(
            zip(widths, self.field_names[row], fields)
        ):
            value = series[index]
            readout = self.readout(value)
            x = left + self.metric_padding
            key = self.monitor_keys.get(row, {}).get(col)
            if key is not None:
                ink, dashed = key
                self.series_sample(cv, x, baseline, ink, dashed=dashed, width=self.metric_key_width)
                x += self.metric_key_width + self.metric_key_gap
            x += text(cv, label, x, baseline, self.tick, MUTED) + self.metric_gap
            x += text(cv, self.metric_separator, x, baseline, self.tick, MUTED) + self.metric_gap
            violation_count = label in {"Violations", "Late"} or (row == 3 and label == "Overdue")
            warning = value.startswith("-") or (violation_count and value not in {"0", "—"})
            text(cv, readout, x, baseline, self.heading,
                 self.violation if warning else INK, tab=True)
            left += width

    def draw_monitors(self, cv, index):
        now = self.times[index]
        cursor = self.x(now)
        events = [(t, label) for t, label in self.run.event_labels if self.start <= t <= now]
        text(cv, "STL Monitors", self.chart_left, self.title_y, self.title)

        for row, track in enumerate(self.tracks):
            area = self.area(row)
            top, bottom = area.top(), area.bottom()
            header_y = top - 12
            unit = track.unit.strip().replace("Alarms", "alarms")
            name = track.heading.removesuffix(" margin").replace(": ", " · ").title()
            name_width = text(cv, name, area.left(), header_y, "dashboard_monitor_heading")
            text(cv, f"({unit})", area.left() + name_width + 7, header_y, self.tick, MUTED)
            self.draw_monitor_metrics(cv, row, index, area.right(), header_y)

            cv.drawRect(area, fill(PLOT_BG))
            grid = stroke(self.plot_grid, 1)
            for timestamp in self.time_ticks:
                x = self.x(timestamp)
                cv.drawLine(x, top, x, bottom, grid)
            for value in self.axes[row][0]:
                y = self.y(row, value)
                cv.drawLine(area.left(), y, area.right(), y, grid)
                text(cv, track.format_tick(value), area.left() - 10, y + 5, self.tick, MUTED, "right", tab=True)
            if row in self.violations:
                y = self.y(row, 0)
                cv.drawLine(area.left(), y, area.right(), y, stroke(self.zero_rule, 1))
            for timestamp, _ in events:
                x = self.x(timestamp)
                cv.drawLine(x, top, x, bottom, self.pen(MUTED, 1, (3, 3)))
            self.draw_paths(cv, row, area, cursor)
            cv.drawLine(cursor, top, cursor, bottom, stroke(self.cursor_ink, 1.25, 215))
            note = ("Live Alarm Data Unavailable" if row == 2 and not self.has_alarm_data else
                    "No Active Recoveries" if row == 3 and not self.has_recovery else None)
            if note:
                ticks = self.axes[row][0]
                note_y = self.y(row, (ticks[-1] + ticks[-2]) / 2)
                note_width = measure(self.tick, note) + 14
                box = skia.Rect.MakeXYWH(area.centerX() - note_width / 2, note_y - 9, note_width, 19)
                cv.drawRect(box, fill(PLOT_BG))
                text(cv, note, area.centerX(), note_y + 5, self.tick, MUTED, "center")
            for j, (_, series) in reversed(list(enumerate(track.traces))):
                value = series[index]
                if value is None or (row == 3 and j == 1):
                    continue
                ink = self.comparison if j else self.violation if self.is_violation(row, value) else self.trace
                y = self.y(row, value)
                cv.drawCircle(cursor, y, 4.8 if j else 3.6, fill(CARD if j else ink))
                cv.drawCircle(cursor, y, 4.8 if j else 3.6, stroke(ink if j else CARD, 1))

        tag = self.readout(f"{now:g} s")
        half = max(measure(self.tick, tag, True), measure(self.tick, self.readout(f"{self.end:g} s"), True)) / 2 + 9
        tag_x = min(max(cursor, self.chart_left + half - 8), self.chart_right - half + 8)
        rect = skia.Rect.MakeXYWH(tag_x - half, bottom + 8, 2 * half, 23)
        cv.drawLine(cursor, bottom, cursor, rect.top(), stroke(self.cursor_ink, 1.25))
        cv.drawLine(cursor, rect.top(), tag_x, rect.top(), stroke(self.cursor_ink, 1.25))
        cv.drawRect(rect, fill(self.trace))
        text(cv, tag, tag_x, bottom + 24, self.tick, CARD, "center", tab=True)
        for timestamp in self.time_ticks:
            text(cv, self.readout(f"{timestamp:g}"), self.x(timestamp), bottom + 45,
                 self.tick, MUTED, "center", tab=True)
        text(cv, "Time (s)", (self.chart_left + self.chart_right) / 2, bottom + 70, self.label, MUTED, "center")
        lane_ends = [self.chart_left, self.chart_left]
        for timestamp, label in events:
            width = measure(self.tick, label)
            x = min(self.x(timestamp), self.chart_right - width)
            for lane, end in enumerate(lane_ends):
                if x >= end:
                    text(cv, label, x, bottom + 92 + lane * 16, self.tick, MUTED)
                    lane_ends[lane] = x + width + 10
                    break

    def draw(self, cv, pos, *, show_monitors=True):
        index, fresh, uav = self.run.at(pos)
        cv.clear(color(CARD))
        self.draw_map(cv, index, fresh, uav)
        if show_monitors:
            self.draw_monitors(cv, index)


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
    dashboard = R3Dashboard(run)
    w0, h0 = (dashboard.height if gif else dashboard.width), dashboard.height
    w, h = round(w0 * scale) // 2 * 2, round(h0 * scale) // 2 * 2  # yuv420p needs even sides
    surface = skia.Surface(w, h)
    cv = surface.getCanvas()
    offset_x = offset_y = 0
    if gif:
        # Include the title and outer axis labels when centering the map.
        cv.save()
        cv.scale(w / w0, h / h0)
        dashboard.draw(cv, positions[0], show_monitors=False)
        cv.restore()
        pixels = surface.makeImageSnapshot().toarray()
        content = np.any(pixels != pixels[0, 0], axis=2)
        rows = np.flatnonzero(content.any(axis=1))
        cols = np.flatnonzero(content.any(axis=0))
        offset_x = (w - 1 - cols[0] - cols[-1]) // 2
        offset_y = (h - 1 - rows[0] - rows[-1]) // 2
    enc = Encoder(out, w, h, fps, crf, preset)
    t0 = time.time()
    for n, pos in enumerate(positions[:limit]):
        cv.save()
        cv.translate(offset_x, offset_y)
        cv.scale(w / w0, h / h0)
        dashboard.draw(cv, pos, show_monitors=not gif)
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
    ap.add_argument("--scale", type=float, default=1.0, help="layout scale (default 1)")
    ap.add_argument("--palette", choices=sorted(PALETTES), default="D")
    still_position = ap.add_mutually_exclusive_group()
    still_position.add_argument("--at", type=float, default=0.0, help="tick position for --mode still")
    still_position.add_argument("--time", type=float, help="exact sample time in seconds for --mode still")
    a = ap.parse_args()
    global LEVELS
    set_palette(a.palette)
    run = Run(a.data, compact_baseline=a.mode in {"video", "gif"})
    category = "multi_depot" if run.extended else "single_depot"
    prefix = run.study_id if run.extended else "baseline"
    stem = media_stem(run.title, prefix)
    defaults = {
        "video": Path(f"assets/{category}/videos/{stem}.mp4"),
        "gif": Path(f"assets/{category}/{stem}.gif"),
        "still": Path(f"results/{media_stem(a.data.stem)}.png"),
    }
    out = a.out or defaults[a.mode]
    out.parent.mkdir(parents=True, exist_ok=True)
    scale = a.scale
    if a.mode == "still":
        LEVELS = 32 if a.stepped else None
        pos = a.at
        if a.time is not None:
            pos = next((i for i, frame in enumerate(run.fr) if frame["t"] == a.time), None)
            if pos is None:
                ap.error(f"the frame export has no sample at {a.time:g} s")
        dashboard = R3Dashboard(run)
        w, h = dashboard.width, dashboard.height
        width, height = round(w * scale), round(h * scale)
        surface = skia.Surface(width, height)
        cv = surface.getCanvas()
        cv.scale(width / w, height / h)
        dashboard.draw(cv, pos)
        surface.makeImageSnapshot().save(str(out), skia.kPNG)
    elif a.mode == "video":
        render(run, video_positions(run, a.fps or 30, speed=a.speed), out, a.fps or 30,
               limit=a.limit, scale=scale)
    else:
        LEVELS = 32
        render(run, gif_positions(run, a.gif_frames), out, a.fps or 12, crf=10, scale=scale)


if __name__ == "__main__":
    main()
