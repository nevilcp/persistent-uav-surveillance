"""Patrol videos and the README GIF, drawn with skia-python from export_frames.py output."""

import argparse
import json
import math
import subprocess
import time
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
BANNER_H = 46  # only runs with a failure reserve the banner row
LOST_SHOWN = 20.0  # seconds the first banner message stays up
SNAP = 25.0  # metres; a larger jump between ticks is a relaunch or swap, not flight
INK, MUTED, RULE = "#14181f", "#586174", "#e3e6eb"  # muted holds 6:1 on white at the 12 px minimum
BG, CARD = "#f4f5f7", "#ffffff"
FAIL = "#6d28d9"
DEPOT = "#e5383b"
PALETTES = {  # red -> green as RdYlGn; stops at 0, .25, .5, .75, 1
    "A": ["#b3262d", "#ea6a3c", "#f3d263", "#86c06c", "#1c7c4d"],
    "B": ["#951c2e", "#d0502c", "#dfa52b", "#5b9f55", "#0f6a3d"],
    "C": ["#7f1527", "#be4528", "#cf9a1e", "#4f9150", "#0b5c35"],
    "D": ["#9b1c31", "#d8502a", "#e8b02d", "#6aab50", "#12703f"],
}
RAMP = [(i / 4, c) for i, c in enumerate(PALETTES["D"])]

# type scale: one family, five sizes; weight carries the hierarchy
TYPE = {
    "title": (26, 600, "Inter Display"),
    "meta": (15, 400, "Inter"),
    "value": (15, 600, "Inter"),
    "label": (14, 500, "Inter"),
    "small": (13, 500, "Inter"),
    "tick": (12, 400, "Inter"),
    "chip_id": (12, 700, "Inter"),
    "chip_soc": (12, 500, "Inter"),
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
    p.setStrokeJoin(skia.Paint.kRound_Join)
    return p


def advances(f, s, tab):
    """Per-glyph advances; tabular digits are as wide as a zero so changing numbers do not shimmer."""
    zero = f.measureText("0")
    return [zero if tab and ch.isdigit() else f.measureText(ch) for ch in s]


def measure(style, s, tab=False):
    f = FONTS.get(style)
    return sum(advances(f, s, True)) if tab else f.measureText(s)


def text(cv, s, x, y, style, c=INK, align="left", tab=False):
    f = FONTS.get(style)
    w = measure(style, s, tab)
    x -= w if align == "right" else w / 2 if align == "center" else 0
    if not tab:
        cv.drawString(s, x, y, f, fill(c))
        return w
    for ch, adv in zip(s, advances(f, s, True)):
        gx = x + (adv - f.measureText(ch)) / 2 if ch.isdigit() else x
        cv.drawString(ch, gx, y, f, fill(c))
        x += adv
    return w


def card(cv, x, y, w, h, r=16):
    rr = skia.RRect.MakeRectXY(skia.Rect.MakeXYWH(x, y, w, h), r, r)
    shadow = fill("#101828", 10)
    shadow.setMaskFilter(skia.MaskFilter.MakeBlur(skia.kNormal_BlurStyle, 6))
    cv.drawRRect(rr.makeOffset(0, 2), shadow)
    cv.drawRRect(rr, fill(CARD))
    cv.drawRRect(rr, stroke(RULE, 1))


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


SLOT_FIELDS = {  # hud fields; digits are tabular so the longest string fixes the width
    "t": lambda f: f"{f['t']:,.0f} s",
    "active": lambda f: f"{f['active']}",
    "spare": lambda f: f"{f['spare']}",
    "c2": lambda f: f"{f['c2']}",
    "c3": lambda f: f"{f['c3_missed']}/{f['c3_alarms']}",
    "cov": lambda f: f"{f['cov']:.1f}%",
    "roll": lambda f: f"{f['roll']:.1f}%",
}


class Run:
    def __init__(self, path):
        d = json.loads(Path(path).read_text())
        self.d = d
        fr = d["frames"]
        self.n = len(fr)
        self.t0 = fr[0]["t"]
        self.fresh = np.array([f["fresh"] for f in fr], dtype=np.float32)
        self.ids = [u[0] for u in fr[0]["uavs"]]
        self.uav = np.array([[u[1:] for u in f["uavs"]] for f in fr], dtype=np.float32)  # tick, uav, (x y soc)
        self.fr = fr
        self.slots = {k: max((fmt(f) for f in fr), key=len) for k, fmt in SLOT_FIELDS.items()}
        self.failure = d["failure"]
        self.has_failure = self.failure is not None
        self.cells = d["cells"]
        self.depot = d["depot"]
        self.title, self.meta = d["title"], d["meta"]
        half = d["cell_size"] / 2
        self.rects = [
            skia.Rect.MakeLTRB(*px(x - half, y + half), *px(x + half, y - half)).makeInset(1.3, 1.3) for x, y in self.cells
        ]

    def at(self, pos):
        """State interpolated at tick position ``pos``."""
        pos = min(max(pos, 0.0), self.n - 1.0)
        i = min(int(pos), self.n - 2)
        f = pos - i
        fresh = self.fresh[i] + (self.fresh[i + 1] - self.fresh[i]) * f
        a, b = self.uav[i], self.uav[i + 1]
        jump = np.hypot(b[:, 0] - a[:, 0], b[:, 1] - a[:, 1]) > SNAP
        u = a + (b - a) * f
        u[jump] = a[jump]
        return i, fresh, u


def canvas_h(run):
    return round(PLOT[1] + PLOT_H + 34 + PAD + (BANNER_H if run.has_failure else 0))


def draw_axes(cv):
    left, top, w = PLOT
    cv.drawRRect(skia.RRect.MakeRectXY(skia.Rect.MakeXYWH(left, top, w, PLOT_H), 8, 8), fill("#fafbfc"))
    for v in range(-500, 501, 100):
        x = px(v, 0)[0]
        cv.drawLine(x, top, x, top + PLOT_H, stroke("#eceef2", 1))
        text(cv, f"{v}".replace("-", "−"), x, top + PLOT_H + 21, "tick", MUTED, "center", tab=True)
    for v in range(0, 501, 100):
        y = px(0, v)[1]
        cv.drawLine(left, y, left + w, y, stroke("#eceef2", 1))
        text(cv, f"{v}", left - 9, y + 4, "tick", MUTED, "right", tab=True)


def marker(cv, x, y, r, soc_color):
    """UAV triangle with a white halo."""
    cv.drawPath(triangle(x, y, r), stroke("#ffffff", 3.6))
    cv.drawPath(triangle(x, y, r), fill(soc_color))
    cv.drawPath(triangle(x, y, r), stroke(INK, 1.3))


def chip_box(cv, x, y, w, h, r, alpha=240):
    rr = skia.RRect.MakeRectXY(skia.Rect.MakeXYWH(x, y, w, h), r, r)
    cv.drawRRect(rr, fill(CARD, alpha))
    cv.drawRRect(rr, stroke("#c8ced8", 1))


def draw_uav(cv, x, y, soc, uid):
    marker(cv, x, y, 8, ramp(soc))
    chip_box(cv, x - 33, y + 11, 66, 18, 5, 240)
    text(cv, uid, x - 28, y + 24, "chip_id", INK)
    text(cv, f"{soc * 100:.0f}%", x + 29, y + 24, "chip_soc", MUTED, "right", tab=True)


def draw_parked(cv, run, parked):
    """Chip grid of the UAVs parked on the depot, where raw positions coincide."""
    dy = px(*run.depot)[1]
    cols, cw, ch = 3, 98, 24
    for k, (uid, soc) in enumerate(parked):
        c, r = k % cols, k // cols
        x, y = PLOT[0] + 10 + c * (cw + 5), dy - 38 - r * (ch + 5) - ch
        chip_box(cv, x, y, cw, ch, 7)
        marker(cv, x + 14, y + 12.5, 6, ramp(soc))
        text(cv, uid, x + 28, y + 16.5, "chip_id", INK)
        text(cv, f"{soc * 100:.0f}%", x + cw - 9, y + 16.5, "chip_soc", MUTED, "right", tab=True)


def draw_hud(cv, run, st):
    """Four-line status box inside the plot."""
    d, sl = run.d, run.slots
    rows = [  # (label, widest value, value); the widest value keeps the box from shifting
        [("Time:", sl["t"], f"{st['t']:,.0f} s")],
        [("Active:", sl["active"], f"{st['active']}"), ("Spare:", sl["spare"], f"{st['spare']}"), ("Return @", f"{d['return_pct']}%", f"{d['return_pct']}%")],
        [("C2 Viol:", sl["c2"], f"{st['c2']}"), ("C3 Miss:", sl["c3"], f"{st['c3_missed']}/{st['c3_alarms']}"), ("Loop:", f"{d['loop']} s", f"{d['loop']} s")],
        [("Coverage:", sl["cov"], f"{st['cov']:.1f}%"), (f"Rolling ({d['rolling_window']}s):", sl["roll"], f"{st['roll']:.1f}%")],
    ]
    sep, rh, pad = " | ", 25, 14
    sw = measure("label", sep)
    widest = max(sum(measure("label", a) + 6 + measure("value", w, True) for a, w, _ in row) + sw * (len(row) - 1) for row in rows)
    x0, y0 = PLOT[0] + 12, PLOT[1] + 12
    chip_box(cv, x0, y0, widest + 2 * pad, rh * len(rows) + 14, 9, 246)
    for r, row in enumerate(rows):
        y = y0 + 7 + r * rh + 18
        x = x0 + pad
        for k, (label, widest_value, value) in enumerate(row):
            if k:
                x += text(cv, sep, x, y, "label", "#b4bbc6")
            lw = text(cv, label, x, y, "label", MUTED)
            text(cv, value, x + lw + 6, y, "value", INK, tab=True)
            x += lw + 6 + measure("value", widest_value, True)


def draw_legend(cv):
    x1 = PLOT[0] + PLOT[2] - 12
    y = PLOT[1] + 12
    w = 24 + measure("label", "Depot") + 26 + 26 + measure("label", "UAVs") + 14
    chip_box(cv, x1 - w, y, w, 32, 9, 246)
    x = x1 - w + 14
    cv.drawPath(star(x + 7, y + 16, 8), fill(DEPOT))
    x += 22
    x += text(cv, "Depot", x, y + 21, "label", INK) + 22
    marker(cv, x + 7, y + 16.5, 7, "#cbd2dc")
    text(cv, "UAVs", x + 22, y + 21, "label", INK)


def draw_colorbar(cv):
    x = PLOT[0] + PLOT[2] + 20
    y, h = PLOT[1], PLOT_H
    bar = skia.RRect.MakeRectXY(skia.Rect.MakeXYWH(x, y, 14, h), 7, 7)
    if LEVELS:
        cv.save()
        cv.clipRRect(bar, doAntiAlias=True)
        for k in range(LEVELS):
            cv.drawRect(skia.Rect.MakeXYWH(x, y + h * (1 - (k + 1) / LEVELS) - 0.5, 14, h / LEVELS + 1), fill(ramp(k / (LEVELS - 1))))
        cv.restore()
    else:
        shader = skia.GradientShader.MakeLinear(
            [(0, y), (0, y + h)], [skia.Color(*hexrgb(c), 255) for _, c in reversed(RAMP)], [1 - p for p, _ in reversed(RAMP)]
        )
        cv.drawRRect(bar, skia.Paint(AntiAlias=True, Shader=shader))
    for k in range(6):
        v = k / 5
        text(cv, f"{v:.1f}", x + 24, y + h * (1 - v) + 4, "tick", MUTED, tab=True)
    cv.save()
    cv.translate(x + 70, y + h / 2)
    cv.rotate(-90)
    text(cv, "UAV SoC / Cell Freshness", 0, 0, "small", MUTED, "center")
    cv.restore()


def banner_text(f, t):
    """Status line of the failure response at time ``t``; None before the failure."""
    if t < f["t_fail"]:
        return None
    route = f["route"]
    if t < f["t_fail"] + LOST_SHOWN:
        return f"UAV {f['uav']} lost at {f['t_fail']:,.0f} s"
    if f["t_handover"] is not None and t >= f["t_handover"]:
        if f["t_restored"] is not None and t >= f["t_restored"]:
            return "Coverage restored"
        return f"Spare UAV {f['spare']} now covering route {route}"
    if f["t_launch"] is not None and t >= f["t_launch"]:
        return f"Spare UAV {f['spare']} flying to take over route {route}"
    return "Failure detected  ·  launching spare"


def draw_banner(cv, run, st, h):
    s = banner_text(run.failure, st["t"])
    if s is None:
        return
    w = measure("small", s) + 36
    x = PLOT[0] + PLOT[2] / 2 - w / 2
    y = h - PAD - BANNER_H + 4
    rr = skia.RRect.MakeRectXY(skia.Rect.MakeXYWH(x, y, w, 32), 16, 16)
    cv.drawRRect(rr, fill(FAIL, 16))
    cv.drawRRect(rr, stroke(FAIL, 1.1, 90))
    text(cv, s, x + 18, y + 21, "small", FAIL)


def draw_frame(cv, run, pos, h):
    i, fresh, uav = run.at(pos)
    st = run.fr[i]
    cv.clear(color(BG))
    rr = skia.RRect.MakeRectXY(skia.Rect.MakeXYWH(PAD / 2, PAD / 2, W - PAD, h - PAD), 16, 16)
    shadow = fill("#101828", 10)
    shadow.setMaskFilter(skia.MaskFilter.MakeBlur(skia.kNormal_BlurStyle, 5))
    cv.drawRRect(rr.makeOffset(0, 2), shadow)
    cv.drawRRect(rr, fill(CARD))
    cv.drawRRect(rr, stroke(RULE, 1))

    tw = text(cv, run.title, PLOT[0], 42, "title", INK)
    text(cv, run.meta, PLOT[0] + tw + 16, 42, "meta", MUTED)

    draw_axes(cv)
    for r, v in zip(run.rects, fresh):
        cv.drawRRect(skia.RRect.MakeRectXY(r, 3.5, 3.5), fill(ramp(float(v))))

    dx, dy = px(*run.depot)
    cv.drawPath(star(dx, dy, 12), fill(DEPOT))
    text(cv, "Depot", dx, dy - 22, "small", INK, "center")

    parked, flying = [], []
    for uid, (x, y, soc) in zip(run.ids, uav):
        if math.hypot(x - run.depot[0], y - run.depot[1]) < 1.0:
            parked.append((uid, float(soc)))
        else:
            flying.append((uid, x, y, float(soc)))
    draw_parked(cv, run, sorted(parked))
    for uid, x, y, soc in sorted(flying, key=lambda u: -u[2]):
        draw_uav(cv, *px(x, y), soc, uid)

    draw_hud(cv, run, st)
    draw_legend(cv)
    draw_colorbar(cv)
    if run.has_failure:
        draw_banner(cv, run, st, h)


class Encoder:
    def __init__(self, path, w, h, fps, crf):
        head = ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-"]
        if str(path).endswith(".gif"):  # straight from rgb so no chroma subsampling reaches the palette
            tail = ["-vf", "split[a][b];[a]palettegen=max_colors=256:stats_mode=full[p];[b][p]paletteuse=dither=none:diff_mode=rectangle",
                    "-loop", "0", str(path)]
        else:
            tail = ["-c:v", "libx264", "-crf", str(crf), "-preset", "medium", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)]
        self.p = subprocess.Popen(head + tail, stdin=subprocess.PIPE)

    def write(self, b):
        self.p.stdin.write(b)

    def close(self):
        self.p.stdin.close()
        self.p.wait()


def render(run, positions, out, fps, crf=18, limit=None, scale=1.0):
    w, h0 = W, canvas_h(run)
    w, h = round(w * scale) // 2 * 2, round(h0 * scale) // 2 * 2  # yuv420p needs even sides
    surface = skia.Surface(w, h)
    cv = surface.getCanvas()
    enc = Encoder(out, w, h, fps, crf)
    t0 = time.time()
    for n, pos in enumerate(positions[:limit]):
        cv.save()
        cv.scale(w / W, h / h0)
        draw_frame(cv, run, pos, h0)
        cv.restore()
        arr = surface.makeImageSnapshot().toarray(colorType=skia.kRGBA_8888_ColorType)
        enc.write(np.ascontiguousarray(arr[..., :3]).tobytes())
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
    ap.add_argument("--limit", type=int)
    ap.add_argument("--stepped", action="store_true")
    ap.add_argument("--scale", type=float, default=1.6, help="size relative to the 1200 px layout")
    ap.add_argument("--palette", choices=sorted(PALETTES), default="D")
    ap.add_argument("--at", type=float, default=0.0, help="tick position for --mode still")
    a = ap.parse_args()
    global LEVELS
    set_palette(a.palette)
    run = Run(a.data)
    defaults = {"video": Path(f"assets/videos/{a.data.stem}.mp4"), "gif": Path("assets/demo.gif"), "still": Path(f"results/{a.data.stem}.png")}
    out = a.out or defaults[a.mode]
    out.parent.mkdir(parents=True, exist_ok=True)
    if a.mode == "still":
        LEVELS = 32 if a.stepped else None
        h = canvas_h(run)
        surface = skia.Surface(round(W * a.scale), round(h * a.scale))
        cv = surface.getCanvas()
        cv.scale(a.scale, a.scale)
        draw_frame(cv, run, a.at, h)
        surface.makeImageSnapshot().save(str(out), skia.kPNG)
    elif a.mode == "video":
        render(run, video_positions(run, a.fps or 30), out, a.fps or 30, limit=a.limit, scale=a.scale)
    else:
        LEVELS = 32
        render(run, gif_positions(run, a.gif_frames), out, a.fps or 12, crf=10)


if __name__ == "__main__":
    main()
