import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir)))

import matplotlib  # noqa: E402

matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt  # noqa: E402

from uav_surveil.config import load_scenario  # noqa: E402


def draw_layout(ax, cfg):
    # Area and origin
    area_w = cfg.mission.area_width
    area_l = cfg.mission.area_length
    depot_x = cfg.mission.depot_x
    depot_y = cfg.mission.depot_y
    origin_x, origin_y = cfg.grid.origin

    xmin = min(depot_x, origin_x) - 50
    xmax = max(depot_x, origin_x + area_w) + 50
    ymin = min(depot_y, origin_y) - 50
    ymax = max(depot_y, origin_y + area_l) + 50

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal")

    # Grid (no cells data; draw a light frame rectangle)
    rect = plt.Rectangle(
        (origin_x, origin_y), area_w, area_l, ec="gray", fc="white", lw=0.6, alpha=1.0
    )
    ax.add_patch(rect)

    # Depot
    ax.plot([depot_x], [depot_y], marker="*", color="red", markersize=15, label="Depot")
    ax.legend(loc="upper right")

    title_str = (
        f"Continuous Patrol – {cfg.config_name} | "
        f"Algo: {cfg.optimization.route_algorithm} | Fleet (layout preview)"
    )
    ax.set_title(title_str)

    return xmin, ymin


def save_preview(out_path: str, banner_text: str) -> None:
    cfg = load_scenario("baseline")
    os.makedirs("results", exist_ok=True)

    fig, ax = plt.subplots(figsize=(8, 8))
    _xmin, _ymin = draw_layout(ax, cfg)

    # Failure banner at bottom center (same style as visualizer)
    fig.text(
        0.5,
        0.02,
        banner_text,
        va="bottom",
        ha="center",
        fontsize=8,
        color="purple",
        bbox={"boxstyle": "round,pad=0.2", "fc": "white", "alpha": 0.7},
    )

    plt.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def main():
    # Two quick variants: just-after-failure, and after-handover
    base = "results/preview_failure"
    save_preview(base + "_banner.png", "Fail t=1800s  |  contingency launched")
    save_preview(
        base + "_handover.png",
        "Fail t=1800s  |  contingency launched  |  handover complete",
    )
    print("Saved:")
    print("  ", base + "_banner.png")
    print("  ", base + "_handover.png")


if __name__ == "__main__":
    main()
