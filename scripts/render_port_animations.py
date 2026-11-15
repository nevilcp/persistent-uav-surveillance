"""Export and render the extended scenarios using the baseline animation pipeline.

Run from any directory with the project's Python environment. JSON and the build
manifest go in results/frames/, GIFs in assets/multi_depot/, and MP4s in assets/multi_depot/videos/.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

from export_frames import dump

ROOT = Path(__file__).resolve().parents[1]
SCENES = {
    "R0": ("straight_return_control", "Return Metric Control", "Straight return metric"),
    "R1_west": ("single_depot_west", "Single Depot: West", "Choset + Round-Robin"),
    "R1_east": ("single_depot_east", "Single Depot: East", "Choset + Round-Robin"),
    "R1_north": ("single_depot_north", "Single Depot: North", "Choset + Round-Robin"),
    "R2": ("multiple_depots", "Multiple Depots", "No obstacles"),
    "R3": ("no_fly_zones_choset_round_robin", "No-Fly Zones", "Choset + Round-Robin"),
    "R4": ("no_fly_zones_single_failure", "Single Failure", "Choset + Round-Robin"),
    "R5": ("no_fly_zones_repeated_failures", "Repeated Failures", "Choset + Round-Robin"),
    "R6": ("no_fly_zones_depot_closure", "Depot Closure", "Choset + Round-Robin"),
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenarios", nargs="*", choices=[*SCENES, "all"], default=["all"])
    parser.add_argument("--mode", choices=["both", "video", "gif"], default="both")
    parser.add_argument("--video-speed", type=float, default=60.0)
    parser.add_argument("--video-fps", type=float, default=24.0)
    args = parser.parse_args()
    names = list(SCENES) if "all" in args.scenarios else args.scenarios
    manifest_path = ROOT / "results" / "frames" / "port_animations_manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else []
    manifest = [entry for entry in manifest if entry["scenario"] not in names]
    for name in names:
        stem, title, description = SCENES[name]
        data = dump(name)
        data["title"] = title
        depot_count = len(data['depots'])
        depot_label = "Depot" if depot_count == 1 else "Depots"
        data["meta"] = f"{name.replace('_', ' ').title()}  |  {description.title()}  |  {depot_count} {depot_label}  |  {len(data['frames'][0]['uavs'])} UAVs"
        frames = ROOT / "results" / "frames" / f"{stem}.json"
        frames.parent.mkdir(parents=True, exist_ok=True)
        frames.write_text(json.dumps(data, separators=(",", ":")))
        media_stem = stem.replace("_", "-")
        outputs = {
            "video": ROOT / "assets" / "multi_depot" / "videos" / f"{media_stem}.mp4",
            "gif": ROOT / "assets" / "multi_depot" / f"{media_stem}-surveillance.gif",
        }
        for mode, path in outputs.items():
            if args.mode in ("both", mode):
                print(f"{name}: rendering {path.relative_to(ROOT)}", flush=True)
                options = ["--speed", str(args.video_speed), "--fps", str(args.video_fps)] if mode == "video" else []
                subprocess.run([sys.executable, str(ROOT / "scripts" / "render_animation.py"),
                                str(frames), str(path), "--mode", mode, *options], check=True, cwd=ROOT)
                probe = subprocess.run(["ffprobe", "-v", "error", "-show_entries",
                                        "stream=codec_name,width,height,nb_frames:format=duration",
                                        "-of", "json", str(path)], check=True, capture_output=True, text=True)
                info = json.loads(probe.stdout)
                if not info.get("streams") or float(info["format"]["duration"]) <= 0:
                    raise RuntimeError(f"invalid media output: {path}")
                print(f"{name}: verified {mode}, {info['format']['duration']} s", flush=True)
        manifest.append({"scenario": name, "planner": "choset_round_robin",
                         "depot_setback_m": 500, "video_speed": args.video_speed, "video_fps": args.video_fps, "frames": str(frames.relative_to(ROOT)),
                         "samples": len(data["frames"]),
                         "start_s": data["frames"][0]["t"], "end_s": data["frames"][-1]["t"],
                         "outputs": {mode: str(path.relative_to(ROOT)) for mode, path in outputs.items()
                                     if args.mode in ("both", mode)}})
        manifest_path.parent.mkdir(parents=True, exist_ok=True)
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
        print(f"{name}: completed video/GIF outputs", flush=True)


if __name__ == "__main__":
    main()
