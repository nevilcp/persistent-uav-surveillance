# Simulation media

All demonstrations use `scripts/export_frames.py` and `scripts/render_animation.py`,
the same JSON → Skia → FFmpeg pipeline as the original nominal animation.
Run commands from the repository root with the `video` extra installed and
`ffmpeg` on PATH. Install the Inter fonts to match the original typography.

## Choset decomposition and round-robin demonstrations

Every current fixed scenario uses Choset decomposition followed by round-robin,
120 UAVs, and depots 500 m from the nearest outer region boundary. Rendering is
sequential: export a scenario, render its MP4, render its GIF, then continue.
Older files labelled shared-loop are historical and do not show the current planner.

| Study run | Scenario | GIF preview | Full-mission video |
|---|---|---|---|
| R0 | Straight-distance return control | [GIF](multi_depot/straight-return-control-surveillance.gif) | [MP4](multi_depot/videos/straight-return-control.mp4) |
| R1 west | West depot alone | [GIF](multi_depot/single-depot-west-surveillance.gif) | [MP4](multi_depot/videos/single-depot-west.mp4) |
| R1 east | East depot alone | [GIF](multi_depot/single-depot-east-surveillance.gif) | [MP4](multi_depot/videos/single-depot-east.mp4) |
| R1 north | North depot alone | [GIF](multi_depot/single-depot-north-surveillance.gif) | [MP4](multi_depot/videos/single-depot-north.mp4) |
| R2 | Three depots, no obstacles | [GIF](multi_depot/multiple-depots-surveillance.gif) | [MP4](multi_depot/videos/multiple-depots.mp4) |
| R3 | Three depots and no-fly zones | [GIF](multi_depot/no-fly-zones-choset-round-robin-surveillance.gif) | [MP4](multi_depot/videos/no-fly-zones-choset-round-robin.mp4) |
| R4 | Single failure at 1800 s | [GIF](multi_depot/no-fly-zones-single-failure-surveillance.gif) | [MP4](multi_depot/videos/no-fly-zones-single-failure.mp4) |
| R5 | Four failures at 1800, 2100, 2400 and 2700 s | [GIF](multi_depot/no-fly-zones-repeated-failures-surveillance.gif) | [MP4](multi_depot/videos/no-fly-zones-repeated-failures.mp4) |
| R6 | North depot closed at 1800 s | [GIF](multi_depot/no-fly-zones-depot-closure-surveillance.gif) | [MP4](multi_depot/videos/no-fly-zones-depot-closure.mp4) |

```bash
python scripts/render_port_animations.py all
python scripts/render_port_animations.py R3 R6
python scripts/render_port_animations.py all --mode gif
```

Study MP4s use H.264, 24 fps and 60× simulation speed, giving approximately
80 seconds per video. They show the map and seven STL monitor timelines side by side. GIFs use
144 frames at 12 fps, 1920 px wide, a stepped 32-level freshness ramp and an
undithered palette. Both span 1–4800 s; GIFs skip intermediate states and are
12-second overviews. Videos interpolate movement between 1 Hz samples.

Tiles range from red (overdue) to green (fresh). White triangles mark UAVs in
flight, dark hatched blocks mark no-fly zones, and red stars mark depots.
Rust-colored crosses beside a depot mark failed UAVs, and outlined depot stars
mark closures. C1 traces instantaneous and rolling coverage. C2–C7 show battery
reserve, spare launch, recovery, return energy, depot queue margin and no-fly
zone clearance. Percentage-point margins use `pp`. Labels below the shared video
time axis mark recorded failures and closures. GIFs show the map alone.
Both overlays report how many routes due by that time lack an on-route UAV,
along with ready spare and failed UAV counts. Videos also show the number of
UAVs on route, and GIFs show the percentage of fresh cells.
The media show measured behavior, including coverage gaps and queues; they do
not imply that all monitor clauses passed. See [the results](../docs/port-study.md).

## Naming and storage

Python files and frame JSON use `snake_case`; media use lowercase `kebab-case`,
matching the baseline assets. GIFs end in `-surveillance.gif`.

| Directory | Contents | Example |
|---|---|---|
| `scripts/` | Exporter, renderer and extended batch command | `render_port_animations.py` |
| `results/frames/` | Regenerable simulation state | `no_fly_zones_choset_round_robin.json` |
| `results/frames/` | Build manifest with scenario IDs, sample range and output paths | `port_animations_manifest.json` |
| `assets/multi_depot/` | Current GIF previews | `no-fly-zones-choset-round-robin-surveillance.gif` |
| `assets/multi_depot/videos/` | Current MP4 videos | `no-fly-zones-choset-round-robin.mp4` |

`results/` and MP4s follow the project's existing `.gitignore` policy: they
are generated locally and are not included in Git. Regenerate the videos after
cloning; the GIF previews and this catalogue are versioned. The manifest
describes the scenarios and formats requested by the latest batch command.

## Original demonstrations

The [nominal GIF](single_depot/nominal-surveillance.gif) previews the full run
in 12 seconds. Baseline MP4s add all seven STL timelines to the compact map.
Baseline MP4 filenames are `under-provisioned.mp4`, `nominal.mp4`,
`over-provisioned.mp4`, `long-horizon.mp4` and `single-failure.mp4` under `single_depot/videos/`.
These commands regenerate the nominal video and GIF at their catalogue paths.

```bash
python scripts/export_frames.py nominal
python scripts/render_animation.py results/frames/nominal.json assets/single_depot/videos/nominal.mp4
python scripts/render_animation.py results/frames/nominal.json assets/single_depot/nominal-surveillance.gif --mode gif
```
