# Simulation media

All demonstrations use `scripts/export_frames.py` and `scripts/render_animation.py`,
the same JSON → Skia → FFmpeg pipeline as the original nominal animation.
Run commands from the repository root with the `video` extra installed and
`ffmpeg` on PATH. Install the Inter fonts to match the original typography.

## Choset decomposition and round-robin demonstrations

The fixed R0–R6 scenarios use Choset decomposition followed by round-robin,
120 UAVs, and depots 500 m from the nearest outer region boundary. Rendering is
sequential: export a scenario, render its MP4, render its GIF, then continue.
Older files labelled shared-loop are historical and do not show the current planner.

| Study run | Scenario | GIF preview | Full-mission video |
|---|---|---|---|
| R0 | Straight-line dispatch | [GIF](multi_depot/r0-straight-line-dispatch.gif) | [MP4](multi_depot/videos/r0-straight-line-dispatch.mp4) |
| R1 west | West depot alone | [GIF](multi_depot/r1-west-depot.gif) | [MP4](multi_depot/videos/r1-west-depot.mp4) |
| R1 east | East depot alone | [GIF](multi_depot/r1-east-depot.gif) | [MP4](multi_depot/videos/r1-east-depot.mp4) |
| R1 north | North depot alone | [GIF](multi_depot/r1-north-depot.gif) | [MP4](multi_depot/videos/r1-north-depot.mp4) |
| R2 | No NFZs | [GIF](multi_depot/r2-no-nfzs.gif) | [MP4](multi_depot/videos/r2-no-nfzs.mp4) |
| R3 | Nominal | [GIF](multi_depot/r3-nominal.gif) | [MP4](multi_depot/videos/r3-nominal.mp4) |
| R4 | Single failure at 1800 s | [GIF](multi_depot/r4-single-failure.gif) | [MP4](multi_depot/videos/r4-single-failure.mp4) |
| R5 | Four failures at 1800, 2100, 2400 and 2700 s | [GIF](multi_depot/r5-repeated-failures.gif) | [MP4](multi_depot/videos/r5-repeated-failures.mp4) |
| R6 | North depot closed at 1800 s | [GIF](multi_depot/r6-depot-closure.gif) | [MP4](multi_depot/videos/r6-depot-closure.mp4) |

The report also compares three variants of nominal R3.

| R3 variant | UAVs | Simulation duration | GIF preview | Full-mission video |
|---|---:|---:|---|---|
| Under-Provisioned | 90 | 4800 s | [GIF](multi_depot/r3-under-provisioned.gif) | [MP4](multi_depot/videos/r3-under-provisioned.mp4) |
| Over-Provisioned | 150 | 4800 s | [GIF](multi_depot/r3-over-provisioned.gif) | [MP4](multi_depot/videos/r3-over-provisioned.mp4) |
| Long Horizon | 120 | 9600 s | [GIF](multi_depot/r3-long-horizon.gif) | [MP4](multi_depot/videos/r3-long-horizon.mp4) |

```bash
python scripts/render_port_animations.py all
python scripts/render_port_animations.py R3 R6
python scripts/render_port_animations.py R3_under_provisioned R3_over_provisioned R3_long_horizon
python scripts/render_port_animations.py all --mode gif
```

Study MP4s use H.264, 24 fps and 60× simulation speed, giving approximately
80 seconds per video. They show the map and seven STL monitor timelines side by side
at 1920 × 1080. The R3 long-horizon video runs for 160 seconds at the same
speed. The square, 1080 × 1080 GIFs show the map and mission summary with a
stepped 32-level freshness ramp and an undithered palette. GIFs sample the
full run in 144 frames at 12 fps, while videos interpolate movement between
1 Hz samples.

Tiles range from red (overdue) to green (fresh). White triangles mark UAVs in
flight, dark hatched blocks mark no-fly zones, and black stars mark depots.
Rust-colored crosses beside a depot mark failed UAVs, and outlined depot stars
mark closures. C1 traces instantaneous coverage. C2–C7 show battery
reserve, spare launch, recovery, return energy, depot queue margin and no-fly
zone clearance. Percentage-point margins use `pp`. Labels below the shared video
time axis appear when failures and closures occur. Measured traces extend to
the playhead, while deadline references span the plot. GIFs show the map alone.
Both overlays report how many routes due by that time lack an on-route UAV,
along with ready spare and failed UAV counts. Videos also show the number of
UAVs on route, and GIFs show the percentage of fresh cells.
Coverage gaps and queues are visible in the animations.
[Scenario results](../README.md#fixed-scenarios-and-nominal-variants)

## Naming and storage

Python files and frame JSON use `snake_case`. GIF and MP4 pairs share a
lowercase, hyphenated filename stem with the `baseline` prefix or the study
run ID.

| Directory | Contents | Example |
|---|---|---|
| `scripts/` | Exporter, renderer and extended batch command | `render_port_animations.py` |
| `results/frames/` | Regenerable simulation state | `no_fly_zones_choset_round_robin.json` |
| `results/frames/` | Build manifest with scenario IDs, sample range and output paths | `port_animations_manifest.json` |
| `assets/multi_depot/` | Current GIF previews | `r3-nominal.gif` |
| `assets/multi_depot/videos/` | Current MP4 videos | `r3-nominal.mp4` |

`results/` and MP4s follow the project's existing `.gitignore` policy: they
are generated locally and are not included in Git. Regenerate the videos after
cloning; the GIF previews and this catalogue are versioned. The manifest
describes the scenarios and formats requested by the latest batch command.

## Original single-depot demonstrations

Each GIF previews the full run in 12 seconds. The MP4s show the baseline map
alongside the STL dashboard.

| Run | GIF preview | Full-mission video |
|---|---|---|
| Under-Provisioned | [GIF](single_depot/baseline-under-provisioned.gif) | [MP4](single_depot/videos/baseline-under-provisioned.mp4) |
| Nominal | [GIF](single_depot/baseline-nominal.gif) | [MP4](single_depot/videos/baseline-nominal.mp4) |
| Over-Provisioned | [GIF](single_depot/baseline-over-provisioned.gif) | [MP4](single_depot/videos/baseline-over-provisioned.mp4) |
| Long Horizon | [GIF](single_depot/baseline-long-horizon.gif) | [MP4](single_depot/videos/baseline-long-horizon.mp4) |
| Single Failure | [GIF](single_depot/baseline-single-failure.gif) | [MP4](single_depot/videos/baseline-single-failure.mp4) |

These commands regenerate the video and GIF for each baseline run.

```bash
python scripts/export_frames.py all
for run in under_provisioned nominal over_provisioned long_horizon single_failure; do
    python scripts/render_animation.py "results/frames/$run.json"
    python scripts/render_animation.py "results/frames/$run.json" --mode gif
done
```
