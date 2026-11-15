# Port study: Choset decomposition plus round-robin

Recorded on 2026-11-15 with Python 3.14.4 from the current working tree.
The shared-loop planner is removed. All runs below use raster Choset
connectivity-event decomposition, back-and-forth region sweeps, sector-local
round-robin assignment and NFZ-safe waypoint connections.

The region is 1200 m × 1200 m with 40 m cells, 830 free cells (900 in R2),
6 m/s speed, 30 m sensing radius, Θ = 180 s, 1890 s usable endurance,
60 s turnaround and two service slots per depot. The initial fleet is held at
120 UAVs: 87 flying, 27 rotation and 6 spare. Depots are west (−500, 600),
east (1700, 600) and north (600, 1700), each exactly 500 m from its nearest
outer boundary point. Sectors contain 309, 290 and 231 cells and initially
have 32/10/2, 30/9/2 and 25/8/2 flying/rotation/spare drones respectively.

Each run lasts 4800 s at a 1 s step. Steady-state means use t > 1200 s.
The pass flag covers sampled C2/C5/C6/C7 only; it excludes freshness, launch
latency and recovery. The UAV column is initial inventory, including airframes later failed or
unavailable at a closed depot. C5–C7 columns count violating monitor steps, not drones.
The executable calls NFZ clearance C7; the design report calls it C8.
The maximum route length includes the closing leg and NFZ detours, but not
ferry legs or temporary failure-bridging insertions.

R0 changes only the return-distance estimate to straight-line distance;
physical flight paths still avoid NFZs. R1 tests each depot alone with the same
fleet and all six service slots. R2 removes NFZs; R3 is the main run;
R4 adds one failure at 1800 s; R5 adds failures at 1800, 2100, 2400 and 2700 s;
R6 closes the north depot at 1800 s. Failures target flying routes served by
the east depot. The best single depot by steady coverage is west (79.47%). Launch schedules
are generated independently per depot, so a single depot has a longer initial
deployment schedule; its deployment can extend beyond 1200 s. This comparison
includes scheduling effects and does not isolate ferry geometry alone.

## Fixed scenarios

| Run | UAVs (fly+rot+spare) | Max route (km) | Steady cov. (%) | Longest wait (s) | C2 min margin | C3 late / alarms | C5 | C6 | C7 | Transit share | C2, C5-C7 met |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| R0 | 120 (87+27+6) | 4.8 | 90.0 | 1072 | 0.019 | 54 / 174 | 0 | 520 | 0 | 0.16 | no |
| R1_east | 120 (87+27+6) | 8.6 | 78.7 | 1606 | 0.019 | 0 / 172 | 0 | 0 | 0 | 0.22 | yes |
| R1_north | 120 (87+27+6) | 8.6 | 77.9 | 1653 | 0.019 | 0 / 174 | 0 | 0 | 0 | 0.27 | yes |
| R1_west | 120 (87+27+6) | 8.6 | 79.5 | 1614 | 0.019 | 0 / 173 | 0 | 0 | 0 | 0.22 | yes |
| R2 | 120 (87+27+6) | 5.1 | 90.2 | 1116 | 0.019 | 47 / 174 | 0 | 600 | 0 | 0.16 | no |
| R3 | 120 (87+27+6) | 4.8 | 90.0 | 1015 | 0.019 | 47 / 174 | 0 | 487 | 0 | 0.16 | no |
| R4 | 120 (87+27+6) | 4.8 | 90.0 | 1015 | 0.019 | 47 / 175 | 0 | 487 | 0 | 0.16 | no |
| R5 | 120 (87+27+6) | 4.8 | 90.0 | 1015 | 0.019 | 47 / 178 | 0 | 487 | 0 | 0.16 | no |
| R6 | 120 (87+27+6) | 4.8 | 89.2 | 1289 | -0.059 | 106 / 174 | 219 | 2069 | 0 | 0.17 | no |

Longest wait: largest cell age after the first 1200 s and after every cell has been seen once. Coverage: mean over the steady state.

**R4**: longest launch latency 223 s.
- breakdown at 1800 s (UAV 100): recovery not reached (limit 300 s, missed)
**R5**: longest launch latency 223 s.
- breakdown at 1800 s (UAV 100): recovery not reached (limit 300 s, missed)
- breakdown at 2100 s (UAV 42): recovery not reached (limit 300 s, missed)
- breakdown at 2400 s (UAV 46): recovery not reached (limit 300 s, missed)
- breakdown at 2700 s (UAV 45): recovery not reached (limit 300 s, missed)
**R6**: longest launch latency 733 s.
- t = 1800 s closure: base 2
- t = 1800 s reassign: 25 routes moved from base 2

## Interpretation

The main run reaches 89.99% steady coverage, with no sampled battery-floor,
return-feasibility or NFZ-clearance violations. It has 487 queue-limit violating
steps and 47 late launch alarms out of 174; its worst cell age is 1015 s against
Θ = 180 s. The two-slot service configuration and the routing/dispatch combination
do not meet all requirements. These outcomes are reported without tuning capacity
or the freshness threshold after observing them.

In R6, closure rehomes 25 routes and redirects incoming returns, but C2 is
violated at 124 sampled steps and C5 at 219. The minimum battery margin is
−0.058666 relative to the 0.1 reserve. This is a reserve violation, not a claim
that battery charge became negative. Depot-outage resilience is not established.

The per-breakdown table measures restoration of **global** max cell age ≤ Θ.
Nominal coverage already leaves stale cells, so this metric cannot isolate the
causal recovery from a failure. The single-failure manager separately reports
all orphan cells re-observed in 128 s, with spare handover at 1993 s; the last
R5 failure's orphan cells are re-observed in 289 s. These local measures can pass
while global freshness restoration remains unmet. Unanswered launch alarms count
elapsed latency through the end of the run, a lower bound on eventual latency.

## Random maps

All 20 attempted maps use the same 120-drone planner and depot coordinates,
with NFZ map seeds 0–19 covering about 8% of the region. Motion on a fixed map
is deterministic. These are layout variations, not repeated stochastic motion
trials. Statistics exclude infeasible runs; pass and infeasibility counts use
all 20 attempted layouts. No confidence intervals or claims of generalisation
to unseen maps are made.

20 maps, movement deterministic; map seeds 0–19.

| Metric | Choset + round-robin |
|---|---:|
| UAVs (min / mean / max) | 120.0 / 120.0 / 120.0 |
| Steady coverage % (min / mean / max) | 86.0 / 88.6 / 91.3 |
| Longest wait s (min / mean / max) | 800.0 / 1176.7 / 1727.0 |
| Maps meeting C2, C5-C7 | 0 |
| Maps infeasible | 0 |

## Reproduce and artifacts

```bash
python scripts/run_port_study.py --out results/port-choset-round-robin
python scripts/run_scenarios.py --out results/baseline-choset-round-robin
python scripts/render_port_animations.py all
```

Fixed-scenario `metrics.csv`, `events.csv`, `routes.csv`, `parameters.json` and
`summary.json`, figures, the random-map CSV and a source fingerprint manifest
are retained locally under `results/port-choset-round-robin/`. `results/` and MP4s
are ignored by Git; preserve them with the working-tree source when archiving.
The five baseline scenarios were also rerun and reproduce the prior baseline
figures because the obstacle-free sweep order is unchanged. The test suite has
146 passing tests after removing the obsolete shared-loop tests and adding
Choset decomposition, sector assignment, closure and exporter coverage.

Videos and GIFs are listed in [the media catalogue](../assets/README.md).
Historical 40 m-setback/shared-loop figures are superseded by this study;
[the historical audit](implementation-status.md) retains their original context.
