# Design notes

How the code is organised, which quantities the design leaves open, where it had
to be interpreted, and what the verification runs showed.

## 1. Module map

| Quantity | Implemented in |
|---|---|
| cell age, fresh cells, coverage | `coverage.py` `CoverageMap` |
| usable endurance T_use | `params.py` `UAVParams.usable_endurance` |
| ferry distance and time | `grid.py` `Grid.ferry_distance`, `routing.py` `ferry_time` |
| loop time, airborne time, endurance condition | `routing.py` `airborne_time`, `check_admissible` |
| fleet-split integer program | `fleet.py` `split_fleet` |
| round-robin on the boustrophedon order | `routing.py` `round_robin_routes`, `grid.py` `boustrophedon_order` |
| in-region distance D_loop | `routing.py` `loop_distance` |
| batch departure times | `schedule.py` `assign_departures` |
| a replacement inherits the route | `uav.py` `UAV.launch` (re-launches the unmodified route) |
| return cost and return threshold | `dispatch.py` `return_cost`, `return_threshold` |
| return-to-base rule | `dispatch.py` `must_return`, `simulation.py` `_dispatch` |
| predicted arrival battery, replacement rule | `dispatch.py` `predicted_arrival_battery`, `needs_replacement` |
| urgent orphan cells | `failure.py` `FailureManager.bridge` |
| least-cost insertion, subject to the endurance condition | `failure.py` `_best_assignee`, `_insertion_cost` |
| the contingency UAV inherits the failed route | `simulation.py` `_serve_alarms` |
| overdue cells, steady-state coverage | `monitor.py` `Monitor.record`, `Monitor.summary` |
| battery margin (C2) | `monitor.py` `Monitor.record` |
| spare-launch latency (C3) | `monitor.py` `Alarm.latency`, `Monitor.summary` |
| recovery time (C4) | `failure.py` `FailureManager.recovery_time` |

## 2. Quantities left open

| Quantity | Value | Rationale |
|---|---|---|
| ρ_min | 0.03 | Gives one contingency UAV for N ≤ 33, hence the 16 + 4 + 1 split at N = 21 |
| c_fly, c_reserve | 1.0, 1.2 | Only c_reserve ≥ c_fly matters; the ratio is the one of the original code |
| σ_buf | 0.02 | About 42 s of flight; covers the one-step discretisation (6 m = 0.0005 of the battery) with a wide margin |
| σ_pre | 0.06 | Must exceed σ_buf; a lead of 0.04 · T_endure = 84 s over the return decision |
| δ_age | 60 s | Warning one third of Θ ahead of the revisit limit |
| ε | 120 s | The C4 verdict depends on it; the recovery time itself does not |
| Sensing footprint | disk of 30 m (0.75 Δ) around the UAV | 30 m is the radius of the original code. A cell is covered when its centroid is inside |
| Depot position | (-500, 0) m | "500 m west of A"; the original code places it at the south-west corner row |
| t_ss | 1200 s | After the last initial launch (630 s) and the first loop |
| Failure | UAV on route 10 at t_f = 1800 s | Chosen so that the failed UAV is mid-sortie and not being replaced |

The surveyed region is the largest block of whole cells inside A: 12 × 12 = 144
cells of 40 m, i.e. 480 m × 480 m. D_ferry is measured to the corner of that block.

## 3. Interpretations

* **Repeated loops.** The loop and endurance conditions describe one depot-to-depot
  loop, whereas the return rules end a sortie when the battery runs low, which only
  happens after several loops. A UAV therefore repeats its loop (including the hop
  from the last waypoint back to the first) until the return rule fires; the
  endurance condition is checked for a single loop.
* **Replacement route.** The replacement starts at the first waypoint of the full
  route, not at the incumbent's position, and is launched when the replacement
  rule first holds. If no rotation UAV is waiting, the alarm stays open and is
  served as soon as one is (oldest first); the incumbent returns regardless.
* **Launch gap.** δ_launch = τ = 60 s (the smallest value allowed) with
  T_batch = 150 s means the last UAV of a batch (180 s) leaves after the next
  batch has started. The depot has no capacity limit: any number of UAVs may be in
  turnaround at once.
* **Alarms (C3).** An alarm is raised when the replacement rule first holds for a
  UAV (rotation alarm) and when a UAV fails (failure alarm); it is answered when a
  reserve UAV is launched. Rotation alarms are served from U_rotate and failure
  alarms from U_spare. A UAV that completes its turnaround rejoins U_rotate.
* **Stage 5.** The failed UAV is the one on the configured route that is not yet
  being replaced. Candidate assignees are the UAVs on a route other than the
  failed one (U_fly \\ {u_f}); the contingency UAV is not a candidate, since it
  already carries every orphan cell. The insertion cost is the increase of the
  open-path length of the route, with insertion at either end allowed. Insertions
  are reverted when the contingency UAV reaches the first waypoint of the failed
  route, leaving every list exactly as Stage 2 produced it. Never-observed cells
  count as urgent.
* **Observation.** Any airborne UAV covers cells, also while ferrying or
  returning. Flight between distant waypoints therefore refreshes the cells it
  crosses.
* **Monitor C2.** The battery margin is taken over every airborne UAV, including
  returning ones.

## 4. Verification

* `pytest` checks each stage (grid size, boustrophedon order, round-robin balance,
  route admissibility, the fleet split against exhaustive search, batch departures,
  the return and replacement rules) and the whole loop (fleet state, C2, route
  inheritance, determinism, the failure sequence, bridging against route
  admissibility).
* The C3 monitor is tested on a deliberately starved reserve, where it has to fire.
* **Analytic coverage estimate.** Shrinking the footprint radius to 1 m, so that
  only waypoint visits count, the steady-state coverage is 45.4 % (N = 21) and
  94.6 % (N = 31), against 48.3 % and 98.6 % from the closed-form estimate with the
  simulated lap times (the closed loop, including the hop from the last waypoint
  back to the first: about 365 s). The same estimate with T_cyc = T_airborne + tau
  (about 703 s) would give only 25.6 % at N = 21, because T_cyc also contains the
  ferry legs and the turnaround, which do not delay the revisit of a cell during
  the loops of a sortie. At N = 11 it is 30.3 % against 23.2 %, because the
  straight flights between waypoints also cross cell centroids.

## 5. Observations from the runs

**Sensing footprint.** Steady-state coverage at N = 21 for footprint radii of
20 / 30 / 40 m is 84.3 / 88.2 / 96.9 %. Absolute coverage figures are only
comparable between studies that use the same footprint.

**Rotation pool and clause C3.** With the default parameters the nominal run
violates C3 in 20 of 32 alarms. The returning UAVs come back in clusters that
follow the batch structure; each rotation UAV is occupied for about 250 s
(incumbent overlap, return flight and the 60 s turnaround), while T_batch is
150 s, so four rotation UAVs are not enough. A first-order estimate agrees:
n_fly (2 T_ferry + τ) / (T_use - 2 T_ferry) = 16 · 424 / 1526 ≈ 4.4. The coverage
cost is small (at most three routes are briefly without a UAV, `rotation_pool.png`),
but the clause is not met. Reproduce with `--set`, e.g.
`python -m uav_surveil --set dispatch.prelaunch_margin=0.04`:

| Variation (N = 21 unless stated) | Rotation UAVs | Steady-state cov. (%) | C3 violations / alarms | Longest wait (s) |
|---|---:|---:|---:|---:|
| σ_pre = 0.03 | 4 | 87.8 | 9 / 32 | 84 |
| σ_pre = 0.04 | 4 | 87.6 | 18 / 32 | 113 |
| σ_pre = 0.06 (default) | 4 | 88.2 | 20 / 32 | 153 |
| σ_pre = 0.10 | 4 | 87.6 | 20 / 35 | 245 |
| N_batch = 2 | 2 | 91.4 | 30 / 36 | 164 |
| N_batch = 6 (14 flying) | 6 | 86.3 | 8 / 28 | 114 |
| N_batch = 8, N = 25 (16 flying) | 8 | 90.5 | 1 / 32 | 63 |

Changing N_batch also changes the batch schedule and, through the fleet-split
integer program, the number of flying UAVs, so these rows are not a pure pool-size
comparison.

## 6. Differences from the previous release of this repository

The simulator was rewritten; results of the earlier code are not comparable. The
deviations that were corrected:

| Earlier code | Now |
|---|---|
| Coverage counted cells seen within 120 s | Θ = 180 s |
| A replacement received only the rest of the incumbent's lap, so routes shrank from 9 to 3 cells and cells were dropped | The replacement flies the full route |
| The C3 monitor never fired (0 alarms in all runs) and used a 1 s deadline | Alarms are raised and judged against τ |
| Fleet split ⌈0.2 N⌉ spares plus a feedback loop on β | The fleet-split integer program |
| Cells sorted by distance from the depot, routed by nearest neighbour | Boustrophedon order, round-robin |
| Fixed 25 % ± 5 % return threshold (always 30 %) | Distance-aware return threshold |
| Pre-launch at the return threshold + 2 %, only beyond 200 m | Predicted-arrival replacement rule |
| 5 s launch gap | δ_launch ≥ τ |
| Battery drained over 1890 s, counting the 10 % reserve twice | Drained over T_endure = 2100 s |
| Orphan cells inserted at the current waypoint only, checked against SoC | Least-cost insertion subject to the endurance condition |
| Routes never checked against the endurance condition | Checked at design time and on every insertion |
| C2 counted ticks of any UAV below the floor, always zero | Battery margin over airborne UAVs |
| Contingency launched with a partial route | Inherits the full route |
| ALNS, k-means and "balanced" routers, six scenarios, Pyomo/SimPy/NetworkX/Pydantic, result CSVs and videos | Removed |

## 7. Limitations

One depot and straight-line flight (the single seam for other metrics is
`geometry.py`), no depot capacity, a single failure, deterministic dynamics with
no uncertainty in energy or travel time, homogeneous UAVs.
