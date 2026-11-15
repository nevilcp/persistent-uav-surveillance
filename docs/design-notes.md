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
| Choset decomposition, sweep order and round-robin | `grid.py` `boustrophedon_decomposition`, `choset_order`; `routing.py` `round_robin_routes`, `depot_round_robin_routes` |
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
| flyable distance d_G and path around no-fly zones | `airspace.py` `Airspace.distance`, `Airspace.path` |
| flyable distance to a base, nearest open base | `airspace.py` `Airspace.depot_distance`, `Airspace.nearest_depot` |
| flight along a detour, nearest-base return | `uav.py` `UAV.fly_leg`, `simulation.py` `_nearest_base`, `_move` |
| turnaround queue with κ slots, open / closed base | `depots.py` `Depot.arrive`, `Depot.advance`, `Depot.close` |
| per-base rotation and spare pools | `depots.py` `reserve_pool` |
| depot-sector routes, per-base ferry time | `routing.py` `depot_round_robin_routes`, `base_ferry_times` |
| reserves shared out over the bases | `fleet.py` `apportion`, `simulation.py` `_create_fleet`, `_take_reserve` |
| zone clearance (C7) | `airspace.py` `Airspace.signed_distance` |
| way-home margin (C5) | `monitor.py` `Monitor._extended_row` |
| base queue (C6), longest wait, busiest moment | `monitor.py` `Monitor._extended_row`, `_track_load` |
| global freshness recovery per breakdown (C4) | `monitor.py` `Monitor._recoveries` |
| C5-C7 verdicts, robustness, `passed` | `monitor.py` `Monitor.summary`, `_extended_summary` |
| primary simulation factory | `__init__.py` `make_simulation` |

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
| t_ss (baseline) | 1200 s | After the last initial launch (630 s) and the first loop |
| Failure | UAV on route 10 at t_f = 1800 s | Chosen so that the failed UAV is mid-sortie and not being replaced |
| κ (`depot_slots`) | 0 (unlimited) | Keeps the paper's depot; the port runs set it explicitly |
| β (`zone_margin`) | 0 m | No clearance beyond the zone edge unless a config asks for it |
| Attach radius | 1.5 Δ | A free point links to the visible cell centres within this radius before the cell-graph search; widened to every visible cell if none is in range |
| Extended monitor | zones, several bases or κ > 0 | The C5–C7 columns and summary keys appear only then, preserving the baseline metric columns; run summaries also identify the planner and fleet |
| Transit share | airborne UAV-steps that are not patrolling | Counts RETURNING and ON_ROUTE before the first waypoint (the ferry out) |
| Random-map shape | rectangles of 2–4 by 4–12 cells, 8% of the cells (at most 1.15 × that), no cell within 3 Δ of a base, free cells 4-connected | The doc asks for zones covering about 8% with the free space connected; the shape and size ranges are ours |
| Longest wait | largest cell age after t_ss and after every cell has been seen once | Deployment schedules differ by depot count; the first-full-observation condition excludes never-observed startup ages |

In the baseline, the surveyed region is the largest block of whole cells inside A: 12 × 12 = 144
cells of 40 m, i.e. 480 m × 480 m. D_ferry is measured to the corner of that block.

## 3. Interpretations

* **Repeated loops.** The loop and endurance conditions describe one depot-to-depot
  loop, whereas the return rules end a sortie when the battery runs low, which only
  happens after several loops. A UAV therefore repeats its loop (including the hop
  from the last waypoint back to the first) until the return rule fires; the
  endurance condition is checked for a single loop.
* **Replacement route (round-robin).** The replacement starts at the first waypoint of the full
  route, not at the incumbent's position, and is launched when the replacement
  rule first holds. If no rotation UAV is waiting, the alarm stays open and is
  served as soon as one is (oldest first); the incumbent returns regardless.
* **Launch gap (baseline).** δ_launch = τ = 60 s (the smallest value allowed) with
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
* **Flyable distance.** d_G is the shortest path over the 8-neighbour graph of free
  cell centres (links crossing an inflated zone are dropped), joined to the end
  points by straight links; a clear straight segment is returned as is. The graph
  can exceed the exact any-angle geodesic; recompute distances for the current
  500 m depot setback. Grazing a zone edge counts as clear, since C7 asks for a
  clearance of at least β. A cell whose centre falls inside an inflated zone is not
  surveyed, so a margin β > 0 can remove cells next to a zone. Without zones every
  query reduces to the Euclidean distance and no graph is built.
* **Legs.** A UAV flies a polyline (`leg`) planned by the airspace each time its goal
  changes: at every waypoint hop and when it turns for home. The step left over at a
  detour vertex carries on to the next one; at the final point it is discarded, as
  before. The return decision uses the distance of `dispatch.metric` to the nearest
  base, while the path flown is always the flyable one, so the "straight" metric
  underestimates the way home and is only meant for the R0 comparison. `UAV.home` is
  the base the UAV is parked at or heading to. Every base needs a clear line to at
  least one free cell, otherwise the airspace refuses to build.
* **Base queue.** A base with κ > 0 swaps at most κ UAVs at a time; the others wait in
  a first-come first-served queue in the QUEUED state and start a full turnaround
  when a slot frees. Q_m for C6 is the number waiting, not those in service, so C6
  asks that no more than κ UAVs ever wait. With κ = 0 every arrival starts at once,
  as in the paper. Reserve selection uses ready UAVs at the applicable open base, with
  round-robin rotation pools tied to route home bases.
* **Choset plus round-robin.** This is the only supported planner. Assign each free
  cell to its nearest depot by flyable distance. Give every nonempty sector one
  flyer, then apportion remaining flyers by the remaining cell counts. Within each
  sector, split/merge events of horizontal free intervals define decomposition
  regions. Traverse their adjacency graph and sweep each region back and forth;
  deal the ordered cells cyclically to its routes. Connect all hops by flyable
  paths, including each route's closing leg. This is a raster implementation,
  rather than exact continuous-polygon decomposition. Odd grid dimensions work.
  On the 500 m-setback port map the sectors have 309, 290 and 231 cells and
  32, 30 and 25 routes. Rotation and spare pools follow route counts. Each depot
  has its own initial departure schedule. Replacements start at a route's first
  waypoint, preserving the existing dispatch policy.
* **Removed shared-loop controller.** The global cyclic planner, arc interpolation,
  ideal-spacing fleet sizing, relief intercept and re-spacing code have been
  removed. `shared_loop` configs are rejected. The port instead fixes 120 drones
  split 87 + 27 + 6 and uses depots at (−500, 600), (1700, 600), (600, 1700).
* **Repeated failures.** Each scheduled failure gets a failure manager and an alarm.
  A configured `near_depot` selects a flying route served by that depot; otherwise
  the configured route is used. Managers bridge urgent cells until their spare
  arrives; cleanup removes only that failure's temporary insertions. A spare can
  come from another open depot if the route's own depot has none. There is no
  global repartition after spare exhaustion.
* **Base closure.** A closed depot's stored UAVs are unavailable. Affected routes
  are rehomed at an open depot by mean flyable waypoint distance, preserving their
  waypoint sets; incoming returns are redirected and future returns use open
  depots only. Queued turnaround at the closed depot stops. The outage run can
  violate the reserve and return-feasibility clauses, as recorded in the study.
* **C5–C7.** C5 is the smallest, over airborne UAVs, of b − d_G / range − σ_min with
  d_G to the nearest open base whatever the dispatch metric, so the "straight" run
  can be caught out. C7 is the smallest signed zone distance minus β. C6 is the
  largest Q_m − κ_m over the bases with κ > 0, and is violated when positive.
  Its robustness is the negative of the maximum excess, reported as `min_margin`. C5 and
  C7 are violated below zero, and their robustness is the smallest value reached.
  A UAV inside a zone has no flyable way home, so C5 falls back to the straight
  line there and C7 reports the violation. A clause with nothing to measure (no
  zones, no slot limit, nobody airborne) is blank in the CSV and null in the summary.
  `passed` means no C2, C5, C6 or C7 violating step.
* **C4 per breakdown.** For each breakdown the recovery time is the time from t_fail
  to the first restoration of max_age ≤ Θ after cells go stale, before the next
  breakdown (or the end of the run): 0 if nothing went stale, null if no restoration
  occurred in that window. Later independent freshness dips do not change that time. It is satisfied when at most Θ + ε.
* **Load figures.** The longest wait is the longest time a UAV spent in a base queue
  (a UAV still queued at the end counts up to then); a base's busiest moment is
  its peak number of UAVs present (in service plus waiting), with the first time it
  was reached and the peak queue.
* **Port study.** R0 changes the return-distance estimate, R1 tests each depot
  alone with all six service slots, R2 removes obstacles, R3 is the main run,
  R4 adds one failure, R5 adds four failures, and R6 closes the north depot.
  Every run uses the primary Choset-plus-round-robin planner and 120 drones.
  Twenty random NFZ layouts use seeds 0–19; infeasible setups remain in pass
  counts and are excluded from min/mean/max metrics. The movement is deterministic.
* **Observation.** Any airborne UAV covers cells, also while ferrying or
  returning. Flight between distant waypoints therefore refreshes the cells it
  crosses.
* **Monitor C2.** The battery margin is taken over every airborne UAV, including
  returning ones.

## 4. Verification

Monitor verdicts are evaluated at the configured time steps (1 s in the reported
runs). Passing tests and sampled checks are not a formal proof of continuous-time
safety. The failure manager measures re-observation of orphan cells. Extended runs also
report global freshness restoration per breakdown. These recovery metrics differ
and the global metric can remain unmet because of pre-existing stale cells.

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
cost is small (at most three routes are briefly without a UAV, `rotation-pool.png`),
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

Known static rectangular zones, instant turns, exact state, homogeneous UAVs and
deterministic energy and travel time. The primary planner supports consecutive
failures and depot closure. Grid detours approximate any-angle paths; boundary
grazing is allowed with numerical tolerance, and a positive margin expands the
zones. The model does not include inter-drone collision avoidance.

Choset decomposition is implemented on the raster. Weighted connected sectors,
capacity optimisation, predictive intercepts and global repartition after spare
exhaustion remain future work. The monitor uses C7 for obstacle clearance.
Observed queue, freshness and outage reserve violations are retained in
[the current study](port-study.md). The former shared-loop implementation is
recorded in [the historical audit](implementation-status.md).
