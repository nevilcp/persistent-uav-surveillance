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
| flyable distance d_G and path around no-fly zones | `airspace.py` `Airspace.distance`, `Airspace.path` |
| flyable distance to a base, nearest open base | `airspace.py` `Airspace.depot_distance`, `Airspace.nearest_depot` |
| flight along a detour, nearest-base return | `uav.py` `UAV.fly_leg`, `simulation.py` `_nearest_base`, `_move` |
| turnaround queue with κ slots, open / closed base | `depots.py` `Depot.arrive`, `Depot.advance`, `Depot.close` |
| per-base rotation and spare pools | `depots.py` `reserve_pool` |
| home base of a route, per-base ferry time | `routing.py` `assign_bases`, `base_ferry_times` |
| reserves shared out over the bases | `fleet.py` `apportion`, `simulation.py` `_create_fleet`, `_take_reserve` |
| shared closed loop, arc position | `routing.py` `shared_loop`, `Loop.point_at`, `Loop.cells_near` |
| loop fleet: flyers, rotation pools, spare bases | `fleet.py` `size_loop_fleet`, `assign_vertices` |
| zone clearance (C7) | `airspace.py` `Airspace.signed_distance` |
| loop simulation, hand-over, spares, closure | `patrol.py` `LoopSimulation` |
| relief intercept point | `patrol.py` `LoopSimulation._intercept`, `_send`, `_hand_over` |
| re-spacing controller | `patrol.py` `LoopSimulation._respace` |
| way-home margin (C5) | `monitor.py` `Monitor._extended_row` |
| base queue (C6), longest wait, busiest moment | `monitor.py` `Monitor._extended_row`, `_track_load` |
| recovery per breakdown (C4, loop) | `monitor.py` `Monitor._recoveries` |
| C5-C7 verdicts, robustness, `passed` | `monitor.py` `Monitor.summary`, `_extended_summary` |
| patrol mode selection | `__init__.py` `make_simulation` |

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
| f (`failures_absorbed`) | 1 | One additional flying UAV above the ideal freshness count; depot spares are sized separately |
| δ (`handover_overlap`) | 10 s | Target lead time at the meeting point; late reliefs and pool shortages can still open gaps |
| T_resp (`response_time`) | 150 s | Between the 141 s that all three port bases achieve and the 161 s of the best pair, so the port needs all three bases for spares |
| T_gain (`respace_gain_time`) | 300 s | Provisional; a spacing error is removed over about two revisit periods |
| Attach radius | 1.5 Δ | A free point links to the visible cell centres within this radius before the cell-graph search; widened to every visible cell if none is in range |
| Staggered first sortie | the i-th of n_m flyers of a base is relieved after i / n_m of (T_use − 2 T̄_m − δ) | Spreads the first relief requests over one sortie instead of one burst; later reliefs are not cut |
| Hover and slow-flight drain | cruise rate | Modelling assumption: loop UAVs and waiting reliefs drain by time at the cruise rate; this is not a calibrated airframe power model |
| Extended monitor | zones, several bases or κ > 0 | The C5–C7 columns and summary keys appear only then, so the paper's CSV and summary stay byte-identical |
| Transit share | airborne UAV-steps that are not patrolling | Counts RETURNING, RELAYING and, for round-robin, ON_ROUTE before the first waypoint (the ferry out) |
| R1 response time | the base's own worst-case reach (s) + 1 | One base cannot meet T_resp = 150 s, so the single-base runs get the response time that base can achieve and still have a spare |
| Random-map shape | rectangles of 2–4 by 4–12 cells, 8% of the cells (at most 1.15 × that), no cell within 3 Δ of a base, free cells 4-connected | The doc asks for zones covering about 8% with the free space connected; the shape and size ranges are ours |
| Longest wait | largest cell age after t_ss and after every cell has been seen once | The round-robin plan launches its last route at about 2000 s, so its ages are infinite after t_ss without the second condition |
| Rotation margin | 0 by default; 3 per serving depot in `port.json` | Extra reserve above the Little's-law average for handover and service clustering; the port therefore has 57 drones rather than the floor of 48 |
| Minimum speed ratio | 0.5 | Provisional; the re-spacing controller never flies slower than half of v_max |

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
  adds a few percent over the exact geodesic (drone A to the east base: 922 m
  against 884 m). Grazing a zone edge counts as clear, since C7 asks for a
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
* **Several bases (round-robin).** The routes are the round-robin split of the free
  cells only, so zone cells are neither routed nor counted in the coverage. A route
  is homed at the base with the smallest mean flyable distance to its cells, and its
  ferry time is that of the base: the longest flyable distance from the base to a
  free cell (the corner formula when there are no zones). Round-robin routes spread
  over the whole region, so their means differ little and the choice leans towards
  the central base: on the port map 56, 20 and 11 of the 87 routes go to the west,
  east and north bases. Flyers start at their base; the rotation and spare UAVs are
  shared out in proportion to the route counts (largest remainder). A rotation
  alarm is answered from the home base of its route only; a failure alarm takes a
  spare from that base or, if it has none, from the nearest base that has one.
* **Shared loop.** The loop is the closed boustrophedon of the full grid: rows
  alternate east and west over columns 1 to n − 1 and column 0 brings it home (a
  Hamiltonian cycle, so L = n_cells · Δ without zones; with an odd number of rows
  the same cycle is built on the transposed grid). Zone cells are dropped from the
  cycle, and consecutive free cells that cannot see each other are joined by the
  flyable path, so each row that meets a zone makes a detour. On the port map this
  gives L = 40.3 km against 33.2 km for 830 cells of 40 m.
* **Loop sizing.** n_fly = ⌈L / (v Θ)⌉ + f. Every loop vertex belongs to the nearest
  open base by flyable distance, weighted by half of the segments around it, which
  gives the base's share of L and its mean transit time T̄_m. The flyers are shared
  out in proportion to the shares (largest remainder). A base's relief requests
  come at λ_m = n_fly,m / (T_use − 2 T̄_m − δ), a relief stays away for
  W_m = 2 T̄_m + τ + δ, and the rotation floor is ⌈λ_m W_m⌉, rounded up per
  base. The configured `rotation_margin` is added at each serving base. ρ_m = λ_m τ / κ_m (0 when κ is unlimited). Spares sit at the smallest set of
  bases from which every loop vertex is reachable within T_resp (ties go to the
  smaller worst-case response, then to the lower indices), one spare per base of the
  set. On the port map, the zero-margin floor is 39 flyers, 6 rotation and
  3 spares (48 UAVs). `configs/port.json` adds 3 rotation UAVs per serving base,
  yielding 39 + 15 + 3 = 57. `fleet.size` plays no part in loop mode.
* **Loop simulation.** `LoopSimulation` owns the same airspace, bases, coverage map and
  dispatch rules as `Simulation`. A looping UAV is ON_ROUTE with no route and an arc
  position; its battery drains at the cruise rate. The flyers launch at t = 0 as
  reliefs of virtual slots spaced L / n_fly apart that move at v_max, targeting
  even spacing after deployment; handovers and failures can disrupt that spacing. A relief request (the
  paper's `needs_replacement` with d_G to the nearest open base, or the staggered
  cut) scans the loop ahead of the incumbent for the first vertex that a relief from
  the nearest open base reaches δ before the incumbent does, skipping vertices the
  incumbent cannot reach with the return rule still satisfied. If none is on time the
  least late one is used, and with no spare battery at all the incumbent is met where
  it is. The relief is launched from the base nearest that vertex, or the alarm
  stays open when that base has none parked, so C3 measures pool shortage. The relief
  hovers (RELAYING) until the incumbent arrives, then takes its arc and the
  incumbent flies home; a relief that arrives after the incumbent has gone joins at
  its meeting point. An incumbent that has to return before any relief is free leaves
  a vacancy that moves on at v_max, and the relief is sent to it when one is free.
* **Re-spacing.** UAVs can only slow down, so the reference UAV is chosen among those
  that leave nobody behind their target (one always exists), and among them the one
  with the smallest largest lead. Each UAV ahead of its target by e flies at
  v − min(e / T_gain, (1 − ρ_min) v). Choosing the reference by smallest lead alone
  would pick a UAV that leaves everyone "behind", and a hole in the train would never
  close. The controller is paused while a relief is on its way to a slot that has no
  incumbent (start-up, spares, vacancies), because that slot would look like a hole
  and the leaders would be slowed for nothing.
* **Breakdowns and spares.** The victim is the looping UAV nearest the chosen base, or
  the `route`-th in arc order when none is chosen. The failure alarm launches a spare
  from the base nearest the midpoint of the largest gap, counting reliefs on their
  way as occupied slots. The spare meets that point as a virtual slot with no
  overlap. A rotation request of the victim is dropped.
* **Base closure.** The closed base stops accepting UAVs: those returning to it are
  re-routed to the nearest open base and every later return and relief request uses
  the open bases only. The sizing is re-run for the open bases and logged as a
  `resize` event; if no spare set reaches the whole loop within T_resp the event says
  so and the run goes on. UAVs already parked at the closed base stay there.
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
* **Port study.** `scripts/run_port_study.py` builds R0–R6 with `replace()` from
  `configs/port.json`. R1 runs each base alone with κ = 6 (all the slots of the three)
  and reports the one with the most steady coverage, then the shortest wait. R3 is the
  round-robin plan with N = 120, batch 27 and ρ_min = 0.05, which `split_fleet` turns
  into 87 + 27 + 6. R5 breaks the loop UAV nearest the east base at 1800, 2100, 2400
  and 2700 s; R6 closes the north base at 1800 s. The doc's predictions (L ≈ 35 km,
  43 UAVs, A_1 = 177 s) are printed next to the simulated values and never asserted.
  Random maps use seeds 0–19; a map whose set-up is infeasible (no spare set within
  T_resp, or a non-admissible round-robin route) is counted as infeasible and left out
  of the min / mean / max.
* **Observation.** Any airborne UAV covers cells, also while ferrying or
  returning. Flight between distant waypoints therefore refreshes the cells it
  crosses.
* **Monitor C2.** The battery margin is taken over every airborne UAV, including
  returning ones.

## 4. Verification

Monitor verdicts are evaluated at the configured time steps (1 s in the reported
runs). Passing tests and sampled checks are not a formal proof of continuous-time
safety. Baseline C4 measures re-observation of orphan cells; shared-loop C4 measures
restoration of freshness over all free cells, so the two recovery metrics differ.

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
deterministic energy and travel time. Consecutive failures and depot closure are
implemented in shared-loop mode. Grid detours approximate any-angle paths. Grazing
a zone boundary is allowed, with numerical tolerance; a positive margin expands
that boundary. Shared-loop interval formulas assume ideal spacing and uninterrupted
handover, so the study reports observed worst age as well. Capacity and Little's-law
checks do not guarantee that no launch or turnaround cluster occurs.

The extension report also proposes weighted connected sectors, Choset decomposition
and global repartition after spare exhaustion. Those algorithms are outside the implemented study scope; the round-robin
comparison here remains globally interleaved. The monitor numbering follows the plan (C7 is obstacle clearance).

See [the implementation audit](implementation-status.md) for the step-by-step review,
verification evidence and current study results.

The verified fixed-map and 20-map study is recorded in [port-study.md](port-study.md).
