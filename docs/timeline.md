# Build timeline

How the end-to-end architecture was designed and implemented from scratch,
leading to the steady-state baseline and the single-UAV failure demo.

## 0) Paper foundation
- Wrote the staged architecture and math: Stage 0–6 (Battery, Grid, Fleet, Routing/Schedule, Policy, Failure, V&V).
- Drafted STL clause intentions (C1–C4) and clarified they would be monitored as proxies in this thesis (not formal robustness tooling in‑repo).
- Baseline/scenario fixed: 500×500 m, 12×12 grid, v=6 m/s (v_max=9 m/s), B=2100 s, SoC floor 10%, depot offset 500 m, Θ=180 s for analysis.

## 1) Repo bootstrap
- Created package skeleton (`uav_surveil/`), configuration system, core models (`UAV`, `Cell`, `Route`).
- Added visualizer and metrics CSV export to validate the loop early.
- Implemented Stage‑0 feasibility check from the battery inequality.

## 2) Routing exploration (Stage‑3A)
- Greedy serpentine: functional but unbalanced loops and depot bursts.
- KMNN/K‑cluster + NN: better locality, still uneven durations and phasing.
- ALNS (time‑limited): on the uniform 500×500 baseline and moderate fleets, showed no consistent improvement over simple baselines while increasing runtime.
- Round‑Robin (final): furthest‑first seeding + light local smoothing → balanced loop lengths, predictable phasing, best coverage/runtime. Selected as steady‑state baseline.

## 3) Schedule & policy (Stage‑3B/4)
- Schedule packing: batch size 4, stagger 5 s, batch period 150 s to keep depot contention low.
- Policies: distance‑aware RTB, ETA pre‑launch, tail‑extension to eliminate wasteful short flights.

## 4) Steady‑state stabilization (sim_014–017)
- Patches: tail‑extension; claim expiry on observation; ETA pre‑launch restored; furthest‑first seeding.
- Achieved peak ≈99% and stable loops at Θ=180 s → steady‑state baseline frozen for thesis.

## 5) Failure handling (Stage‑5 quick‑patch)
- Added `FAILED` state (freeze visually + logically) and failure triggers.
- Bridging: periodic urgent‑cell insertion to nearby UAVs with caps and hysteresis.
- Contingency UAV: reserved spare for takeover; after handover the contingency is promoted back into rotation.
- Fixes across runs: first true freeze (sim_021), promotion to avoid depot‑stuck (sim_022→sim_023).

## 6) Runtime/I/O hardening
- SoC sampling reduced to 5 s and limited to contingency + bridging neighbors.
- Global throttle for repeated console warnings; capped bridging work per tick; option to reduce coverage snapshot cadence.

## 7) Verification & figures (Stage‑6)
- Built figure generator to produce: coverage vs time, violations, revisit percentiles, fleet state, SoC traces, cost index, and comparison overlays from existing CSVs.
- Clarified monitors: **C2** (battery safety policy) and **C3 a/b miss** (spare availability). Proxies, not formal STL.

## 8) Comparative studies & long‑run
- sim_020 (21 UAVs): avg 85.2%, peak 99.3%, time≥90% 2282 s.
- sim_024 (11 UAVs): under‑provisioned; avg 67.6%, time≥90% 0 s.
- sim_025 (31 UAVs): stronger compliance; avg 88.5%, time≥90% 3485 s; diminishing returns vs 21.
- sim_026 (21 UAVs, 9600 s): long‑horizon; percentiles healthy; a few chronic hotspots reveal route omissions to refine (not capacity‑limited).

## 9) Documentation consolidation
- Stage math, algorithms, GSS flow, monitors, outputs and how-to are in
  `docs/architecture.md`; canonical baseline numbers in
  `docs/baseline_config.md`; headline results in `RESULTS.md`.

## 10) Post-thesis alignment (Sep 2026)
A cross-check of the thesis (Ch. 3, Table 4.1, App. B) against the codebase
and `docs/` found real gaps, bugs, and doc drift accumulated since the
thesis was written. Fixed in place (no legacy flags), one commit per
work-package:

- **WP1 — Stage 0 wiring**: corrected ξ to a deficit (0 when feasible,
  >0 shortfall otherwise, per §3 above); wired the check into
  `GSSSimulation.initialize()` after Stage 3A so an infeasible route
  raises before the sim starts.
- **WP2 — Stage 3A**: implemented the documented serpentine sweep +
  furthest-first seeding + boundary-smoothing algorithm (§4.1), replacing
  the prior greedy/KMNN-derived generator.
- **WP3 — Phase preservation**: routes keep a fixed phase offset across
  battery swaps (`UAV.phase_offset`); added `RTB`/`IDLE` states to
  `UAVState` so on-mission-returning and post-swap-waiting are
  distinguishable from `SPARE`.
- **WP4 — Stage 5 bridging**: capped insertions at exactly `K` per tick
  (was up to `2K`), insert at the best-scoring index within a bounded
  horizon instead of the current waypoint slot, and enforce the
  `max_detour_ratio` geometry guard.
- **WP5 — UAV state model**: `RTB`/`IDLE` propagated through
  `simulation.py`, `stage4_policy.py`, `stage5_failure.py`, and the
  plotting scripts' colour maps.
- **WP6 — Bugs and config hygiene**: fixed the ETA pre-launch range
  (`battery.max_range` never existed — silently defaulted to 10,000 m),
  read the depot from config instead of a hardcoded value, computed
  `n_surge` from true peak swap-window overlap, wired
  `spare_launch_deadline`/`max_detour_ratio`/`total_endurance`, moved
  `prelaunch_margin` to `battery.*`, and removed six unused config keys.
- **WP7 — Headless runner**: added `tools/run_scenario.py` for
  CI/batch-friendly runs without the animation overhead, reusing the
  same CSV export path as `examples/visualize_simulation.py`.

Two further bugs surfaced only while re-running the baseline under the
phase-preservation fix (WP3): a UAV parked `IDLE` with `launch_time = inf`
could get permanently stranded, and `IDLE` UAVs were wrongly excluded from
the spare-selection pool, starving rotation under load. Both fixed in
`simulation.py`; see `RESULTS.md` for the full account.

**Re-run mapping** (Phase 2 — new tags continue from 027, old tags/figures
retired once the new ones were validated):

| New tag | Replaces | Command |
|---|---|---|
| `sim_027` | `sim_020` | `--config baseline --sim-number 27` |
| `sim_028` | `sim_024` | `--sim-number 28 --inventory 11` |
| `sim_029` | `sim_025` | `--sim-number 29 --inventory 31` |
| `sim_030` | `sim_026` | `--sim-number 30 --duration 9600` |
| `sim_031` | `sim_023` | `--sim-number 31 --failure` (u03 @ 1800 s) |

See `docs/thesis_errata.md` for the full thesis-passage-by-passage mapping
and `RESULTS.md` for the regenerated headline numbers.

---

## Definitions
- **Steady‑state scenario**: failure disabled; Round‑Robin routing + staggered schedule + Stage‑4 policies; objective is sustained high coverage with max revisit ≤ Θ=180 s for almost all cells.
- **Failure scenario**: single UAV failure at configured time/ID; FAILED UAV frozen; bridging covers urgent orphans; contingency takes over and is promoted to rotation; Stage‑5 disabled post‑handover; recovery metrics tracked.

## Earlier methods vs final choice
- Greedy/serpentine and KMNN/NN gave uneven loop durations and depot bursts.
- ALNS showed limited benefit on the uniform baseline; future gains expected with non‑uniform priorities, multi‑depot, heterogeneous fleets, time windows, wind/energy models, or multi‑failure resilience.
- Round‑Robin with furthest‑first provided the best coverage/runtime trade‑off for the thesis baseline and was adopted as default.

## Future scope (post‑thesis)
- Formal STL robustness + Monte‑Carlo validation; dynamic reallocation (Stage‑5b) and local merge optimizer; advanced ALNS/VRP in richer scenarios; hardware integration via GSS adapters.
