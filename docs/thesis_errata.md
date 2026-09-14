# Thesis errata

The thesis source (LaTeX/Word) is not version-controlled in this repo, so it
cannot be corrected in place. This table records where the *thesis prose*
(and, in a few cases, App. B's verbatim config listing) has diverged from
the final, implemented design — either because the thesis describes an
earlier iteration that was later superseded, or because a real code bug
has since been fixed and the thesis text was never updated to match.
`docs/architecture.md` and `docs/baseline_config.md` are the current
source of truth; this page exists purely to reconcile the published thesis
against them.

| Thesis passage | What it says | Final decision | Code location |
|---|---|---|---|
| §3.4 / Alg. 2 | Stage 0 battery check is described but is dead code, never called from the pipeline; ξ is framed as a battery-reserve fraction to maximise. | Stage 0 is wired into `GSSSimulation.initialize()` after Stage 3A (routes known), before Stage 3B; ξ is redefined as a **deficit** — 0 when feasible, growing above 0 by the shortfall when a route's loop exceeds usable battery. | `uav_surveil/stage0_battery.py:35` (`optimize_battery_reserve`), `uav_surveil/gss/simulation.py` (`initialize()`) |
| §3.6 | `n_surge` (simultaneous pad demand) is described informally without a precise definition; the code used a rough approximation. | `n_surge` is the true peak concurrency of swap windows `[d_u + k·L_u, +Δ_swap)` across all routes, found by a sweep-line over one cycle `T_cyc`. | `uav_surveil/stage3_schedule.py::_peak_swap_concurrency` |
| §3.7.1 | References the route-generation module by the file name `stage3_route_rr.py`. | The module is `stage3_route_roundrobin.py`; no `stage3_route_rr.py` exists in the repo. | `uav_surveil/stage3_route_roundrobin.py` |
| §3.7.2 / Alg. 6 | Intra-batch stagger is specified as `s ≥ Δ_swap` (i.e. at least one full hot-swap time, 60 s). | Final baseline uses a fixed 5 s intra-batch stagger (pad-clearance only, not swap-avoidance) — `Δ_swap` is instead handled by phase offsets across the full cycle, not the intra-batch gap. | `uav_surveil/stage3_schedule.py` (`within_batch_gap = 5.0`) |
| §3.8 / Alg. 7 | θ_return is described as a continuous function of distance to depot. | θ_return is a **step** function: Δθ ∈ {−0.05, 0, +0.05} for d < 200 m, 200–400 m, > 400 m respectively. | `uav_surveil/stage4_policy.py:50-74` |
| §3.8 | A separate `σ_pre` parameter is described for the pre-launch margin. | There is no `σ_pre`; the margin is `ε` (`battery.prelaunch_margin`), added directly to `θ_return` in the ETA pre-launch check. | `uav_surveil/gss/simulation.py:1146` (`_check_eta_prelaunch`), `uav_surveil/config/parameters.py` (`BatteryParameters.prelaunch_margin`) |
| §3.8 | Tail-extension is described with a time-based bound on the trivially-short segment. | Implemented as a cell-count rule: extend the starting index backwards to keep at least 3 cells in the spare's remaining assignment. | `uav_surveil/gss/simulation.py:1091` |
| §3.10 | Bridging is described as processing up to `K` urgent cells per tick, but the code capped insertions at `2K` and inserted at the current waypoint slot rather than the best index. | Capped at exactly `K` insertions per tick (`bridge_policy.realloc_k`); each candidate insertion is scored by `ΔT_u(j)` over a bounded window and the best `(uav, index)` pair is chosen, subject to the `max_detour_ratio` geometry guard. | `uav_surveil/stage5_failure.py` (`BridgeManager.tick`) |
| §3.11 | Describes a `C4` violation counter tracked during the run. | C4-equivalent recovery evidence (minimum post-failure coverage, rolling-average time-to-90%-recovery) is exported to `results/<tag>_recovery_metrics.csv` after the run, not tracked as a live counter. | `uav_surveil/gss/simulation.py:1286` (`_export_failure_recovery_csvs`) |
| App. B | Verbatim parameter listing includes `max_speed`, `coverage_overlap`, and `orphan_recovery_slack`. | These three keys are unused by any stage and have been removed from the Pydantic config models and all `configs/*.json` scenarios. (`spare_launch_deadline`, `max_detour_ratio`, and `total_endurance`, also listed in App. B, remain and are now wired — see `docs/baseline_config.md`.) | `uav_surveil/config/parameters.py` |
| Table 4.2 | Headline coverage/violation numbers for `sim_020/023/024/025/026`. | Superseded by the regenerated `sim_027-031` runs (same scenarios, post Stage 0/3A/3B/5 fixes) — see `RESULTS.md` for current numbers and the tag mapping. | `RESULTS.md` |
