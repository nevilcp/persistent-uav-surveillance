# Results

Headline metrics from the thesis (Chapter 4, Table 4.2). All runs use the
baseline 12×12 grid (500×500 m, Δ=40 m), v=6 m/s (v_max=9 m/s), B=2100 s, SoC floor 10%,
depot offset 500 m, deterministic Round-Robin routing with batch-stagger
phasing, and Θ=180 s as the revisit-gap analysis bound.

> Results regenerated on 2026-09-14 after the Stage 0/3A/3B/5 fixes
> (route-phase preservation, RTB/IDLE states, bridging K-cap/best-index,
> ETA/depot/n_surge/config-key corrections); thesis Table 4.2 reports the
> earlier code version. `sim_027-031` replace `sim_020/023/024/025/026`
> respectively; see `docs/thesis_errata.md` for the full mapping.

| Run | Description | Avg cov. % | Peak % | Time ≥ 90% (s) | Final overdue |
|---|---|---:|---:|---:|---:|
| `sim_027` | Steady-state baseline (N=21, 4800 s) | 83.3 | 98.6 | 1527 | 30 |
| `sim_028` | Under-provisioned (N=11) | 64.0 | 84.7 | 0 | 64 |
| `sim_029` | Over-provisioned (N=31) | 84.6 | 100.0 | 2964 | 29 |
| `sim_030` | Long-run stability (N=21, 9600 s) | 84.1 | 98.6 | 1960 | 16 |
| `sim_031` | Single-UAV failure at t=1800 s | 84.0 | 98.6 | 1823 | 25 |

C2 (battery-floor) and C3 (spare-latency) monitors stay at zero across all
steady-state runs.

## Baseline vs failure (Table 4.3)

|  | Avg cov. % | Time ≥ 90% (s) | Peak % | Avg overdue | Avg deployed |
|---|---:|---:|---:|---:|---:|
| `sim_027` baseline | 83.30 | 1527 | 98.6 | 24.05 | 16.80 |
| `sim_031` failure  | 83.96 | 1823 | 98.6 | 23.10 | 17.03 |
| Δ (fail − base)    | +0.66 pp | +296 | 0.0 | −0.95 | +0.23 |

Contingency promotion plus bridging absorb a single-UAV failure without
materially degrading mission-level behaviour: post-failure coverage dips
to a minimum of 69.4% (t≈1930-2020 s, driven by the lost UAV's own route
gap plus a nearby swap overlapping it) before the 240 s rolling average
recovers to ≥90% by t_fail+1128 s, and deployed capacity stays within
1.4% of baseline.

## Takeaways

1. **Steady cadence.** A simple, deterministic Round-Robin router with
   fixed batch-stagger phasing sustains low-80s coverage and clean C2/C3
   monitors on the baseline geometry.
2. **Provisioning is non-linear.** Moving from 11 → 21 UAVs is a
   qualitative step (never sustaining 90 % → spending nearly a third of
   the mission above it); 21 → 31 mainly narrows oscillations with
   diminishing returns on the average.
3. **No long-horizon drift.** The 9600 s repeat (`sim_030`) preserves
   the baseline envelope; no slow drift is observed once the fleet's
   spare pool is correctly replenished (see note below) — an earlier,
   buggy version of this fix showed a severe periodic collapse over
   the same duration.
4. **Single-failure resilience.** With one UAV lost at t=1800 s,
   contingency promotion plus capped bridging keep deployed capacity
   essentially unchanged, bound the post-failure coverage dip, and
   restore ≥90% rolling coverage within ~19 minutes.

## Note: two bugs found and fixed while validating this re-run

Re-running the baseline after the Phase-1 route-phase-preservation work
(`sim_027`) initially came back at 76-78% average coverage — well outside
the ±5 pp sanity band around the prior 85.2% baseline — and the 9600 s
long run periodically collapsed to single-digit coverage. Root-caused to
two bugs in `uav_surveil/gss/simulation.py`, both fixed in place:

- A UAV that finished a swap while the fleet was momentarily at full
  active strength was parked `IDLE` with `launch_time = inf` and never
  re-evaluated, permanently dropping it (and its route) from rotation.
- `IDLE` UAVs (waiting for their own phase slot) were excluded from the
  spare-selection pool used to cover *other* routes' swaps, contradicting
  the "state stays SPARE until then" design and starving rotation once
  the small dedicated spare pool was used up — this produced synchronized
  fleet-wide swap bursts and the long-run collapse.

A third, unrelated pre-existing bug (present since the initial public
release) was also fixed: `_export_failure_recovery_csvs` read the wrong
CSV column for coverage (`contingency_spares` instead of `coverage_%`),
making `sim_023`/`sim_031`'s `*_recovery_metrics.csv` meaningless
(`min_coverage_post_fail` was always ~0). The recovery-time check was
also switched from the raw per-tick coverage series to the rolling
average, since the raw series can read ≥90% momentarily right at
`t_fail`, before the failure's effect shows.

## Reproducing these numbers

Each run is fully self-contained:

| Asset | Location |
|---|---|
| Per-tick metrics CSVs | `results/sim_NNN_*.csv` |
| Final figures (8 per run) | `figures/sim_NNN_*.png` |
| Zipped per-run pack | `sim_packs/sim_NNN_*.zip` |
| Pack index | `sim_packs/index.json` |

Comparison plots across the three provisioning levels are in
`figures/compare_*`.

Configuration files for the baseline and the other four scenarios live in
`configs/`; all five runs were produced headlessly with
`tools/run_scenario.py` (e.g. `--sim-number 31 --failure` for `sim_031`'s
UAV-03-at-t=1800s failure, which is otherwise pre-configured in
`configs/baseline_v2.json`'s `failure.trigger`).
