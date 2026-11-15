# Historical implementation audit

This is the audit of the former shared-loop implementation at the commits named
below. The current planner is Choset decomposition plus sector-local round-robin;
the shared-loop module, sizing code and associated tests have been removed. See
[the current study](port-study.md) and the extension report for the new runs.

Audit of the implemented extension scope on
`extension/multi-depot-no-fly-zones`, based on baseline commit `8e1c6b6`. Simulation and study code is recorded in
`cce6ca4`; animation support is recorded in `3308e40`. Documentation and media
filename updates were uncommitted at this audit revision.
The temporary implementation plan was removed after its implementation and
verification work was completed. The experiment definitions implemented here
differ from the [extension report](extending-resilient-uav-swarm-surveillance-multiple-depots-and-no-fly-zones.md),
which specifies different sector routing, failure redistribution, monitor
numbering and run labels; this audit does not claim conformance to those.

| Step | State at audit start | Completion and evidence |
|---|---|---|
| 0 — branch | Feature work local and uncommitted | Current extension branch retained; extra feature branch deleted; `main` retained. Each commit requires permission. |
| 1 — parameters | Implemented | Frozen parameters, nested JSON tuples, depot fallback, patrol and failure controls; `test_params.py`. |
| 2 — airspace | Implemented with a boundary interpretation | Cached Dijkstra paths, free-cell mapping, depot tables and Euclidean fallback; `test_airspace.py`. Boundary grazing is allowed within 1e-9 m rather than rejected, consistent with nonnegative clearance. |
| 3 — motion | Implemented | Polyline legs, nearest-open-base dispatch, relay state and hover drain; `test_motion.py`, `test_patrol.py`. |
| 4 — depots | Implemented | FIFO turnaround, parallel slots, closure, reserve pools and unlimited-slot fallback; `test_depots.py`. |
| 5 — multi-depot round-robin | Implemented | Free-cell routes, mean-distance base assignment, per-base ferry bounds and pools, flyable insertion cost; `test_multibase.py`. Routes remain globally interleaved as the plan specifies. |
| 6 — shared loop | Implemented | Closed sweep, obstacle detours and arc interpolation; `test_loop.py`. Odd-by-odd grids cannot have the specified adjacent Hamiltonian cycle and are rejected. |
| 7 — loop sizing | Implemented with an optional reserve margin | Flying count, proportional base shares, Little's-law rotation floor, utilisation and exhaustive spare set cover; `test_loop_sizing.py`. The default extra margin is zero; the port uses three extra rotation drones per serving depot. |
| 8 — loop simulation | Implemented | Virtual-slot deployment, phased first sorties, relief intercepts, spare launches, re-spacing and closure rerouting; `test_patrol.py`. Infeasible post-closure sizing is logged and does not create additional drones. |
| 9 — monitoring | Mostly implemented | Added explicit C6 robustness (`min_margin = -max_excess`) and corrected per-breakdown C4 to report the first restoration after staleness rather than the last stale period. Baseline output keys are preserved; `test_monitor_extended.py`. |
| 10 — study | Implemented | Port config, R0–R6, single-base comparison, prediction tables, 20 seeded random maps and figures; `test_port_study.py`. Recorded study results are reported below and in the README. |
| 11 — animation | Missing | Added baseline and port scenario selection, free-cell freshness mapping, zones, loop, extent, multiple depots, closure markers and per-depot parked chips. Baseline frame JSON and GIF verified byte-identical. |
| 12 — tests | Component tests implemented | Added exporter/renderer coverage, full-mission R4 safety, a controlled R0 pocket-return counterexample, and first-restoration C4 coverage. Original tracked tests are unchanged. |
| 13 — documentation | Design notes partially extended | Updated README scope, architecture, test count, port commands and results; completed design interpretations and limitations, plus this audit. |

## Verification

Verification was performed on 2026-11-15 with Python 3.14.4. The checks below
record that run; they are not claims of statistical generality or a formal safety
proof. The figure and media filename changes preserve their binary contents.

- Full suite: 171 tests, including the original tests without edits.
- Nominal CLI stdout and all five baseline scenario summaries: byte-identical to
  `main`. Baseline verification used an archive of `main`, without switching branches.
- Full 4800-frame nominal JSON export: byte-identical to `main`.
- Nominal GIF regenerated with the default GIF command: byte-identical to
  `assets/single_depot/nominal-surveillance.gif`.
- Full R6 frame export, a post-closure still and a 72-frame full-horizon GIF:
  rendered successfully. Inspection confirmed obstacles, the loop, three depot
  markers and a hollow closed north depot.
- R0–R6 plus all three R1 locations: fresh 4800 s simulations, with steady-state
  measurements after 1200 s. R3/R4 also run on 20 deterministic random maps.
- Matplotlib figures generated from the saved study time series. C4 summaries
  recomputed from those same series using the corrected first-restoration metric.
- Full results snapshot: [port-study.md](port-study.md).
- The verification run used `/tmp/uav-plan-audit/` for generated artifacts;
  that temporary path is not a permanent data archive. Normal commands write
  raw time series and summaries to ignored `results/` directories; retain those
  outputs together with the configuration and code revision for later analysis.

## Observed results and limits

The fixed-map loop is 40.347 km; the sizing rule assigns 39 flyers. Its rotation floor is 6 drones;
with the port's extra margin it uses 15 rotation drones and 3 spares, 57 in total.
The predicted 35 km / 43-drone result is not reproduced. R4 has 99.17% steady-state
coverage and no C2/C5/C6/C7 violation. This safety verdict is distinct from C1:
coverage still dips below 100% during handovers.

R0 has no C2/C5 violation in its natural trajectory with this reserve margin.
The controlled regression starts a returning drone in the L-shaped pocket at the
straight-distance return threshold and observes a C2 violation. This demonstrates
the metric flaw without claiming that the fixed R0 trajectory exposes it.

R1 east has the best single-base coverage (95.05%) but violates C6 for 14 steps.
R3 violates C6 for 48 steps. R6 violates C6 for 4 steps; its post-closure spare
response-time cover is infeasible and logged. R5 retains battery, return, queue
and zone safety through four failures; spare latency and freshness recovery are
reported independently. The ideal spacing formula is not a transient guarantee.

The plan asks for set-cover verification against exhaustive search; the solver
itself enumerates subsets, and tests independently verify minimum cardinality.
The plan's C6 CSV signal uses excess queue size, so its satisfying robustness has
the opposite sign. C4's window ends at the next failure; it does not attribute
later independent coverage dips to the earlier failure.

On the 20 seeded maps, mean steady coverage is 78.8% for round-robin and 98.6%
for the shared loop. Three round-robin maps and six shared-loop maps meet all
safety and queue checks. One round-robin layout and five shared-loop layouts are
infeasible. The fourth R5 failure recovers after 971 s, beyond the 300 s limit;
the earlier failure windows do not recover before the next event. These are
measured limitations, not successful freshness or latency guarantees.

Random-map coverage and age summaries exclude infeasible configurations: 19 maps
contribute to round-robin statistics and 15 to shared-loop statistics. Pass counts
use all 20 attempted maps. These unequal samples do not support a paired effect
estimate; no confidence intervals or significance tests were computed.
