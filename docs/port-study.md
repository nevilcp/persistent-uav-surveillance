# Port study

Recorded simulation results, verified on 2026-11-15 using Python 3.14.4 and
simulation/study commit `cce6ca4`. The configuration is `configs/port.json`:
1200 m × 1200 m, 40 m cells, 30 m sensing radius, 6 m/s cruise speed,
Θ = 180 s, 2100 s battery endurance (1890 s usable), 60 s turnaround and
2 service slots per depot. Shared-loop runs add 3 rotation UAVs per serving depot
above the Little's-law floor. R3 uses a fixed fleet of 120; loop-mode fleets are
computed, so comparisons also change fleet size.

Reproduce from the repository root with `python scripts/run_port_study.py` after
installing the project with its `plot` extra. Runs last 4800 s at a 1 s step;
steady-state coverage is averaged over samples with t > 1200 s. Raw metrics,
events, summaries and figures are generated under `results/port/` and are not
tracked. Preserve those outputs and the code/configuration revision when archiving
an experiment. C4 in this snapshot was recomputed from the saved time series
using the first-restoration definition implemented in `cce6ca4`.

R0 uses the straight return metric; R1 tests each depot alone with 6 service slots
and a relaxed spare-response threshold; R2 removes obstacles; R3 uses round-robin;
R4 uses the shared loop; R5 adds four failures; R6 closes the north depot. R1 is
selected by highest steady coverage, breaking ties by lowest worst cell age.

C5–C7 columns count violating monitor steps. C2 margin is a battery fraction above
the reserve; C3 counts late alarms out of all alarms, including rotation alarms.
The combined verdict covers C2/C5/C6/C7 at sampled times and excludes freshness,
launch latency and recovery. It is not a continuous-time safety proof. Transit
share is the fraction of airborne UAV-steps spent outside patrol.

| Run | UAVs (fly+rot+spare) | L (km) | Steady cov. (%) | Longest wait (s) | C2 min margin | C3 late / alarms | C5 | C6 | C7 | Transit share | C2, C5-C7 met |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| R0 | 57 (39+15+3) | 40.3 | 97.9 | 378 | 0.016 | 15 / 175 | 0 | 0 | 0 | 0.16 | yes |
| R1_east | 51 (39+11+1) | 40.3 | 95.1 | 682 | 0.020 | 113 / 159 | 0 | 14 | 0 | 0.22 | no |
| R1_north | 51 (39+11+1) | 40.3 | 92.7 | 576 | 0.019 | 109 / 171 | 0 | 0 | 0 | 0.22 | yes |
| R1_west | 52 (39+12+1) | 40.3 | 94.6 | 660 | 0.020 | 75 / 175 | 0 | 0 | 0 | 0.23 | yes |
| R2 | 51 (35+14+2) | 36.0 | 98.9 | 362 | 0.020 | 9 / 155 | 0 | 0 | 0 | 0.15 | yes |
| R3 | 120 (87+27+6) | - | 78.5 | 1936 | 0.019 | 66 / 150 | 0 | 48 | 0 | 0.12 | no |
| R4 | 57 (39+15+3) | 40.3 | 99.2 | 245 | 0.020 | 4 / 178 | 0 | 0 | 0 | 0.15 | yes |
| R5 | 57 (39+15+3) | 40.3 | 97.7 | 372 | 0.020 | 2 / 177 | 0 | 0 | 0 | 0.15 | yes |
| R6 | 57 (39+15+3) | 40.3 | 97.7 | 420 | 0.020 | 46 / 172 | 0 | 4 | 0 | 0.16 | no |

Longest wait: largest cell age after the first 1200 s and after every cell has been seen once. Coverage: mean over the steady state.

Best single base (R1): R1_east.

## Design predictions

Prior estimates are hypotheses, not acceptance targets. The A_1 row evaluates the
ideal-spacing formula at the implemented loop length and fleet size; it is not a
measured revisit time. The final row is an observed maximum during 1800 ≤ t < 2100 s.

| Quantity | Prior estimate | Implemented / observed |
|---|---:|---:|
| Loop length L (km) | 35 | 40.3 |
| Total UAVs, shared loop | 43 | 57 |
| A_1 = L / ((n_fly - 1) v), at the simulated L and n_fly (s) | 177 | 177 |
| Longest wait after the first breakdown, R5 (s) | 177 | 226 |

## Breakdown and closure

Alarm latency includes elapsed waits for alarms still unanswered at run end; those
values are lower bounds on their eventual launch latency. C4 is measured from the
failure to the first restoration of max_age ≤ Θ after staleness. Each window ends
at the next failure or run end. “Not reached” is an observation-window limit, not
evidence that recovery can never occur.

**R5**: maximum alarm latency at run end 2100 s.

- breakdown at 1800 s (UAV 30): recovery not reached (limit 300 s, missed)
- breakdown at 2100 s (UAV 43): recovery not reached (limit 300 s, missed)
- breakdown at 2400 s (UAV 46): recovery not reached (limit 300 s, missed)
- breakdown at 2700 s (UAV 19): recovery 971 s (limit 300 s, missed)

**R6**: maximum alarm latency at run end 242 s.

- t = 1800 s closure: base 2
- t = 1800 s resize: infeasible: no set of bases reaches every loop point within T_resp = 150

## Random maps

20 layouts generated with seeds 0–19, with zones covering about 8% of the cells.
One deterministic run per method and layout; repeated runs with the same seed
are not independent replications. Min/mean/max summaries exclude infeasible
setups: 19 layouts for R3 and 15 for R4. Pass counts use all 20 attempted layouts.
The unequal samples and differing fleet sizes preclude interpreting these means
as a controlled paired treatment effect. No confidence intervals or significance
tests are reported; generalisation beyond these layouts has not been established.

| | R3 round-robin | R4 shared loop |
|---|---:|---:|
| UAVs (min / mean / max) | 120.0 / 120.0 / 120.0 | 51.0 / 53.4 / 56.0 |
| Steady coverage % (min / mean / max) | 73.4 / 78.8 / 82.8 | 96.9 / 98.6 / 99.6 |
| Longest wait s (min / mean / max) | 1585.0 / 2024.6 / 2709.0 | 235.0 / 378.6 / 423.0 |
| Maps meeting C2, C5-C7 | 3 | 6 |
| Maps infeasible | 1 | 5 |
