# Baseline configuration

Key baseline parameters and configuration values used for the thesis runs.
All values are set in `configs/baseline_v2.json` (canonical) or the
Pydantic defaults in `uav_surveil/config/parameters.py`, unless otherwise
noted.

---

## Baseline Parameters Table

| Parameter                | Value / Default      | Units         | Description                                      |
|---------------------------|---------------------|---------------|--------------------------------------------------|
| **Cell size**              | 40                  | m             | Side length of each grid cell                    |
| **Grid area (width)**      | 500                 | m             | Width of surveillance area (12 cols × Δ=40 m)    |
| **Grid area (length)**     | 500                 | m             | Length of surveillance area (12 rows × Δ=40 m)   |
| **Grid origin**            | (0, 0)              | m             | Southwest corner of the grid                     |
| **Depot**                  | (−500, 0)           | m             | Single ground station, 500 m west of grid origin |
| **Flight altitude**        | 50                  | m             | UAV cruise altitude (typical)                    |
| **Cruise speed**           | 6.0                 | m/s           | UAV cruise speed (also used as v_max)            |
| **Endurance**               | 2100                | s             | Total battery endurance (35 min)                 |
| **Usable endurance**       | 1890                | s             | After SoC floor (31.5 min)                       |
| **SoC floor (σ_min)**      | 0.1                 | fraction      | Minimum battery reserve (10%)                    |
| **Prelaunch margin (ε)**   | 0.05                | fraction      | Stage 4 ETA pre-launch SoC margin (`battery.prelaunch_margin`) |
| **Revisit cap (Θ)**        | 180                 | s             | Max time between cell observations (C1 proxy)    |
| **Spare floor (β)**        | 0.2                 | fraction      | Minimum fraction of fleet as spares              |
| **Cost coeff. (C_L)**      | 1.0                 | -             | Penalty for launch (active) UAVs                 |
| **Cost coeff. (C_S / C_rot)** | 1.2              | -             | Penalty for rotation spare UAVs (C_rot = C_S)    |
| **Cost coeff. (C_cont)**   | 1.5                 | -             | Penalty for the contingency spare (C_cont = 1.25·C_S) |
| **Battery swap time (Δ_swap)** | 60              | s             | Hot-swap time at depot                           |
| **Batch size (b)**         | 4                   | UAVs          | UAVs per departure batch                         |
| **Batch period (T_batch)** | 150                 | s             | Time between successive batch departures         |
| **Intra-batch stagger**    | 5                   | s             | Gap between launches within a batch              |
| **Bridge tick (T_bridge)** | 30                  | s             | Stage-5 bridging re-evaluation cadence           |
| **Bridge K cap**           | 2                   | inserts/tick  | Max cell–UAV insertions per bridging tick         |
| **Bridge age guard (δ_age)** | 60                | s             | Min cell age before it is eligible as urgent     |
| **Max detour ratio**       | 0.2                 | fraction of L_u | Geometry guard on a bridging detour            |
| **Cell priority**          | 1.0                 | -             | Uniform for all cells in baseline                |
| **No-fly zones**           | None                | -             | Ignored in baseline                              |
| **Failures**                | Optional            | -             | Single-UAV failure scenario available            |

---

## Explanatory Notes

- **Grid area** can be changed for scenario studies; 500×500 m is the
  baseline default (`configs/baseline_v2.json`; other scenarios in
  `configs/` vary this).
- **Depot** is a fixed offset from the grid, not the grid origin itself —
  ferry distance is the straight-line distance from `(depot_x, depot_y)`
  to a route's nearest grid cell.
- **SoC floor** is enforced everywhere to ensure battery safety (C2 proxy).
- **Prelaunch margin (ε)** is the safety margin added to the distance-aware
  return threshold when deciding whether to launch a rotation spare ahead
  of an incoming swap (`architecture.md` §4.3).
- **Revisit time (Θ)** is the maximum allowed time between cell
  observations (C1 proxy).
- **Spare floor (β)** ensures operational resilience (C3 proxy).
- **Cost coefficients** are dimensionless and can be tuned for sensitivity
  analysis. `C_rot`/`C_cont` only apply when `optimization.use_enhanced_fleet`
  is true (baseline default); see `optimize_fleet_from_config`.
- **Bridge K cap / age guard / detour ratio** bound Stage-5 bridging cost
  and reach; see `architecture.md` §4.4.
- **Failures, no-fly zones, and variable cell priorities** are not included
  in the baseline but can be added for advanced scenarios.

C1–C4 are policy monitors with STL-style intent, not formal STL robustness.
