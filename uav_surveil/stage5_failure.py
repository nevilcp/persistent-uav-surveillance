"""Stage 5 - Failure handling and bridging policy.

Provides a minimal, testable implementation scaffold:
- FailureTrigger: config-driven triggers (off|time|position|soc)
- FailureManager: orchestrates failure, contingency promotion, handover
- BridgeManager: placeholder for greedy k-NN insertion (to be filled next)

Disabled by default via config.failure.enabled.
"""

from __future__ import annotations

import contextlib
from dataclasses import dataclass
from typing import TYPE_CHECKING

from .core.uav import UAV, UAVState

if TYPE_CHECKING:
    from .core.cell import Cell
    from .gss.simulation import GSSSimulation


@dataclass
class FailureTrigger:
    kind: str
    uav_id: str | None = None
    t_s: float | None = None
    x: float | None = None
    y: float | None = None
    soc_threshold: float | None = None
    fired_once: bool = False

    def check(self, sim: GSSSimulation) -> bool:
        if self.fired_once:
            return False
        if self.kind == "time" and self.t_s is not None and self.uav_id is not None:
            return sim.metrics.current_time >= float(self.t_s)
        if (
            self.kind == "position"
            and self.uav_id is not None
            and self.x is not None
            and self.y is not None
        ):
            u = self._u(sim)
            return (
                u is not None
                and ((u.x - self.x) ** 2 + (u.y - self.y) ** 2) ** 0.5 < 2.0
            )
        if (
            self.kind == "soc"
            and self.uav_id is not None
            and self.soc_threshold is not None
        ):
            u = self._u(sim)
            return u is not None and u.soc <= float(self.soc_threshold)
        return False

    def _u(self, sim: GSSSimulation) -> UAV | None:
        for u in sim.uavs:
            if u.id == self.uav_id:
                return u
        return None


class BridgeManager:
    """Stage-5a bridging logic (bounded greedy k-NN with micro-guards)."""

    def __init__(self, sim: GSSSimulation) -> None:
        self.sim = sim
        self.enabled: bool = False
        self.failed_id: str | None = None
        self.last_tick_time: float = 0.0
        self._assigned_cells: dict[str, set[str]] = {}  # failed_id -> set(cell_id)
        self._events_path: str | None = None

    def enable(self, failed_id: str) -> None:
        self.enabled = True
        self.failed_id = failed_id
        if failed_id not in self._assigned_cells:
            self._assigned_cells[failed_id] = set()
        # Wire events path from simulation if available
        self._events_path = getattr(self.sim, "_events_path", None)

    def disable(self) -> None:
        self.enabled = False
        self.failed_id = None

    def tick(self) -> None:
        if not self.enabled:
            return
        cfg = self.sim.config.failure.bridge_policy
        now = self.sim.metrics.current_time
        if now - self.last_tick_time < cfg.bridge_tick_s:
            return
        self.last_tick_time = now

        failed_id = self.failed_id
        if not failed_id:
            return

        # Determine urgent orphan cells by age
        orphan_cells = self._get_orphan_cells(failed_id)
        if not orphan_cells:
            return
        max_gap = self.sim.config.stl.max_revisit_gap
        urgent = [
            c for c in orphan_cells if c.age(now) >= max_gap - cfg.realloc_age_guard_s
        ]
        # Oldest first
        urgent.sort(key=lambda c: -c.age(now))

        # Candidate active neighbors
        active = [
            u
            for u in self.sim.uavs
            if u.state == UAVState.ON_MISSION and not u.is_failed
        ]

        speed = self.sim.config.uav.cruise_speed
        usable_range = self.sim._usable_range_m()
        soc_margin = self.sim.config.battery.prelaunch_margin
        soc_threshold = self.sim.config.battery.soc_return_threshold

        # Bound the detour-insertion search horizon (candidate indices ahead
        # of each UAV's current waypoint) to keep the per-tick cost bounded.
        DETOUR_HORIZON = 8

        from math import hypot

        # Process the full urgent list but stop once K cells have actually
        # been inserted this tick (cells with no feasible candidate are
        # skipped without consuming the cap).
        inserted = 0
        for cell in urgent:
            if inserted >= cfg.realloc_k:
                break
            if cell.id in self._assigned_cells.get(failed_id, set()):
                continue

            best = None  # (delta_t, uav, insert_index)
            for u in active:
                # Cap per UAV
                if u.temp_assignments.get(failed_id, 0) >= cfg.max_inserts_per_uav:
                    continue
                # Hold time hysteresis
                if now - getattr(u, "last_insert_time", 0.0) < cfg.min_hold_time_s:
                    continue
                if not hasattr(u, "_waypoints") or not hasattr(u, "_waypoint_idx"):
                    continue
                wp_idx = u._waypoint_idx
                waypoints = u._waypoints
                if wp_idx >= len(waypoints):
                    continue

                route = u.route_list[0] if u.route_list else None
                l_u = route.loop_time if route and route.loop_time else None
                j_max = min(wp_idx + DETOUR_HORIZON, len(waypoints))

                for j in range(wp_idx, j_max):
                    prev_xy = (u.x, u.y) if j == wp_idx else waypoints[j - 1]
                    next_xy = waypoints[j]
                    d_prev_next = hypot(
                        prev_xy[0] - next_xy[0], prev_xy[1] - next_xy[1]
                    )
                    d_prev_cell = hypot(prev_xy[0] - cell.x, prev_xy[1] - cell.y)
                    d_cell_next = hypot(cell.x - next_xy[0], cell.y - next_xy[1])
                    d_extra = max(0.0, d_prev_cell + d_cell_next - d_prev_next)
                    delta_t = d_extra / speed if speed > 0 else float("inf")

                    # Geometry guard: detour time bounded by a fraction of
                    # the UAV's own loop time.
                    if l_u and delta_t > cfg.max_detour_ratio * l_u:
                        continue

                    # SoC feasibility: margin + added distance budget
                    if usable_range <= 0:
                        continue
                    added_soc = d_extra / usable_range
                    if u.soc < soc_threshold + soc_margin + added_soc:
                        continue

                    if best is None or delta_t < best[0]:
                        best = (delta_t, u, j)

            if best is None:
                continue
            _, u_best, idx_best = best

            # Insert detour waypoint at the best index found
            u_best._waypoints.insert(idx_best, (cell.x, cell.y))
            inserted += 1
            # Track as temporary assignment
            u_best.temp_assignments[failed_id] = (
                u_best.temp_assignments.get(failed_id, 0) + 1
            )
            u_best.last_insert_time = now
            self._assigned_cells[failed_id].add(cell.id)
            # Claim cell to avoid duplicates
            self.sim._claimed_cells.add(cell.id)
            self.sim._cell_claim_times[cell.id] = now
            # Telemetry increments
            with contextlib.suppress(Exception):
                self.sim.metrics.bridge_inserts_count += 1
            # Event log
            with contextlib.suppress(Exception):
                if self._events_path:
                    import csv as _csv

                    with open(self._events_path, "a", newline="") as _f:
                        _w = _csv.writer(_f)
                        _w.writerow(
                            [
                                f"{now:.0f}",
                                "bridge_insert",
                                u_best.id,
                                f"cell={cell.id}",
                            ]
                        )

            # Stop early if contingency is close
            if self._eta_contingency(failed_id) <= cfg.realloc_horizon_s:
                break

    # -------------------------------------------------------------------------
    # Orphan tracking util methods
    # -------------------------------------------------------------------------
    def _get_orphan_cells(self, failed_id: str) -> list[Cell]:
        # Build orphan list as the failed UAV's remaining loop from failure tail index
        failed = next((u for u in self.sim.uavs if u.id == failed_id), None)
        if failed is None or not failed.route_list:
            return []
        seq = failed.route_list[0].cell_sequence
        idx = failed.tail_index_at_failure or 0
        ids = seq[idx:] + seq[:idx]
        return [self.sim.cell_lookup[cid] for cid in ids]

    def _eta_contingency(self, failed_id: str) -> float:
        # Rough ETA of contingency to the first orphan cell
        cont = next(
            (
                u
                for u in self.sim.uavs
                if u.is_contingency and u.state == UAVState.ON_MISSION
            ),
            None,
        )
        if cont is None:
            return float("inf")
        failed = next((u for u in self.sim.uavs if u.id == failed_id), None)
        if failed is None or not failed.route_list:
            return float("inf")
        seq = failed.route_list[0].cell_sequence
        idx = failed.tail_index_at_failure or 0
        if not seq:
            return float("inf")
        first_cell = self.sim.cell_lookup[seq[idx % len(seq)]]
        from math import hypot

        d = hypot(cont.x - first_cell.x, cont.y - first_cell.y)
        v = self.sim.config.uav.cruise_speed
        return d / v if v > 0 else float("inf")


class FailureManager:
    """Stage-5 orchestrator for failure handling and takeover/bridge."""

    def __init__(self, sim: GSSSimulation) -> None:
        self.sim = sim
        cfg = sim.config.failure
        self.enabled = bool(getattr(cfg, "enabled", False))
        trig_cfg = cfg.trigger
        self.trigger = (
            FailureTrigger(
                kind=trig_cfg.kind,
                uav_id=trig_cfg.uav_id,
                t_s=trig_cfg.t_s,
                x=trig_cfg.x,
                y=trig_cfg.y,
                soc_threshold=trig_cfg.soc_threshold,
            )
            if self.enabled and trig_cfg.kind != "off"
            else None
        )
        self.bridge = BridgeManager(sim)
        self._handover_started: bool = False
        self._handover_done: bool = False
        # markers: t_fail, t_takeover_start, t_handover
        if self.enabled:
            sim._failure_markers = {}

    def tick(self) -> None:
        if not self.enabled:
            return
        # Check trigger
        if self.trigger and self.trigger.check(self.sim):
            self._handle_failure()
            self.trigger.fired_once = True
        # Bridge tick and possible handover progression
        self.bridge.tick()
        # Handover once contingency ETA is within grace: freeze bridge, revert neighbors
        if self.bridge.enabled and self.bridge.failed_id:
            eta = self.bridge._eta_contingency(self.bridge.failed_id)
            if eta <= self.sim.config.failure.bridge_policy.handover_grace_s:
                self._handover(self.bridge.failed_id)
                self.bridge.disable()
                self._handover_done = True
                # After handover: disable per-step bridge work to avoid overhead
                self.enabled = False

    # -------------------------------------------------------------------------
    # Internal helpers
    # -------------------------------------------------------------------------
    def _handle_failure(self) -> None:
        failed_uav = self._get_uav(self.trigger.uav_id if self.trigger else None)
        if failed_uav is None:
            return
        # Mark failure time
        self.sim._failure_markers["t_fail"] = self.sim.metrics.current_time
        # Export failure meta and failed-route cell list for exact orphan analysis
        try:
            import csv as _csv
            import os

            os.makedirs("results", exist_ok=True)
            if hasattr(self.sim, "_simulation_info") and self.sim._simulation_info:
                info = self.sim._simulation_info
                meta_path = f"results/{info['base_name']}_failure_meta.csv"
                with open(meta_path, "w", newline="") as f:
                    w = _csv.writer(f)
                    w.writerow(["t_fail", "failed_uav_id"])
                    w.writerow([f"{self.sim.metrics.current_time:.0f}", failed_uav.id])
                if failed_uav.route_list:
                    cells_path = f"results/{info['base_name']}_failed_route_cells.csv"
                    with open(cells_path, "w", newline="") as f:
                        w = _csv.writer(f)
                        w.writerow(["cell_id"])
                        for cid in failed_uav.route_list[0].cell_sequence:
                            w.writerow([cid])
                print("🗂️  Failure meta exported for orphan analysis")
        except (
            Exception
        ) as _e:  # noqa: BLE001 - optional export, must not abort failure handling
            print(f"⚠️  Failure meta export failed: {_e}")
        # Mark as failed and freeze tail index; `_fly_home` (RTB) is
        # derived from state so this alone stops any RTB motion.
        failed_uav.is_failed = True
        failed_uav.state = UAVState.FAILED
        # Expose ids to sim for selective SoC logging
        self.sim._failed_id = failed_uav.id
        if not hasattr(failed_uav, "_waypoint_idx"):
            failed_uav._waypoint_idx = 0
        failed_uav.tail_index_at_failure = int(failed_uav._waypoint_idx)
        # Orphan segment is just the remaining cells from route_list[0]
        orphan_cells: list[str] = []
        if failed_uav.route_list:
            seq = failed_uav.route_list[0].cell_sequence
            idx = failed_uav.tail_index_at_failure or 0
            orphan_cells = seq[idx:] + seq[:idx]
        # Clear any existing claims for these cells in sim
        for cid in orphan_cells:
            self.sim._claimed_cells.discard(cid)
            self.sim._cell_claim_times.pop(cid, None)
        # Promote contingency (launch the reserved spare)
        contingency = self._promote_contingency()
        if contingency is None:
            return
        # Assign takeover route (same loop, start at tail index)
        if failed_uav.route_list:
            contingency.route_list = [failed_uav.route_list[0]]
            contingency._waypoints = contingency.route_list[0].get_waypoints(
                self.sim.cell_lookup
            )
            contingency._waypoint_idx = failed_uav.tail_index_at_failure or 0
        contingency.state = UAVState.ON_MISSION
        contingency.is_active = True
        # Once promoted, treat contingency as a normal rotation member thereafter
        contingency.is_contingency = False
        contingency.launch_time = self.sim.metrics.current_time
        # Expose contingency id for SoC logging
        self.sim._contingency_id = contingency.id
        self.sim._failure_markers["t_takeover_start"] = self.sim.metrics.current_time
        # Enable bridge mode until contingency is near or inside horizon
        self.bridge.enable(failed_uav.id)
        # Emit events for trigger/promotion
        with contextlib.suppress(Exception):
            path = getattr(self.sim, "_events_path", None)
            if path:
                import csv as _csv

                with open(path, "a", newline="") as _f:
                    w = _csv.writer(_f)
                    w.writerow(
                        [
                            f"{self.sim.metrics.current_time:.0f}",
                            "failure_trigger",
                            failed_uav.id,
                            "",
                        ]
                    )
                    w.writerow(
                        [
                            f"{self.sim.metrics.current_time:.0f}",
                            "contingency_promoted",
                            contingency.id,
                            "",
                        ]
                    )
        print(
            f"🛑 FAILURE: UAV {failed_uav.id} at "
            f"t={self.sim.metrics.current_time:.0f}s → freezing & reallocating"
        )
        route_id = failed_uav.route_list[0].id if failed_uav.route_list else ""
        print(
            f"🛡️  CONTINGENCY LAUNCHED: UAV {contingency.id} taking over "
            f"route {route_id} from tail index {contingency._waypoint_idx}"
        )

    def _handover(self, failed_id: str) -> None:
        """Revert temporary inserts and rebuild neighbor waypoints after takeover."""
        # Clear outstanding claims for assigned cells (if still present)
        assigned = self.bridge._assigned_cells.get(failed_id, set())
        for cid in assigned:
            self.sim._claimed_cells.discard(cid)
            self.sim._cell_claim_times.pop(cid, None)
        # Rebuild waypoints for any UAV with temporary assignments
        for u in self.sim.uavs:
            if u.temp_assignments.get(failed_id, 0) > 0:
                if u.route_list:
                    u._waypoints = u.route_list[0].get_waypoints(self.sim.cell_lookup)
                    # Adjust waypoint index to safest bound
                    u._waypoint_idx = min(
                        getattr(u, "_waypoint_idx", 0), len(u._waypoints)
                    )
                u.temp_assignments.pop(failed_id, None)
                u.last_insert_time = self.sim.metrics.current_time
        self.sim._failure_markers["t_handover"] = self.sim.metrics.current_time
        print(
            f"✅ HANDOVER COMPLETE at t={self.sim.metrics.current_time:.0f}s "
            f"– neighbors reverted, steady state resumed (16+4+0)"
        )
        # Log event
        with contextlib.suppress(Exception):
            path = getattr(self.sim, "_events_path", None)
            if path:
                import csv as _csv

                with open(path, "a", newline="") as _f:
                    w = _csv.writer(_f)
                    w.writerow(
                        [
                            f"{self.sim.metrics.current_time:.0f}",
                            "reentry_detected",
                            failed_id,
                            "handover_complete",
                        ]
                    )

    def _get_uav(self, uav_id: str | None) -> UAV | None:
        if uav_id is None:
            return None
        for u in self.sim.uavs:
            if u.id == uav_id:
                return u
        return None

    def _promote_contingency(self) -> UAV | None:
        # Find spare with is_contingency True
        for u in self.sim.uavs:
            if u.state == UAVState.SPARE and u.is_contingency:
                return u
        # If none are SPARE (already used), do nothing
        return None
