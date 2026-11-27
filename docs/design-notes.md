# Design notes

## Airspace and routes

The grid uses whole cells. A cell is surveyed if its centre lies outside the
interior of every buffered no-fly zone. Any airborne UAV refreshes the cells
whose centres are within sensing range, including during transit.

The planner uses straight segments where clear and finds detours with Dijkstra's
algorithm on an eight-neighbour graph of free cell centres. A route can graze a
zone boundary within numerical tolerance. [Airspace](../uav_surveil/airspace.py)

Each free cell belongs to the depot with the shortest flyable path to it.
Every occupied sector gets at least one patrol UAV, and the rest are allocated
by cell count. Within each sector, the planner splits or joins horizontal runs
of free cells using Choset's boustrophedon decomposition. It sweeps the resulting
regions and deals the ordered cells round-robin to the patrol routes. The
initial endurance check budgets the waypoint sequence and both ferry legs.
In flight, patrols repeat the route until the return threshold is reached.
[Grid decomposition](../uav_surveil/grid.py), [routing](../uav_surveil/routing.py)

## Dispatch and depot service

The fleet split sets aside at least one launch batch for rotation and a
configured fraction for failures. Both pools are split among depots by route
count. Each depot staggers its initial departures independently.
[Fleet allocation](../uav_surveil/fleet.py)

Dispatch estimates the charge that would remain on arrival at the nearest open
depot. By default, it requests a replacement at 16% and sends the incumbent
home at 12%, two percentage points above the 10% reserve. The incumbent
returns even if the replacement has yet to launch. The replacement starts at
the first waypoint of the full route.
[Dispatch rules](../uav_surveil/dispatch.py), [simulation](../uav_surveil/simulation.py)

Rotation alarms draw from the route's home depot. Failure alarms use a separate
contingency pool and can draw from another open depot. Arrivals queue in order
when service slots are full. A depot configured with zero slots has unlimited
service capacity. [Depot service](../uav_surveil/depots.py)

## Failures and closure

After a failure, surviving patrols can add overdue or soon-due cells from the
failed route. For each cell, the controller picks the insertion that adds the
least distance and fits within usable endurance. It removes those waypoints
when the spare reaches the first waypoint of the failed route.
[Failure handling](../uav_surveil/failure.py)

When a depot closes, its stored UAVs become unavailable. The controller reassigns
each affected route to the open depot with the shortest mean flyable distance
to its waypoints and diverts aircraft returning to the closed depot.
[Simulation](../uav_surveil/simulation.py)

## Measurements and limits

Local recovery ends when every cell on the failed route has been seen again.
Region-wide recovery requires all surveyed cells to be fresh in the same sample
within the failure window. Stale cells elsewhere can keep that measure unmet.
[Measurement protocol](../README.md#measurement-protocol)

The simulation assumes identical aircraft, exact positions, constant speed,
instantaneous turns, deterministic battery use and static obstacles. It checks
obstacle clearance on flight segments and has no aircraft-to-aircraft collision
avoidance. Monitor results describe the configured sample times.

To regenerate the baseline plots, install the `plot` extra and run
`python scripts/run_scenarios.py --out results/baseline`.
