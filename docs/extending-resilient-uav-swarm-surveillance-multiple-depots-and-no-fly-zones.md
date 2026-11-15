# Extending Resilient UAV Swarm Surveillance: Multiple Depots and No-Fly Zones

This report proposes an extension to a published method for watching a region with a swarm of drones, so that it works with several launch bases and with areas where flying is forbidden.

## Overview

A swarm of drones (UAVs, unmanned aerial vehicles) can keep a region under continuous watch if some drones fly while others stand ready to replace them when their batteries run low. The starting point of this report is Abhirva et al. (*Resilient & Uninterrupted Surveillance by UAV Swarm*, IEEE TAES 2026, referred to as "the paper" below), which presents a control framework for this, run by a central computer called the Ground Station Monitor (GSM). The framework keeps the region well covered even when drones fail.

The paper makes two simplifying assumptions: all drones start from **one base**, and every distance is a **straight line**. Real operations rarely look like that, so this report proposes removing both:

- **Extension A: several depots (bases) outside the surveillance region**, instead of one.
- **Extension B: no-fly zones (NFZs) inside the region**, meaning areas drones must not enter.

The cheapest first step is to replace straight-line distances with shortest *flyable* distances around a single no-fly zone. This alone is expected to expose a battery-safety flaw in the paper's current rule for when a drone should return to base.

## Background: the surveillance problem

This section sets out the building blocks of the problem, on which everything that follows depends. The goal is to keep every part of an area recently observed, using drones that can only fly for a limited time.

- **Grid cells.** The surveillance region is split into a grid of small squares called cells. In the paper's setup the region is 500 m × 500 m with 40 m cells, so the largest block of whole cells is 12 × 12 = 144 cells (480 m × 480 m), as in the baseline implementation.
- **Freshness and coverage.** A cell is *fresh* if a drone observed it within the last Θ seconds (the paper uses Θ = 180 s). A drone observes a cell while the cell's centre lies within its sensing radius (30 m in the baseline implementation, against 40 m cells), so it also sees neighbouring cells it passes close to. **Coverage** is the fraction of cells that are fresh at a given moment. Higher coverage means better surveillance.
- **Routes.** Each flying drone follows a *route*: a fixed, repeating list of cells (waypoints) to visit.
- **Depot.** A depot is a ground base where drones take off, land and are made ready for their next flight. The time a depot needs to ready one drone is the *turnaround time*, written τ (tau).
- **Ferry.** Drones cannot patrol while travelling between the depot and the region. This travel is called *ferrying*.
- **Battery limit.** A drone can stay airborne for only a limited usable time, T\_use (1890 s in the paper). Before that runs out it must return to a depot, and another drone must take over its route.

In symbols, let 𝒞 be the set of cells being watched and t\_c the last time a drone was inside cell c. Coverage at time t is then:

$$
\mathrm{Cov}(t) = \frac{\left|\{\, c \in \mathcal{C} : t - t_c \le \Theta \,\}\right|}{|\mathcal{C}|}
$$

The drones are divided into three groups so that coverage never has to stop:

| Group | Role |
| --- | --- |
| Flying | Drones currently patrolling their routes |
| Rotation | Replacement drones that take over a route when its drone must return |
| Spare | Reserve drones, used when a drone fails |

The three groups add up to the whole fleet:

$$
N = n_{fly} + n_{rotate} + n_{spare}
$$

The paper's fleet is 21 = 16 + 4 + 1.

The question the paper asks (its Problem 3) is: given N battery-limited drones, what steady-state coverage can be *guaranteed*, even if some drones fail?

## The baseline paper: how it works

In the paper, the GSM runs once per second (1 Hz) and coordinates six stages. Each stage can be read as the answer to one practical question.

1. **Fleet split.** *How many drones go into each group?* A small optimisation problem called an ILP (integer linear program) divides the N drones into flying, rotation and spare groups. An ILP chooses whole-number quantities, here drone counts, that satisfy a list of constraints.
2. **Routes.** *Which cells does each drone visit?* Cells are first put in **boustrophedon** order, a back-and-forth sweep like ploughing a field. They are then dealt out **round-robin**, like cards to players: cell 1 to route 1, cell 2 to route 2, and so on. Each route therefore ends up spread over the whole region.
3. **Launch schedule.** *When does each drone take off?* Drones leave the depot in staggered batches so that the depot is never overloaded.
4. **Battery-aware dispatch.** *When does a drone go home?* A drone returns when its battery reaches a threshold that depends on how far it is from the depot. A rotation drone then takes over its route.
5. **Failure handling.** *What if a drone breaks down?* Cells whose freshness is about to run out are first absorbed by neighbouring routes. A spare drone then takes over the failed route.
6. **Verification.** *Is the system staying safe?* Four rules, written in a formal language called **STL** (signal temporal logic), are checked continuously while the system runs. STL states rules about behaviour over time, such as "the battery must *always* stay above a floor". The four clauses are:
   - C1, freshness: cells are observed often enough.
   - C2, battery floor: no drone's battery drops below its safe minimum.
   - C3, spare-launch latency: a spare launches quickly after a failure.
   - C4, recovery time: coverage recovers within a bounded time after a failure.

With the paper's fleet of N = 21 drones (16 flying, 4 rotation, 1 spare), mean coverage is 85.2%. Losing one drone costs only 0.33 percentage points of coverage. These are the paper's figures. This repository's reimplementation gives 81.6% mean and 88.2% steady-state coverage, and with 4 rotation drones it misses clause C3: 20 of 32 replacements launch late.

## Where the baseline falls short, and what changes

Four assumptions in the paper are convenient but limiting. The two proposed extensions target them.

- **One depot, one worst-case ferry distance.** The paper computes ferry time from the *largest* depot-to-region distance and applies it to every route. This is safe but wasteful: routes close to the depot are charged as if they were far away.
- **Straight-line distances everywhere.** The paper uses straight-line (Euclidean) distance to compute route length, the battery threshold for returning home, the predicted battery on arrival, and the cost of repairing routes after a failure. This is accurate only when nothing blocks the way.
- **An implicit one-at-a-time depot.** The depot's limit of handling one drone at a time is never stated. It appears only through the rule that launches must be spaced at least τ apart.
- **Spread-out routes.** Round-robin spreads every route over the whole region, and the failure repair quietly relies on this, because a failed route's cells then have neighbouring routes everywhere.

**What stays the same:** the six-stage architecture, the round-robin and boustrophedon routing baseline, the ILP fleet split and the STL runtime checks are all kept. Choset boustrophedon decomposition followed by round-robin assignment is the primary path planner. The shared-loop planner has been removed. Randomised assignment is a possible later extension to this primary method.

**What changes:**

1. the distance model (shortest flyable paths instead of straight lines);
2. extra constraints in the fleet-split ILP;
3. how cells are assigned to routes;
4. new STL clauses.

## Extension A: multiple depots outside the region

Let the depots be D₁, …, Dₘ. Each depot must lie outside the surveillance region A, exactly 500 m from its nearest point on the outer boundary ∂A: min_{q ∈ ∂A} ‖Dₘ − q‖₂ = 500 m. This geometric setback is measured to the region boundary, not to a cell centre or an NFZ boundary; ferry costs still use shortest flyable distance d_G. Each route is given a *launch depot* and a *recovery depot*, which may be different. This fits the paper's own wording of a "depot-to-depot patrol loop". With several depots, drones can start closer to their routes, but the system must now decide which depot serves which route and how many drones each depot holds.

### A1. Per-route ferry legs instead of one worst case

The time a drone u spends in the air on its route is now computed from that route's own distances:

$$
T_{airborne}(u) = \frac{d(D_{s(u)}, w_{u,1}) + D_{loop}(u) + d(w_{u,m_u}, D_{e(u)})}{v_{max}} \le T_{use}
$$

In words: the flight out from the launch depot to the route's first waypoint, plus one lap of the route, plus the flight back from the last waypoint to the recovery depot, all divided by cruise speed v (top speed would understate the time), must fit within the usable battery time T\_use.

*Why:* the paper charges every route the farthest ferry distance. In this formulation, per-route legs give tighter, more honest endurance budgets, and the benefit of adding depots then shows up directly in the numbers.

### A2. Ferry time costs rotation drones, not revisit time

Because a replacement is launched early (the paper's pre-launch rule), a route stays covered during a swap. As long as a rotation drone is ready at every swap, ferry and turnaround time therefore do not make cells go stale. Instead, they determine *how many extra drones are in transit* at any moment. **Little's law** is a simple rule from queueing theory: in a steady system, the average number of items in the system equals the rate at which they arrive times the time each one stays. It is used here to size the rotation pool:

$$
n_{rotate} \gtrsim n_{fly}\,\frac{2\bar T_{ferry} + \tau}{T_{use} - 2\bar T_{ferry}}
$$

Here n\_fly is the number of flying drones and T̄\_ferry is the average one-way ferry time. The numerator is the time a drone spends away from its route on each cycle (out, back and turnaround). The denominator is the time it spends on the route. The more time drones lose to ferrying, the more replacements are needed.

The formula is Little's law, L = λW, written out. Here L is the number of rotation drones, λ = n\_fly / (T\_use − 2T̄\_ferry) is how often a flying drone needs replacing, and W = 2T̄\_ferry + τ is how long each replacement is tied up:

$$
L = \lambda W, \qquad \lambda = \frac{n_{fly}}{T_{use} - 2\bar T_{ferry}}, \qquad W = 2\bar T_{ferry} + \tau, \qquad L = n_{rotate}
$$

With the paper's numbers (T\_use = 1890 s, τ = 60 s, ferry about 83–186 s) this gives roughly 2–5 rotation drones, against the paper's n\_rotate = 4. Being an average, it is a floor, not a sizing rule: with 4 rotation drones the baseline implementation's pool runs empty when batch-launched drones return together. Evenly phased returns could make this floor closer to sufficient and could replace the paper's ad hoc constraint n\_rotate ≥ N\_batch (rotation drones at least as many as one launch batch). It also shows that closer depots free up drones for surveillance.

### A3. A depot-aware fleet-split ILP

The fleet-split ILP is extended with counts for each depot m: n\_fly,m, n\_rotate,m and n\_spare,m, which together sum to N. Three constraints are then added:

- the Little's-law rotation constraint from A2, applied to each depot;
- a **service-capacity limit** per depot, with κₘ service slots (drones that can be handled at once) and turnaround time τₘ. The paper's rule that launches be spaced at least τ apart is really an implicit one-at-a-time limit, and several depots relax it;
- **spare placement** as a small *set-cover* problem, meaning "choose the fewest depots so that every item is covered". Here every route must be within a response time of some depot that holds a spare. Clause C3 then becomes "the spare is on station within τ plus its ferry time".

These are written in symbols as follows, with M depots:

$$
N = \sum_{m=1}^{M}\left(n_{fly,m} + n_{rotate,m} + n_{spare,m}\right)
$$

$$
n_{rotate,m} \ge n_{fly,m}\,\frac{2\bar T_{ferry,m} + \tau_m}{T_{use} - 2\bar T_{ferry,m}} \qquad \forall m
$$

For spare placement, let y\_m = 1 if depot m holds a spare, and let T\_ferry(m,u) be the ferry time from depot m to route u. The fewest depots are chosen so that every route is within the response time T\_resp of one of them:

$$
\min \sum_{m} y_m \quad \text{s.t.} \quad \sum_{m:\,T_{ferry}(m,u) \le T_{resp}} y_m \ge 1 \;\; \forall u, \qquad n_{spare,m} \ge y_m, \qquad y_m \in \{0,1\}
$$

### A4. Partitioning the free cells among the depots

Assign each free cell to the depot with the shortest flyable distance d_G, breaking ties by depot index. Allocate flying drones in proportion to sector cell counts using largest remainders, with at least one flyer per nonempty sector and no empty routes. Apply Choset decomposition and round-robin separately within each sector. This keeps route waypoints local to their serving depot; NFZ-safe connectors may leave the sector.

The implementation uses unweighted nearest-depot sectors. Capacity-weighted balancing and a connected-sector repair are future work, so equal cell counts or equal per-depot inventories are not assumed. On the fixed 500 m-setback map the west, east and north sectors contain 309, 290 and 231 cells, served by 32, 30 and 25 flying drones respectively. Rotation and spare inventories are apportioned by route count.

### A5. Dispatch changes (Stage 4)

The following are design targets. The current controller implements nearest-open-depot return, but retains a fixed pre-launch battery margin and sends replacements to the route's first waypoint. It does not yet predict an intercept or rebalance inventories to explicit targets.

- **Return to any depot.** The return threshold uses the distance to the *nearest* depot, not a fixed one.
- **Launch the replacement at the right time.** Launch it when the outgoing drone's remaining flight time falls below the replacement's own ferry time plus a margin, instead of using one fixed pre-launch margin.
- **Fly to the right place.** The replacement flies to the outgoing drone's current waypoint, not to the start of the route.
- **Track inventory.** The GSM records how many drones each depot holds. It chooses each launch depot so that inventories drift back to their targets, with no dedicated rebalancing flights.

In symbols, drone u returns when its battery b\_u(t) falls to the reserve plus the battery needed to reach the nearest depot. The replacement r is launched when the outgoing drone's remaining flight time T\_rem(u,t) drops below the replacement's own ferry time plus a margin δ:

$$
b_u(t) \le \sigma_{min} + \frac{\min_m d(u, D_m)}{v\,T_{endure}}
$$

$$
T_{rem}(u,t) \le T_{ferry}(r) + \delta
$$

### A6. A depot outage as a correlated failure

A *correlated* failure is one where several things fail together for a shared reason, here a depot going out of service. The paper lists correlated failures as future work. An outage is handled like Stage 5: only the affected routes are reassigned to other depots, a longer ferry is accepted for a while, and battery feasibility is checked before switching.

### A7. Primary planner: Choset decomposition plus round-robin

Use Choset decomposition inside each depot sector, then deal the resulting ordered free cells round-robin among that sector's flyers. The decomposition supplies obstacle-aware coverage structure; round-robin distributes the waypoints across several routes. Every consecutive waypoint pair, including the closing leg, is connected by a shortest flyable path. The shared-loop alternative is removed.

Round-robin within sectors retains interleaving for failure bridging while limiting long cross-region travel. It is deterministic unless a separate randomisation policy is added. It gives no analytic worst-cell-age guarantee: measure coverage, cell age, battery margins and failure recovery in simulation.

### A8. Scaling up: a constant-fraction fleet per depot

To test the design at scale, the region is enlarged (more cells) and the fleet grows with it, for example to 120 drones served by three depots. The governing rule is that every depot keeps the same shares of flying, rotation and spare drones, with no fixed minimum or maximum count for any group.

**The rule.** Let N\_m be the drones assigned to depot m, so that the N\_m sum to N. The shares φ\_fly, φ\_rot and φ\_spare add up to 1. The reference global split is 87 : 27 : 6 for 120 drones; the following per-depot rounding rule is a future capacity-balancing proposal rather than the implemented route-count apportionment. Each depot's groups are then:

$$
n_{fly,m} = \mathrm{round}(\varphi_{fly}\,N_m), \qquad n_{rot,m} = \mathrm{round}(\varphi_{rot}\,N_m), \qquad n_{spare,m} = N_m - n_{fly,m} - n_{rot,m}
$$

The spare group takes the rounding remainder, so the three counts always add up to N\_m. The flying and rotation counts are rounded to the nearest whole drone, and if the remainder would fall below one, a drone moves from rotation to spare, so the spare count is never below one, so every depot keeps a spare. In the experiments this replaces the set-cover choice of spare placement in A3: with a spare at every depot, each route has one within its own sector. A depot's size N\_m is fixed first, from its service capacity κₘ or equally as in the experiments, and a future capacity-weighted partition could then follow it. The implemented A4 instead fixes geometric sectors first and apportions inventory by route count. A future weighted scheme must avoid deriving inventories and partition weights circularly.

For a future balanced-sector experiment, the target could be 120 drones split equally, 40 per depot. This table is a design target, not the allocation in the current simulation. With shares of 0.72 flying, 0.23 rotation and 0.05 spare, each depot's groups come out as follows:

| Depot | Assigned | Flying | Rotation | Spare |
| --- | --- | --- | --- | --- |
| D\_W | 40 | 29 | 9 | 2 |
| D\_E | 40 | 29 | 9 | 2 |
| D\_N | 40 | 29 | 9 | 2 |
| Total | 120 | 87 | 27 | 6 |

**Consecutive failures.** While a depot still has a spare, a failure is absorbed: the spare takes over the failed route (Stage 5), the flying and rotation counts stay as they were, and only the spare group shrinks. Failures can therefore repeat without any change to the cell assignment until that depot's spares are gone.

**Future spare-exhaustion rearrangement (not implemented).** The current controller borrows a spare from another open depot; it leaves alarms pending when no spare is available. A future capacity-weighted implementation could instead use the following rearrangement when a depot has no spare left and one of its flying drones fails, the cells of the whole region are redistributed from scratch, as if the allocation were starting again:

1. Recount each depot's surviving drones, N\_m'.
2. Run a capacity-weighted variant of A4 on all free cells using N\_m' as the weights, with nothing carried over from the old assignment.
3. Reset every depot's groups to the same shares φ of its new total.
4. Rebuild the routes in every sector (Choset decomposition, then round-robin) and the launch schedules, and give each new route to the nearest drone of that depot. Drones switch one route at a time, each at its next handover, so coverage dips only locally. Switching every drone at once would leave the whole region briefly unwatched and send every drone across it at the same moment, against the local repair of A6. A drone whose battery would not cover the new route returns and is replaced as in A5.
5. Re-check endurance (A1) and clauses C2, C5 and C6 on all routes, and check the rotation pool of A2 at each depot.

Because the weights follow surviving drones, every depot ends up with about the same number of cells per drone. The loss is shared across the region instead of leaving one depot's sector thinly covered.

**Two checks when scaling.**

- The rotation share cannot be copied unchanged from a smaller region or a smaller depot setback. Recompute each depot's mean and worst ferry times with the 500 m setback before accepting the 29 : 9 : 2 allocation. The earlier estimates of 85 s mean and 103 s worst ferry time, 4–5 rotation drones by Little's law and 5–6 with phased returns assumed depots only 40 m outside; they do not establish feasibility here. The batch-schedule margin must also be measured again.
- Depot capacity must grow with the fleet. For depot m, estimate λₘ = n_fly,m / (T_use − 2T̄_ferry,m) and check λₘτₘ / κₘ < 1, then simulate queue peaks. The previous 34% load estimate for three slots depended on the old 85 s ferry time. Keep κₘ = 3 as an experimental candidate, subject to this check and C6. The executable port configuration currently uses two slots and must be evaluated separately.

### New STL clauses

In the notation below, □ means "always" and ∀ means "for every".

**C5, return always possible.** Every drone's battery b\_u(t) always covers a minimum reserve σ\_min plus the battery needed to reach the nearest depot. This is the safety rule behind the return threshold in A5. The return threshold must sit a buffer σ\_buf above this floor (0.02 in the baseline implementation); a drone that turns back exactly at the floor breaks C5 one GSM step later.

$$
\Box\ \forall u:\quad b_u(t) \ge \sigma_{min} + \frac{\min_m d(u, D_m)}{v\,T_{endure}}
$$

**C6, depot queue.** The number of drones waiting at depot m never exceeds its service slots.

$$
\Box\ \forall m:\quad Q_m(t) \le \kappa_m
$$

**C7, spare exhaustion handled.** Whenever a flying drone fails at a depot with no spare left, the rearrangement of A8 starts within one GSM step. Using up a spare is normal operation and is not a violation.

$$
\Box\ \forall m:\quad I_m(t) \ge n_{spare,m}
$$

## Extension B: no-fly zones inside the region

A no-fly zone (NFZ) is an area that drones must not enter, for example because of a building or restricted airspace. Inside the surveillance region it creates obstacles that routes, distances and battery calculations must respect.

To keep the model simple, Extension B rests on these assumptions:

- **Instant turns.** Drones can make sharp left or right turns instantly, so no turning radius is needed.
- **NFZs with a buffer****.** Each NFZ is inflated by a safety margin b before planning, as airspace rules require in practice; b = 0 reproduces the zones as drawn. C8's robustness ρ then has to stay above b.
- **Exact positions.** A drone's position is known exactly, so the model does not depend on positional error.
- **Fixed zones.** Every NFZ is known in advance and does not change during a mission.
- **Footprint observation****.** A cell is observed while its centre is within a drone's sensing radius (30 m), whether or not the drone is inside it, as in the experiments.

### B1. Cell bookkeeping

Every cell is labelled either **free** (a drone may fly in it) or **NFZ** (part of a no-fly zone). Each cell is entirely one or the other; there are no partly blocked cells. Coverage is measured over the free cells only.

NFZ cells are left out of the coverage count because the task is to watch free space. With a 30 m sensing radius, NFZ cells that border free space are still seen from outside, so a restricted site can be watched without being entered; this can be reported as a separate perimeter metric.

### B2. Shortest-path distances replace straight-line distances (the key change)

The paper uses straight-line distance for route length, the return-to-base threshold, the predicted battery on arrival, and the cost of repairing routes after a failure. With an NFZ in the way, a straight line may pass through the zone, so it is only a **lower bound** on the distance a drone can really fly. The true path must go around the zone and is longer.

Writing d\_G(a, b) for the length of the shortest path from a to b that stays out of every NFZ, the relationship is:

$$
\lVert a - b \rVert_2 \;\le\; d_G(a, b)
$$

This matters for safety. The drone's return threshold is set from the straight-line distance, so it would turn back too late and could run out of battery before reaching the depot. That would violate the battery-floor clause C2. The old example at (920, 760), with D\_E only 40 m outside the region, does not apply to the revised depot positions. Recompute both distances and the selected return depot with the 500 m setback. The extra battery needed for a detour is (d_G − d_Euclidean) / (v T_endure); compare it with the 0.02 return buffer before predicting a violation. A nearest-depot return rule may select another depot, so an obstacle alone does not guarantee that R0 will violate C2.

The fix is to precompute the shortest *flyable* path between every pair of free-cell centres and depots. Two standard methods do this: a *visibility graph* (a network linking points that can see each other without crossing an NFZ) or *Dijkstra's algorithm* (the standard shortest-path search) run on the grid. With the 830 free cells of the experiments this is cheap. Because drones can turn instantly, a shortest path is simply a chain of straight segments that bends only at NFZ corners.

The return-to-base rule then changes from the straight-line distance d to the flyable distance d\_G:

$$
b_u(t) \le \sigma_{min} + \frac{\min_m \lVert u - D_m \rVert_2}{v\,T_{endure}} \quad\longrightarrow\quad b_u(t) \le \sigma_{min} + \frac{\min_m d_G(u, D_m)}{v\,T_{endure}}
$$

*Flyable-path implementation.* A clear straight segment is used directly because it is already a shortest flyable path. When an NFZ blocks that segment, cached Dijkstra paths through the free-cell graph supply a detour. Thus the implementation combines any-angle clear links and grid detours; it does not force every leg onto the grid. R0 deliberately uses a straight-line battery-return estimate as a control.

### B3. NFZ-aware route generation

The paper already cites Choset's *boustrophedon cellular decomposition*, a method for sweeping an area that contains obstacles. It is used here in place of the row-by-row ordering:

1. split the free space at the NFZ vertices (corners) into simple strips;
2. sweep each strip back and forth;
3. link the strips along the graph that records which strips touch.

This stays within the paper's boustrophedon approach. Plain row ordering with NFZs causes many detours, and round-robin's long jumps between cells would cross NFZs constantly.

### B4. An STL safety clause with a robustness margin

**C8, NFZ avoidance.** At all times, every drone's position p\_u(t) stays outside every NFZ.

$$
\Box\ \forall u,\ \forall k:\quad p_u(t) \notin \mathrm{NFZ}_k
$$

STL *robustness* (Fainekos & Pappas, already cited by the paper) turns this yes/no rule into a number: the minimum signed distance to any NFZ, where sd is positive outside a zone and negative inside it. A larger positive value means a safer margin, and a negative value means a violation.

$$
\rho(t) = \min_{u,k}\ \mathrm{sd}\big(p_u(t), \mathrm{NFZ}_k\big)
$$

### B5. Failure handling near NFZs

When a drone fails, the nearest surviving drone in a straight line may be on the far side of an NFZ. The paper's insertion-cost rule (its Eq. 40) must therefore use shortest-path cost, and the urgency margin δ\_age must allow for the longer real detour.

### Possible future extension: pop-up and time-varying NFZs

The present model treats every NFZ as fixed and known in advance. Zones that appear suddenly or are active only during a time window are left as a natural next step. They could be handled like the Stage 5 failures, with a local repair of the affected routes, but they are outside the current formulation.

## How the two extensions interact

Depots and no-fly zones affect each other, so the two extensions should be designed together.

**Depot regions by flyable distance.** An NFZ can make the depot that is nearest in a straight line the wrong choice, because the real path to it is long. Instead, the free cells are divided among the depots by shortest flyable distance, weighted by each depot's capacity. The resulting pipeline is:

1. partition the free cells among the depots;
2. apply the Choset decomposition within each partition;
3. deal each partition's cells round-robin into routes, then check that every route satisfies the endurance constraint;
4. build a launch schedule for each depot.

**A bound on fleet size (the paper's Problem 1).** Combining the pieces gives an interpretable lower bound on how many drones are needed. It generalises the paper's first-order coverage relation (its Eq. 49):

$$
N \gtrsim \frac{N_{free}\,\Delta\,\kappa_{det}}{v\,\Theta}\left(1 + \frac{2\bar T_{ferry} + \tau}{T_{use} - 2\bar T_{ferry}}\right) + n_{spare}
$$

The first factor counts the drones needed to keep the free cells fresh: N\_free cells, each Δ apart (the cell size), must all be visited within Θ seconds at speed v. The bracket adds the rotation drones from A2, and n\_spare adds the spares. The symbol κ\_det ≥ 1 is a **detour factor** that grows when NFZs force longer paths.

More depots shrink the bracketed term. NFZs shrink N\_free but raise κ\_det. This partly addresses the paper's own first future-work item.

**Coverage gained in transit.** Part of the baseline's 85% coverage is expected to come from drones observing cells while they cross the region on long round-robin transits. NFZs block those transits, and extra depots change them. Transit coverage should therefore be measured separately, or the comparisons will be confounded.

## Proposed experiments

The following runs on one fixed map test what the proposed approach adds. Everything not listed as a difference between runs is held fixed.

**Fixed setup**

- **Region:** 30 × 30 cells of 40 m (1200 m × 1200 m, 900 cells). This keeps the paper's cell size, Θ = 180 s, T\_use = 1890 s and τ = 60 s.
- **Fleet:** 120 drones split globally into 87 flying, 27 rotation and 6 spare. Flying drones follow sector cell counts and reserves follow route counts (A4), with κₘ = 2 service slots per depot, matching `configs/port.json`. The map has 830 free cells, about 9.5 per flying drone, against 9.0 in the baseline implementation (144 cells, 16 flying). The load per drone is therefore close to the baseline's.
- **Drones:** speed 6 m/s, usable flight time T\_use = 1890 s (battery endurance 2100 s with a reserve σ\_min = 0.1), sensing radius 30 m and a GSM step of 1 s, as in the baseline implementation.
- **Distances:** use clear straight links where possible, and Dijkstra detours on the 8-connected free-cell graph otherwise. A diagonal graph step costs √2 times the cell size. Every physical segment is checked against buffered NFZ interiors. The current implementation permits boundary grazing within 1e-9 m, consistent with nonnegative signed clearance. Depots connect through visible free-cell centres. This differs from forcing all links through only the nearest border cells.
- **Routes:** the free cells are divided among the depots by d\_G, each part is swept by boustrophedon decomposition split at the NFZ corners, and the ordered cells are dealt round-robin into that depot's routes.

**Map.** Positions are in metres from the south-west corner of the region. Three depots sit opposite the middle of the west, east and north sides, each exactly 500 m outside its nearest boundary point. The partition of A4 assigns 309, 290 and 231 free cells to the west, east and north depots respectively. The two NFZs cover 70 of the 900 cells (7.8%).

| Item | Position | Why there |
| --- | --- | --- |
| D\_N | (600, 1700) | Reaches the open side of the L-shape's pocket |
| D\_W | (−500, 600) | Directly behind the rectangle, so the way east is blocked |
| D\_E | (1700, 600) | Faces the bar of the L, so pocket cells next to the bar are far to fly |
| NFZ 1, rectangle | x 200–520, y 520–680 (8 × 4 cells, 32 cells) | A convex block in D\_W's path |
| NFZ 2, L-shape | bar x 960–1040, y 640–1120; arm x 680–960, y 640–720 (38 cells) | A non-convex shape: its pocket (x 680–960, y 720–1120) opens north and west |

The nearest boundary points are (600, 1200), (0, 600) and (1200, 600) for D_N, D_W and D_E respectively. Each depot-to-boundary distance is 500 m. Validate these distances for every generated map as well; randomising internal NFZs does not change the outer boundary.

**Setback consequences.** Ferry paths, endurance, replacement lead times, rotation inventory and service loads must use the 500 m depot positions. The former shared-loop response budget is removed along with that planner. The round-robin controller reports alarm-to-launch latency separately from ferry travel; it does not assert that a spare can be on station within 150 s or 210 s. Previous results and animations from the 40 m layout are historical.

**Runs**

| Run | Depots | NFZs | Event | What it shows |
| --- | --- | --- | --- | --- |
| R0 | all three | both | none; the return rule uses straight-line distance | The flaw of B2: drones in the L-shape's pocket that head for D\_E turn back too late, so C2 may fail; verify that the chosen return depot actually requires a detour larger than the buffer. R3 is the same run with d\_G. |
| R1 | each depot alone in turn (D_W, D_E, D_N), with 6 service slots | both | none | Same 120-drone fleet and total service capacity at each candidate single depot. Report all three and select the best by steady coverage, then worst cell age. |
| R2 | all three | none | none | Effect of removing NFZs; 900 free cells. |
| R3 | all three | both | none | Primary Choset-plus-round-robin result. |
| R4 | all three | both | one flying-drone failure served by D_E at 1800 s | Single-failure bridging and spare takeover. |
| R5 | all three | both | four flying-drone failures served by D_E at 1800, 2100, 2400 and 2700 s | Repeated failures and borrowing spares from another open depot. |
| R6 | all three | both | outage of D_N from 1800 s to the end | Stored drones are unavailable, incoming returns are redirected, and affected routes are homed at another open depot without changing their waypoint sets. |

R0 against R3 tests sensitivity to the return-distance metric; a violation need not occur on the natural trajectory. R1 against R3 shows the benefit of depot locations, though it also changes how spares are pooled. R2 against R3 shows the effect of the NFZs; R2 watches 900 cells and R3 830, so compare them on worst cell age and the share of time spent ferrying, not on coverage alone.

**Metrics.** Every run reports steady-state coverage over each run's free cells (830, or 900 in R2), overdue cells, the worst cell age (the longest any cell goes unobserved, the figure an operator would contract for), minimum battery margin, the share of flight time spent ferrying, and each depot's peak queue. The event runs add recovery time and spare-launch latency, and R6 adds the number of routes rehomed after closure. The pass condition is that clauses C2, C5, C6 and C8 are never violated in any run; C1, C3 and C4 are reported. Each run lasts 4800 s and steady-state values are averaged over t > 1200 s, as in the baseline implementation. The unchanged controls are deterministic, so repeating an identical setting provides no independent evidence. A future stochastic planner would require multiple movement seeds per setting. Determinism does not remove the dependence on one hand-placed map, so R3 is also repeated on 20 maps with randomly placed NFZs covering about 8% of the cells.

## Implemented path planner and simulation review

The only supported path-planning mode is `round_robin`: [Choset and Pignon's boustrophedon decomposition](https://publications.ri.cmu.edu/coverage-path-planning-the-boustrophedon-decomposition), followed by round-robin waypoint assignment within depot sectors. The previous shared-loop algorithm, its fleet-sizing rule and its re-spacing controller have been removed.

### Decompose, sweep, then assign round-robin

1. Compute free cells and nearest-flyable-depot sectors (A4).
2. Sweep each sector upwards through horizontal free intervals. Continue a decomposition region only when one interval overlaps exactly one interval in the next row and vice versa. End and create regions at births, deaths, splits and merges. Record their adjacency graph. This is a raster implementation of Choset's connectivity-event construction; it is not an exact continuous-polygon decomposition for arbitrary geometry.
3. Traverse the region graph in deterministic depth-first order, starting a new traversal for any disconnected component. Sweep each decomposition region back and forth. Every required free cell appears exactly once in the resulting order.
4. Deal ordered cells cyclically to the sector's routes. Replacements inherit a route. All travel between waypoints and depots uses the existing NFZ-safe path planner; straight-line links are used only where they are clear. The R0 control changes the battery-return distance estimate, not the physical collision-free paths.
5. Check endurance before launch, stagger departures independently at each depot, monitor battery reserve throughout flight, and launch rotation/spare drones in response to alarms. Failure bridging uses flyable insertion costs. Depot closure rehomes affected routes and redirects returning drones.

This primary method deliberately uses multiple interleaved routes. It does not maintain one shared loop, global spacing or a cyclic revisit-time formula. A cell's freshness depends on the realised routes, departures, footprint observations and handovers; the simulations report these outcomes.

### Randomness remains an optional extension

The selected planner is deterministic. Round-robin alone does not make movements random. If randomness is required later, vary admitted sweep entry choices, choose among NFZ-safe equal-cost connectors, or randomly permute recipients within each complete round of waypoint assignments. Merely relabelling routes does not change flight paths. Reject candidates that break endurance, return feasibility or the configured freshness budget, and use staged handovers for route changes.

Evaluate such a future policy with separate map and movement seeds, repeated trials per fixed map, and the same fleet and service capacities. Report predictability together with worst cell age, battery margins, queue peaks and failure recovery. Random obstacle layouts in the present campaign test map dependence; they do not randomise motion on a fixed map.

### Simulation outputs and limits

Run `python scripts/run_port_study.py --out results/port-choset-round-robin` for R0–R6, including all three R1 locations, and 20 seeded NFZ layouts. The runner writes metrics, events, route assignments and JSON summaries for fixed scenarios, a random-map CSV, summary tables and figures. Run `python scripts/run_scenarios.py` for the five baseline scenarios. Measured results and reproducibility details are recorded in [the port study](port-study.md).

The implementation monitors C2/C5/C6/C7 at sampled times; its combined pass flag excludes freshness, launch latency and recovery. The design report calls NFZ avoidance C8, while the executable monitor labels it C7. Neither a successful startup nor zero NFZ violations establishes uninterrupted coverage. Capacity-weighted sector balancing, spare-exhaustion repartitioning (A8), exact launch-to-on-station guarantees and continuous-time collision separation remain outside the current implementation.

## Glossary

| Term | Plain meaning |
| --- | --- |
| UAV | Unmanned aerial vehicle, i.e. a drone |
| Depot | A ground base where drones take off, land and are readied again |
| Ferry | Travel between a depot and the surveillance region, during which no patrolling happens |
| Turnaround time (τ) | The time a depot needs to ready a drone for its next flight |
| Cell | One small square of the grid that divides the region |
| Fresh cell / coverage | A cell observed (its centre within a drone's sensing radius) within the last Θ seconds / the fraction of cells that are fresh |
| GSM | Ground Station Monitor, the central computer that coordinates the swarm |
| ILP | Integer linear program: an optimisation problem that chooses whole numbers subject to constraints |
| Boustrophedon | A back-and-forth sweep pattern, like ploughing a field |
| Round-robin | Dealing items out in turn, like cards, so each route receives cells from all over the region |
| STL | Signal temporal logic: a formal language for rules about behaviour over time, checkable while the system runs |
| NFZ | No-fly zone: an area drones must not enter |
| Euclidean distance | Straight-line distance between two points |
| Shortest-path (flyable) distance | The length of the shortest route that avoids all no-fly zones |
| Little's law | In a steady system, average number in the system = arrival rate × time each spends there |
| Set cover | Choosing the fewest options so that every item is covered by at least one |
| Correlated failure | Several failures with one shared cause, such as a depot outage |

## Symbols used in the formulas

| Symbol | Meaning |
| --- | --- |
| N, n\_fly, n\_rotate, n\_spare | Total drones, and the flying, rotation and spare groups |
| Θ | Freshness window: a cell is fresh if observed within the last Θ seconds |
| T\_use | Usable flight time on one battery |
| τ | Turnaround time at a depot |
| T\_ferry, T̄\_ferry | One-way ferry time, and its average |
| v, v\_max | Drone speed, and top speed |
| Dₘ, M | Depot m, and the number of depots |
| d(a, b) | Straight-line distance between a and b |
| d\_G(a, b) | Shortest flyable distance from a to b, avoiding all NFZs |
| w\_{u,j} | The j-th waypoint of route u |
| D\_loop(u) | Length of one lap of route u |
| κₘ, τₘ | Service slots and turnaround time of depot m |
| Qₘ(t), Iₘ(t) | Queue length and drone inventory at depot m at time t |
| b\_u(t) | Battery level of drone u at time t |
| σ\_min | Minimum battery reserve |
| T\_endure | Endurance time used to convert flight distance into a battery share |
| p\_u(t) | Position of drone u at time t |
| NFZ\_k | The k-th no-fly zone |
| N\_free, Δ | Number of free cells, and the cell size |
| κ\_det | Detour factor (at least 1) caused by NFZs |
