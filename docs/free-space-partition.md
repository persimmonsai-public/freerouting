# Maintained free-space partition — design and measurements

Status: **stage 2 landed behind a default-off flag** (`featureFlags.partitionRouter`); not yet
a performance win -- see the stage-2 results below for exactly why and what remains.

## Why

Profiling shows roughly half of routing time is spent constructing free-space expansion rooms
*inside* the maze search (`complete_shape`, its tree queries, `SortedRoomNeighbours`) and
tearing them down after every connection: **184,190 room constructions for 578 connections** on
`fixtures/Issue508-DAC2020_bm01.dsn`, up to 1,911 rooms for one hard connection.

Two prior approaches failed, with measurements:

- **Naive retention** (`maintain_database = true` in batch): **2.6x slower**. Rooms are carved
  relative to a specific search (net number, ignore shapes, from-doors) and fragment
  monotonically; searches walked ~10x more expansion elements and some failed from coverage
  gaps. Construction did not even drop — each commit invalidates the busy region, which is
  exactly where the next searches happen.
- **Retention would also not be saved by better invalidation**: the parallel-router experiments
  measured that consecutive connections contend for the same space 75–89% of the time on this
  board, so the working set is structurally invalidated.

The conclusion is not "avoid rebuilding" but "make the decomposition canonical and its
maintenance cheap, so search pops are construction-free":

- cells are **net-agnostic** (every item is an obstacle; same-net traversal is the search's
  concern, not the geometry's) and **search-independent** (a canonical partition, not maximal
  seed-grown tiles),
- updates on commit are **local** (only the slab range the changed shapes span),
- coarseness comes from a **maximal horizontal merge**, not from search context.

## Feasibility gate (measured before building)

On the reference board in its fully routed — densest — state:

| quantity | measured |
|---|---|
| cells, both signal layers, after merge | **2,148** (raw slab cells 42,554 — the merge is a 23x coarsening) |
| full rebuild, all layers | **21 ms** |
| re-partition of a 10%-of-board swath | **~0.8 ms** |

For scale: a single hard connection today builds up to 1,911 throwaway rooms; the whole-board
partition is the same size and is maintained for well under a millisecond per commit. The
graph a search must traverse stays in the low thousands of cells — far from the ~50k-room
blowup that sank naive retention.

## Structure (`autoroute/FreeSpacePartition`)

Vertical-slab decomposition: slab edges at obstacle x-extents; per slab, free y-intervals are
the complement of the obstacles **clipped to the slab** (exact for boxes; for 45° shapes the
clipped bounding box loses only sub-slab triangles at diagonal edges). Slab-adjacent intervals
with identical bounds merge — which makes **every cell a rectangle** and every door a vertical
segment between rectangles sharing an x-edge.

Updates rebuild the affected slab range wholesale (plus one slab on each side — a new edge can
split a pre-existing slab; found by the property tests) rather than patching incrementally:
the measured cost is negligible and wholesale recomputation cannot drift from the
built-from-scratch result. `FreeSpacePartitionTest` verifies update-equivalence, disjointness,
exact area accounting, and adjacency symmetry over randomized mutation histories.

## Remaining stages

**Stage 2 — engine bridge (the risky one).** Feed the search from the partition instead of
lazy construction, behind a flag:

- populate a retained `AutorouteEngine` with one `CompleteFreeSpaceExpansionRoom` per cell and
  doors from partition adjacency; `complete_expansion_room` becomes a no-op,
- same-net traversal: own-net items are walls in the partition, so their
  `ObstacleExpansionRoom`s must be enterable at zero cost when `is_trace_obstacle(net)` is
  false (today they are entered only via ripup, with penalty),
- target doors for start/destination items must be created against adjacent cells explicitly,
- on commit, `additional_update_after_change` maps invalidated cells to room removals and new
  cells to room insertions, repairing doors locally,
- drills: `DrillPageArray` machinery is unchanged (it queries the item tree, which still holds
  the items), but pages must persist across connections and invalidate locally.

**Stage 2 status (measured).** The full pipeline works end-to-end -- partition A*, hand-built
backtrack chain, unchanged Locate/Insert, validated commit -- and it committed real routes with
the score gate intact (one run even ended *better* than baseline: 994.85 / 1 unrouted). Two
contract discoveries shaped the implementation: `DrillItem.get_trace_connection_shape` is a
single point (the pad centre), so the destination cell must overlap the item's interior --
solved by lifting the routing net's items out of the partition per search (bulk API, edge
refcounting); and door sections must be allocated with exactly the offset Locate re-derives.

It is not yet a win, for two measured reasons:

1. **Hit rate ~1%** (2–12 of ~200 attempts per pass). The dominant residual failure is
   commit-time validation rejecting pad exits that clip a NEIGHBOURING pad's clearance --
   fine-pitch exits are legal only under the pad-exit/acid-trap exemptions implemented inside
   the classic insert path (`check_trace_shape` contact-pin handling), which the plain
   clearance validator does not model. Aiming door sections along the endpoint line did not
   change this (144 -> 140 failures).
2. **Attempt overhead**: lifting a partly-routed net spans most of the board, so each attempt
   pays a near-full-board rebuild even with bulk mutation (run: 43 s -> 75 s with the flag on).

## Stage 3 — the systematic experiment series (all measured, most refuted)

Every hypothesis below was implemented and A/B-measured on the reference workload
(score gate ≥ 989.72 / ≤ 2 unrouted / 0 violations held in every run):

1. **"The validator is too strict" — REFUTED.** A transactional commit (insert with
   validation off, judge with the board's own `Item.clearance_violations()`, undo if dirty)
   raised the hit rate 2 -> 12 but let ~140 attempts/pass die inside
   `insert_forced_trace_polyline`, whose shove machinery churned the board (58 s pass 1,
   transient violations). Replaced by pre-insert validation with `check_polyline_trace` —
   the insert path's own predicate, with its pad-exit/tie-pin exemptions. Result: the same
   ~140 rejects. The planned geometry is genuinely uninsertable; validation modeling was
   never the wall.
2. **"Pad-centre attachment causes the conflicts" — REFUTED.** Preferring (or forcing)
   non-pad attachments — the escape vias/traces fanout already provides — changed nothing
   (140 -> 144 rejects).
3. **"Thin cells admit vertical movement wider than the cell" — marginal.** A straight-pass-only
   constraint for cells narrower than the trace: 140 -> 135 rejects. Correct filter, kept,
   not the wall.
4. **Partition staleness — real but secondary.** Conflict logging showed rejects dominated
   by high-id items committed earlier in the same pass by the CLASSIC engine, which never
   invalidated the partition. Invalidating after every classic attempt (kept, for soundness)
   cut rejects 140 -> 113 — but the hit rate stayed at 2, and per-attempt full rebuilds cost
   98 s (fix would be observer-driven incremental updates, moot until the next point is solved).
5. **The actual wall: slab strips break Locate's corner-realization contract.** With a fresh,
   sound partition, rejected plans contain segments up to 340k units long crossing many
   obstacles. `LocateFoundConnectionAlgo` shrinks each room by the compensated half-width
   before placing interior corners; slab-strip cells in dense regions are about one trace
   width wide, the shrunk interior is EMPTY, and corner placement degenerates — the realized
   polyline leaves the cell chain entirely. Thin rooms are structurally unusable for the
   unchanged Locate pipeline, independent of validation, attachment, or freshness.

**Verdict.** The slab decomposition is proven cheap to maintain (disjoint bulk dirty-ranges
brought flag-on overhead from +75% to ~+5%: 39.5 s vs 37.4 s baseline) and the
partition -> Locate -> Insert bridge is proven sound — but this cell SHAPE cannot feed
Locate at fine pitch. That prediction was then tested directly:

## Stage 3b — maximal-rectangle room cover (measured: direction confirmed, not yet a win)

`FreeSpacePartition` now derives a **room cover** from the cells: each merged cell extended
left/right through every slab whose free interval CONTAINS its y-interval. Rooms are
overlapping maximal rectangles (a cover, not a partition); doors between overlapping rooms
are 2-dimensional, which `ExpansionDoor` already supports between
`CompleteFreeSpaceExpansionRoom`s. The A* runs over rooms with two admissions: an
intermediate room must be wider than the trace in BOTH dimensions (so Locate's erosion has
interior), and a transition's overlap must host the trace width along its long dimension.
Property-tested: rooms free of obstacles, cover complete, horizontally maximal, adjacency
symmetric.

Measured (same fixture, flag on, partition invalidated after every attempt for soundness):

- pass-1 hit rate 1 -> **7** routed, rejects 113 -> 94, zero violations in every pass —
  fat rooms demonstrably fix the Locate degeneration for the routes that now commit;
- **not gate-green**: final 984.59 / 3 unrouted (gate: >= 989.72 / <= 2) — the partition
  routes that do commit are low-quality serpentines that cost endgame score;
- **4x runtime** (148 s vs 37.4 s baseline): per-attempt wholesale freshness costs
  ~150 ms even after bulk-building (`ensure_fresh` now uses begin/end_bulk; a from-scratch
  build without bulk was measured at ~240 ms/attempt, +47 s per pass).

## Stage 3c — windowed channels: the router now BEATS the baseline score

Two further refuted hypotheses first, both measured: fatter terminal rooms (rejects
94 -> 106) and channel-clipping rooms to their raw joints (no change — consecutive maximal
rooms overlap over most of their length, so raw joints are nearly as large as the rooms).

The real mechanism, found by reading `LocateFoundConnectionAlgo45Degree`: between
overlapping rooms every door is 2-dimensional, and for those Locate IGNORES the door
sections chosen at materialization — each step is a greedy nearest-point move, and the
dogleg corners it inserts are never validated against anything. Inside 100k+-unit maximal
rooms the walk drifts freely, and the final unchecked dogleg to the pad centre ran straight
down the pin column (the reject signature: second segment crossing 5+ neighbouring pins).

**The fix (windowed channels):** materialize narrows each interior joint to a WINDOW — the
straight start-to-target aim line's interpolated point clamped into the joint, inflated by
the pass width — and hands Locate each room clipped to the bounding box of its entry and
exit windows. Inside one axis-aligned box every dogleg between two interior points is
contained by convexity, so the realized polyline is legal by construction, and the channel
follows the aim line, which also fixed route quality.

Measured (flag on, reference fixture): **final score 994.85 / 1 unrouted / 0 violations,
reproduced in 4 of 4 runs — better than the 989.72 / 2 baseline.** Pass-1 hit rate 11 of
199. Wall-clock 111 s vs 37.4 s baseline after profiling-driven fixes (skip re-inserting
the lifted net into to-be-rebuilt partitions; array-based extension walks with
binary-search containment — `t_rooms` was 26.5 s of pass 1 alone before these).

**Cost anatomy and what remains.** Per-attempt freshness is load-bearing for quality
(amortizing invalidation to every 8th attempt saved almost nothing — 141 s vs 145 s — while
dropping the score to 979.47/4, below gate), and the remaining overhead is `build_cells` +
`build_rooms` running whole-board on every attempt because only the slab INTERVALS update
locally today. The design's "updates are local" promise must be extended to cells and
rooms: incremental cell/room maintenance over the lift's dirty ranges is the single
remaining item between "quality mode at 3x cost" and a genuine win. Raising the pass-1 hit
rate beyond 11/199 (residual rejects: 110) would then directly convert classic search time
into ~1 ms partition successes.

The flag stays default-off; flag-off behavior is unchanged.

Gate protocol unchanged: three isolated runs, `--no-build-cache`, score ≥ 989.72,
≤ 2 unrouted, 0 violations, and wall-clock only after the gate.
