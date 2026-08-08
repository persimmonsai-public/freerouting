# Maintained free-space partition — design and measurements

Status: **stage 1 of 3 landed** (data structure + feasibility gate). The router does not use
it yet.

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

**Stage 3 — measure and gate.** Same protocol as everything else in this effort: three
isolated runs, `--no-build-cache`, score-parity gate (≥ 989.72, ≤ 2 unrouted, 0 violations)
before any wall-clock claim. The upside case from the profile is ~1.7–1.9x sequential; the
history of this optimization effort (nine estimates, eight dead on measurement) is the reason
the gates come first.
