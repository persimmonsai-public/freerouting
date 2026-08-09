# Roadmap: routing fine-pitch 8-layer dense-BGA boards

Goal: take freerouting from "greedy per-net router that struggles past 2 layers" to
completing a fine-pitch (≤0.8 mm) dense-BGA board on an 8-layer stack. Ordered by blocking
severity; every phase keeps the campaign's methodology — a measured gate before/after, every
hypothesis A/B'd, refuted ideas preserved on side branches.

**Existing assets this builds on** (branch `perf/query-stack-and-parallel-autorouter` and
side branches): `PadArrayDetector` (array classification + per-ball dogbone directions),
the free-space partition + room cover + windowed-channel materialization,
multilayer (layer, room) A* with `ExpansionDrill` materialization proven end-to-end
(`perf/partition-via-support`), sentinel-safe attachment machinery, and the
refuted-hypothesis catalog in `docs/free-space-partition.md`.

## Phase 0 — a real fixture and gate (prerequisite, ~small)

- Acquire/construct an 8-layer dense-BGA DSN fixture (open hardware boards with FPGA BGAs
  exist; else synthesize: 0.8 mm 400-ball BGA + peripheral components).
- Establish the gate the way Issue508 was gated: fixed pass count, score/unrouted/violation
  baseline from the current router, 3 isolated runs. Everything later measures against it.
- Extend `ArrayPadFieldProbeTest` to report ring depths, per-channel capacities at each
  layer's rules, and the theoretical escape budget (balls vs total channel capacity) — the
  numbers Phase 2's planner must satisfy.

## Phase 1 — rule regions and via spans (foundations, medium)

1. **Region-scoped rules**: a rule area (box/polygon + layer set) carrying trace width and
   clearance overrides; plumb into `AutorouteControl` (per-region compensated widths), the
   DRC (`check_trace_shape` picks the region's clearance), and the partition's `min_pass`
   admissions. The existing global neckdown retry (`retryConnectionNecked`) becomes the
   fallback, not the mechanism.
2. **Span-aware vias**: model blind/buried/micro spans explicitly in via selection
   (`ViaInfo` already carries padstack layer ranges). The maze search and the partition's
   cross-layer transition both choose a span, costed by the layers it blocks. Via-in-pad
   activates through the existing attach flags when the fixture's rules allow it.
- Gate: baseline board unchanged (flag-off byte-identical); new fixture must show the
  router *using* necked regions and short spans where rules permit.

## Phase 2 — BGA escape planner (the core, large)

Ordered assignment before any maze search, as its own stage between fanout and routing:

1. **Capacity model**: from `PadArrayDetector` geometry + Phase 1 region rules, per-channel
   capacity per layer, antipad obstruction map under the array.
2. **Ring-to-layer assignment**: assign each ball's escape layer ring-by-ring (outer rings
   on surface, deeper rings via dogbone to deeper layers), respecting channel capacities —
   formulate as min-cost flow on the channel graph; fall back to greedy ring order first
   and A/B the flow solver against it (the campaign rule: measure the simple thing first).
3. **Escape realization**: emit the dogbone stub + via + inner-layer escape run for each
   ball as FIXED pre-routes (the partition's windowed-channel materialization already
   builds exactly these short, legal chains; reuse it directly). Unassignable balls are
   reported, not silently dropped.
- Gate: escape completion rate on the BGA (target 100% of routable balls), zero
  violations, and total router wall-clock vs the no-planner baseline.

## Phase 3 — congestion-negotiated ordering (large)

Replace hand-tuned acceptance gates (airline length, via cost — both measured brittle)
with PathFinder-style negotiation on the partition's room graph:

1. Route all remaining connections independently over the room cover (cheap, ~1 ms each,
   parallelizable with the existing bulk-synchronous machinery from the parallel router).
2. Price overused rooms/channels; iterate until no overuse; commit the final set through
   the existing validated-commit path.
3. The classic engine remains the finisher for whatever negotiation leaves incomplete.
- Gate: score parity-or-better at equal wall-clock on BOTH fixtures; this phase subsumes
  the acceptance-policy problem that capped the quality mode at 994.85.

## Phase 4 — scale performance (medium, re-measure old verdicts)

- Re-measure incremental cell/room maintenance (`perf/partition-incremental-refuted`) on
  the 8-layer fixture — the refutation was economics, not correctness, and bigger boards
  shift the wholesale/incremental ratio.
- Extend the partition's via search with Phase 1 span costs; profile the O(n²) room
  adjacency and cross-layer scans at 8-layer room counts; sweep-based adjacency if the
  profile says so.
- Multi-thread the negotiation phase's independent searches (the parallel router's
  phase-separation machinery applies directly).

## Phase 5 — signal-integrity features (independent track, large)

- Differential pairs: pair detection from net classes, paired search (route the pair
  centerline, offset both traces), gap rules from Phase 1 regions.
- Length matching: post-route meander insertion inside the partition's rooms (free-space
  knowledge makes meander placement cheap); match groups from net classes.
- Plane awareness: antipad fields as first-class obstacles per plane layer; report (not
  yet fix) return-path discontinuities.

## Sequencing and risk

Phases 0→1→2 are strictly ordered (each needs the previous); Phase 3 can start against the
2-layer fixture in parallel with Phase 2; Phase 5 is independent. The highest-risk item is
Phase 2's assignment quality — mitigated by the flow-vs-greedy A/B and by the fact that
escape stubs are short fixed routes validated by the existing insertability check. The
recurring campaign lesson applies everywhere: the machinery is rarely the blocker; WHICH
routes commit is — hence gates on every phase and negotiation (Phase 3) as the structural
answer rather than more hand-tuned acceptance heuristics.

## Phase 0 baseline (measured 2026-08-09)

Fixture `Issue732-RoyalBlue54L-Feather.dsn` (8 layers, 333 pins, U1 = 10x10 BGA at 0.9 mm):
tests take the fixture via `-Dfr.fixture` and the job budget via `-Dfr.timeout`.

Current router, 20-minute budget, maxThreads=1: fanout escapes 246/303 SMD pins (81.2%,
vs 98.4% on the 2-layer fixture); auto-routing reaches pass 5 in the budget at score
**451.97 / 59 unrouted / 538 violations**, with per-pass wall-clock ~140 s and the
violation count constant from pass 3 on -- the router cannot repair them. This is the
number the roadmap exists to move. GATE CONFIRMED: 3 isolated runs produced the identical
final board (same hash, same 451.97 / 59 / 538) -- fully deterministic. Phase 0 complete;
Phase 1 (region rules + via spans) is next.
