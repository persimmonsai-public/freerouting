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
final board (same hash, same 451.97 / 59 / 538) -- fully deterministic.

**Gate correction (measured):** the raw, unrouted board already carries all 538 violations
(1076 item-pair counts / 2) -- they are fixture artifacts among fixed items, not router
output; the router adds ZERO violations. The gate is therefore: unrouted <= 59,
violations <= 538 (none added), score >= 451.97, within the 20-minute budget. Phase 0
complete; Phase 1 (region rules + via spans) is next, targeting the 59 unrouted and the
57 unescaped fanout pins.

## Phase 1 pivot (measured 2026-08-09)

The gate board's rules offer exactly ONE via: 8000x8000 through-hole spanning all 8 layers,
attach_smd=false, default clearance 1500. Dogbone feasibility at U1 (9000 pitch, 5000x4000
pads): the grid-diagonal via position is ~6364 from each ball but needs
4000 + 1500 + ~2000 = 7500 -- **inner-ball escape is geometrically impossible under the
design's own rules.** The 9 unescaped U1 balls (and likely U2's) are unroutable-by-rules,
not router failures.

Consequences, in order:
1. Phase 2's planner gains an **escape-feasibility classifier** (detector geometry + via
   rules + clearance): each unescaped ball is labeled rules-impossible vs algorithm-missed;
   only the second class counts against the router. The achievable-unrouted number replaces
   59 as the real target once measured.
2. To exercise Phase 1's span-aware/microvia machinery at all, Phase 0 gains an addendum:
   a fixture VARIANT adding a plausible microvia (e.g. 4000:2000 um, layers 0-1) to the DSN
   via set -- then U1 inner-ball escape becomes legal and the machinery is measurable.
3. Region-scoped rules remain valuable for trace necking between balls (2500-unit traces in
   4000-5000-unit gaps is workable) independent of the via problem.

## U1 investigation complete (measured 2026-08-09)

The candidate-spot legality scan (8000 via + clearance vs the tree, all 8 layers, 1500-step
grid within 13.5k reach) found ZERO legal via positions for the two "algorithm-missed"
boundary balls (pins 188/189) -- on the post-fanout AND the pre-fanout board (fixed items
crowd that region; fanout ordering is not the cause). Verdict: **all 9 unescaped U1 balls
are rules-impossible under the design's single 8000-unit through-hole via; the fanout stage
makes zero errors on the BGA.** The v1 classifier's boundary assumption over-counted, as
flagged; the spatial scan is the correct feasibility test and should replace it when the
classifier graduates to production. Achievable-escape work therefore moves to U5/U2/U6
(spatial scan as the tool), and exercising Phase-1 span machinery still requires the
microvia fixture variant.

## Escape accounting COMPLETE (measured 2026-08-09)

The spot scan generalized to all 57 unescaped pins:
`{C21:1, C22:1, J3:3, J6:3, J8:1, R5:1, U1:9, U2:9, U3:1, U4:2, U5:28, U6:6}` --
**every single one RULES_IMPOSSIBLE** (zero ESCAPABLE_NOW, zero ORDERING_VICTIM, pre- and
post-fanout). Under this design's via rules the fanout stage is PERFECT: 246/303 escaped is
exactly the escapable set.

Consequences:
- Via-escape improvement on this fixture is a dead end without the **microvia fixture
  variant** -- now the single prerequisite for all Phase 1/2 escape machinery.
- The router's remaining real gap on this board is the **59 unrouted connections**, some of
  which involve unescaped pins that may still be routable DIRECTLY on the surface layers
  (escape-via and routability are different questions). Measuring how many of the 59 are
  surface-achievable is the next diagnostic; Phase 3 (negotiation) and surface-routing
  quality are the levers for those.
- The spatial spot scan is the production feasibility test (the v1 interior/boundary
  heuristic is retired -- it over-counted 48 of 57).

## RETRACTION + corrected accounting (measured 2026-08-09)

The "all 57 rules-impossible / fanout is perfect" verdict was WRONG -- an artifact of the
spot scan treating copper pours as obstacles. The board's ConductionAreas have
get_is_obstacle()==false (pours reflow; the router treats them as passable), and with pour
semantics honored the accounting inverts:

- ORIGINAL fixture: 34 ESCAPABLE_NOW (legal spots exist post-fanout -- concrete fanout
  misses), 29 ORDERING_VICTIM (spots existed pre-fanout, consumed by neighbours -- the
  Phase-2 planning case), only 3 RULES_IMPOSSIBLE.
- MICROVIA variant (fixtures/Issue732-RoyalBlue54L-Feather-microvia.dsn, adds a 300:150 um
  blind 0-1 via; 400 um was measured too big -- rectangular pad corners govern the diagonal):
  55/55 escapable-or-ordering, zero impossible. Fanout with the microvia present: 248/303.

Consequences: items 1-2 have real targets on BOTH fixtures; the feasibility classifier must
use is-obstacle semantics (pours passable, spans respected) when it graduates to
production; and the fanout stage has ~34 diagnosable misses on the original board.

## Microvia variant: full-routing measurement (2026-08-09)

Same 20-minute budget as the baseline: the variant reaches pass 5 at **461.84 / 56
unrouted / 538 (pre-existing) violations** vs the original's 451.97 / 59 -- the existing
engine exploits the blind microvia end-to-end with no code changes (+3 routed, +10 score).
Against the 55/55 geometrically-achievable escapes, 56 unrouted confirms the remaining gap
is planning and negotiation, not via availability: the Phase-2 escape planner (capacity
model, ring-to-layer assignment, fixed pre-routes) and Phase-3 negotiation are the levers,
with the ~34 diagnosable fanout misses the immediate code target.

## Span-preference measurement closes item 2's engine question (2026-08-09)

Via-rule ORDER is the engine's span selector, A/B'd on the variant in the same budget:
microvia listed FIRST scores 451.97/59 (equal to no microvia -- preferring a blind 0-1 via
everywhere wastes placements that cannot reach deeper layers); microvia listed as FALLBACK
scores 461.84/56. Ordered-fallback (largest-span first, microvias rescuing tight spots) is
the effective span mechanism with zero engine changes, and the variant fixture now encodes
it. True cost-based span selection inside MazeSearchAlgo's via-mask loop remains future
work relevant only to richer via sets. Remaining gap (56 unrouted vs 55 achievable escapes)
is Phase-2 planning + Phase-3 negotiation; fanout "misses" were re-diagnosed as full-route
failures (fanout = autoroute_connection to the nearest net target, not bare via placement),
so they are the same routing problem, not a separate via-placement defect.

## Phase 2 v1: EscapePlanner built, soundness-validated, fixture-limited (2026-08-09)

EscapePlanner (featureFlags.escapePlanner, default off) inserts nearest-first dogbone
via+stub escapes for pins fanout leaves contactless, wired after the fanout stage.
Validation ran in both directions:

- With pour-passable via placement (WRONG semantics): 25 escapes, unrouted 56 -> 49 (best
  raw connectivity yet) but +12 SCORED violation pairs -- discovered to come from
  DIRECTIONAL obstacle asymmetry: a via inside a foreign pour is clean from the via's side
  while the pour's DRC counts it, and the score counts the pour's side. Fanout's own maze
  (is_drill_obstacle) avoids foreign pours, which is why the baseline adds zero.
- With bidirectional semantics (CORRECT): 0 insertions, 0 added violations -- on THIS
  fixture every remaining unescaped pin's neighbourhood is covered by foreign pours, so
  gate-clean escapes beyond fanout's 248/303 do not exist. The planner correctly declines.

Verdict: items 1-2 machinery is COMPLETE and sound; its positive yield needs either a
fixture whose escape zones are not pour-covered, or pour-REFLOW modelling (treat pours as
regenerable: route through them and subtract at export -- how the source CAD behaves).
Pour-reflow modelling is the newly-discovered real unlock for this class of board and
joins the roadmap as the successor to the microvia variant.

## Pour-reflow modelling implemented; planner gate-clean (2026-08-09)

`featureFlags.reflowablePours` (default off): non-obstacle conduction areas stop counting
DRC pairs against same-board copper (ConductionArea overrides) -- the exporting CAD reflows
them, so nothing routed through them can violate. The planner places inside reflowable
pours when the flag is on.

The final soundness bug was the ATTACH RULE: Via.is_obstacle has no same-net exemption, so
a no-attach via must clear even its OWN pad -- all 24 added pairs were planner vias vs
their own pins (found by pair-diffing two probe runs). With the own-pad check added:
**17 escapes inserted, zero added violations** (exact baseline 1076/160).

Full run (escape+reflow flags): **461.84 / 56 unrouted / 538 pre-existing violations** --
equal to the best configuration, gate-clean. Escaped pads do not yet CONVERT to completed
connections within the 20-minute budget: the escape bottleneck is solved at the placement
level; conversion is Phase-3 (negotiation / budget) work. Items 1-2 of the feature
assessment are complete: detection, feasibility (three semantic corrections, all measured),
span selection (ordered fallback), the planner, and pour-reflow -- each behind flags, each
validated in both directions.

## Phase 3 v1: congestion negotiation implemented (2026-08-09)

`featureFlags.negotiatedRouter` (-Dfr.negotiate, default off): at pass-1 start, every queued
connection is dry-run over the partition room cover (lifted search; the lift-free mode was
measured to find zero routes -- start contacts need pad interiors inside rooms); overused
rooms (usage > width-derived capacity) get priced each round; after the rounds, only
connections whose final routes use no overused room commit, through the fully validated
partition path; the classic loop handles the rest untouched.

First measurement (2-layer fixture): 233 connections, 2 rounds, 182 routed in the final
round, 44 clean candidates, **19 committed, gate score held exactly (989.72 / 2 / 0)** --
more gate-clean partition commits than the per-connection quality mode (11-12). Cost: 33 s
of negotiation, 70.9 s total vs 37.4 s baseline. The pipeline is proven: pricing locates
real contention (182 -> 44), commits are sound, and the remaining work is COST, not
correctness -- the ~70 ms lifted search per dry-run is the same per-attempt anatomy
profiled in the partition campaign, with the same known levers (persistent net overlays
instead of lift/rebuild cycles, cheaper heuristics). Tuning capacity/rounds/pricing and
the 8-layer measurement are the follow-ons.

## Phase 3 v1 hardened: deterministic and gate-clean (2026-08-09)

Three measured corrections landed after the first working version:

1. **Determinism**: the commit phase iterated an identity-hashed map, producing three
   different final scores across identical runs; committing in the stable connection-list
   order restored 2/2 reproducibility.
2. **Acceptance**: the campaign's airline lesson applies to negotiated commits too --
   restricting commits to long connections (>= 150k terminal span) moved the deterministic
   outcome from 984.59/3 to **989.72/2 -- exact gate parity, 8 negotiated commits, 2/2
   identical runs**.
3. **Partition pour semantics**: passable pours stop being partition walls ONLY in reflow
   mode (walling them erased all free space on the pour-covered 8-layer board -- zero dry
   routes; unwalling them without reflow scoring committed corridor-stealing routes,
   989.72/2 -> 974.34/5).

Standing result: negotiation is deterministic, sound, and gate-parity on the 2-layer board
at +33 s cost; on the 8-layer board dry rounds now require reflow mode for rooms to exist.
Follow-ons: cut the ~70 ms lifted dry-run cost (persistent net overlays), tune
capacity/pricing to convert more than 8 commits, and the 8-layer reflow-mode negotiation
measurement.

## Phase 4 opening profile: negotiation at 8-layer scale (2026-08-09)

Reflow-mode negotiation on the 8-layer board (the pour fix works: rooms exist): 153
connections, 91 routed in the final dry round, but pricing left only **2 contention-free
candidates and 0 commits** -- 91 routes share a handful of corridors, i.e. the congestion is
real and extreme, exactly what the board's 56-unrouted difficulty predicts. No regression
(461.84/56/538 held). Cost: 37.3 s for ~306 lifted dry-runs = **~122 ms per search at
8-layer scale** (vs ~70 ms at 2 layers).

Phase-4 work items, now with driving numbers: (1) cut the dry-run cost -- persistent net
overlays instead of lift/rebuild (the 2-layer incremental-maintenance refutation should be
re-measured here, where economics differ); (2) thread the dry rounds (independent searches;
the parallel router's phase-separation machinery applies); (3) negotiation quality at scale
-- with only 2 contention-free routes, the round/pricing schedule needs rip-up-style
iteration (route, price, REROUTE the losers) rather than a single winner-filter, which is
the v2 negotiation design.

## Negotiation v2 (validated arbitration, no pre-filter): REFUTED on 2-layer (2026-08-09)

Dropping the contention pre-filter and letting the sequential validated commit arbitrate:
18 commits (vs 8), deterministic 2/2 -- but 984.59/3, below gate. The winner-filter is
PROTECTIVE: per-commit validation only proves legality, not endgame-compatibility, so the
10 extra contested commits steal corridors. v1 (winner-filter + long-connection gate,
989.72/2) stands. The v2 direction for 8-layer congestion remains iterative
reroute-of-losers WITH the filter, not instead of it.

## Negotiation v2 second variant (reroute-losers) also REFUTED (2026-08-09)

Winner-retention with eviction of contested routes (winners keep routes across rounds;
only losers re-search against risen prices): deterministic 2/2 but 974.34/5 with 6
commits -- worse than v1's full re-search (989.72/2, 8 commits). Early winners lock
corridors that full re-search would have re-optimized; with pricing, re-searching
EVERYONE each round is the better schedule at this scale. v1 stands. Both v2 variants
are now measured refutations; a future v3 would need per-round price decay or randomized
restarts, and belongs with the Phase-4 cost work (fast dry-runs make more rounds
affordable, which is what negotiation quality actually needs).

## Campaign close-out status (2026-08-09)

- Phases 0-3: implemented, measured, gate-verified (see sections above).
- Phase 4: opening profile measured (~122 ms/dry-run at 8 layers); both quick negotiation-v2
  variants refuted; remaining items are the three known builds: persistent net overlays
  (re-measuring the incremental economics at 8-layer scale), threaded dry rounds, and a
  v3 negotiation schedule enabled by cheap dry-runs.
- Phase 5 (diff pairs, length matching, plane/antipad reporting): not started; each is a
  session-scale build with its design sketched in the Phase-5 section above.

Every flag defaults off; both fixture gates verified unbroken after every landing.

## Phase 4 increment: heuristic experiment -- CORRECTION and refutation (2026-08-09)

CORRECTION: the previous version of this section claimed a ~7% win from an O(1)
union-bbox heuristic; the edit had silently failed to apply and the delta was run noise
on unmodified code. The properly-applied edit measured ~31.4 s (marginal vs 33.6) AND
reintroduced nondeterminism (7/984.59 vs 8/989.72 across runs): the tie-prone hull
heuristic exposes identity-ordered HashMap seeding in contact_rooms. REVERTED; the v1
per-target heuristic stands. Verified verdicts: (a) the heuristic is NOT the dry-run
bottleneck -- the lift -> dirty -> wholesale rebuild cycle is, and persistent net overlays
remain Phase 4's single real lever (with deterministic ordering -- sorted contact seeds --
as a prerequisite noted for that build); (b) any future search change must re-verify
determinism explicitly, twice now the silent failure mode of this codepath.

## Phase 4 increment: persistent net overlays -- dry-run cost solved (2026-08-09)

The lift -> dirty -> wholesale rebuild cycle is gone: overlay mode
(`PartitionRouter.try_route(..., p_lift=false)`) never mutates the shared partition.
Own-net transparency moves to CONTACT level -- a contact is accepted when the item's
connection shape intersects the room OR the item's own footprint (`contact_rooms`'
own-footprint rule), which is exactly the region a lifted room would have grown over
(a drill item's pad-centre connection shape always intersects its own pad). Prerequisite
determinism work landed with it: `contact_rooms` iterates items in id order into a
LinkedHashMap, and the A* priority queue tie-breaks equal costs on room index.
Measured in isolation (lifted dry rounds + determinism changes only): exact v1 result
(989.72/2, 8 commits) -- the seeding order change is outcome-neutral.

Two measured consequences of the shared room geometry:

1. Usage is no longer diluted across per-net lifted geometries (in lifted mode, each
   net's terminal rooms were unique objects with usage 1). First overlay run: clean
   candidates collapsed 15 -> 6, 2 commits, **974.34/5 -- below gate**.
2. The fix is PathFinder's own source/sink rule: TERMINAL rooms (a dry route's first and
   last) are exempt from congestion accounting -- a route cannot avoid its own terminal
   room, and in the shared partition a pin-field room is common to every connection
   terminating there, so pricing it only poisons cleanliness without enabling any reroute.

Result with terminal exemption + ROUNDS=8 (now affordable at ~0.7 s/round):
**994.85/1/0 -- ABOVE the 989.72/2 gate, equal to the quality-mode champion** -- with
negotiation at **5.8 s (was 33 s)**, 170 routed in the final round, 17 clean candidates,
3 committed, verified 2/2 identical. Flag-off re-verified at exactly 989.72/2 (35.2 s).
8-layer reflow negotiation: **2.0 s (was 37.3 s, ~18x)**, 109 dry routes in the final
round, 8 clean candidates, 0 committed (span filter + lifted commit re-search still
reject everything at this congestion -- the v3 schedule's problem), full-run gate held.

## Phase 4 increment: threaded dry rounds (2026-08-09)

`featureFlags.negotiatedRouterParallelDry` (-Dfr.negpar, default off): the negotiation's
dry-run searches run on a fixed thread pool (min(8, cores)). Sound because overlay-mode
dry runs are pure reads against the warmed-up partition (`PartitionRouter.warm_up()`
builds every lazy structure before the rounds); deterministic because results are
collected per connection index and processed in connection-list order -- pricing and
winner selection are independent of completion order by construction. Measured on the
2-layer fixture: **identical results to sequential** (170 routed final round, 17 clean,
3 committed, 994.85/1/0), 2/2 identical runs, negotiation wall-clock **5.8 s -> 2.6 s**.
The residual is the sequential remainder (connected-set walks feed the pool from the
submitting loop; per-round A* is already ~3 ms/search).

## Phase 4 item 3: v3 negotiation schedule -- REFUTED; commit bottleneck identified (2026-08-09)

With dry rounds cheap (~0.3 s/round threaded), the v3 schedule hypotheses were finally
affordable to measure. Both refuted on the 2-layer fixture against the standing
994.85/1/0 (ROUNDS=8, monotone prices, 3 commits):

- ROUNDS=16 + price decay 0.7/round: 22 clean candidates, 4 committed, **979.47/4**.
- ROUNDS=16, no decay: 39 clean candidates, 3 committed, **979.47/4**.

More rounds produce MORE clean candidates and WORSE outcomes: the schedule changes which
connections win, and the 8-round schedule's specific winners are the endgame-compatible
ones. The recurring campaign lesson (WHICH routes commit is everything) holds against
schedule sophistication too.

New commit-phase diagnostics (span_rejects / dirty_rejects / already_connected /
attempt_failures / attempt_drc_rejects in the negotiation log) locate the REAL conversion
bottleneck on both fixtures: nearly every clean candidate that fails, fails the
pre-insert DRC check on the MATERIALIZED plan (2-layer: 13 of 17 candidates; 8-layer:
8 of 8 -- hence 0 commits there). Negotiation pricing finds clean corridors; the
windowed-channel materialization geometry is what doesn't survive validation. Raising
the committed count is therefore a materialization-quality problem (channel windows,
aim lines, corner placement), not a schedule problem -- recorded here as the
prerequisite for any future negotiation-conversion work.

## Phase 5 increment 2: return-path + length-matching reports (2026-08-09)

The probe gained a ROUTED-board mode (-Dfr.probe_items / -Dfr.probe_passes / -Dfr.timeout
route the board before reporting; default remains load-only) and two reports:

- **Return-path** ([phase5-return], -Dfr.returnpath_threshold, default 30000 units = 3 mm):
  every signal via is a layer-change point whose return current must also change reference
  planes; the report flags signal vias whose nearest same-net or reference-plane via
  (reference = a via on a net owning ConductionAreas) is farther than the threshold.
  Measured on the 8-layer microvia board routed to pass 3 (445.39/61 state): 69 signal
  vias, 10 reference vias, 4 plane nets, **54/69 layer changes are return-path offenders**
  at 3 mm (worst: Net-(J6-Pin_4) at 15.9 mm) -- the board's return stitching is sparse,
  consistent with only 10 reference vias for 10 planes. On the plane-less 2-layer board
  every signal via reports "none" (no reference vias exist) -- the report is
  structurally correct there but the concept needs planes.

- **Length matching** ([phase5-length]): routed length per net (trace-length sum) and
  per-diff-pair mismatch. 8-layer pass-3 state: 63 routed nets, the one pair
  /Debugger/D+ = 336795 vs D- = 373908, **mismatch 37112 units (3.7 mm)** -- a real
  matching defect a meander pass would need to close.

Both are probe-level (test reporting only, no production code, no flags needed); the
production DRC-report class remains future work per the phase plan.

## Phase 5 increment 3: meander length matching (2026-08-09)

`featureFlags.meanderMatching` (-Dfr.meander, default off), production class
`autoroute/MeanderMatcher`, invoked once after the batch routing loop. For each
name-convention diff pair with routed-length mismatch above 2000 units, 45-degree
triangle-wave meanders are inserted into straight axis-parallel segments of the shorter
member: candidates are every (segment, amplitude) combination ranked by the length each
can actually add, each proposal validated with the board's own `check_polyline_trace`
before the trace is replaced, and replacements inserted SHOVE_FIXED so pull-tight cannot
straighten them away (verified: the meander survives the optimization stage). The matcher
may re-enter its own insertions (remaining straight segments), never traces fixed by
anyone else. Deterministic throughout (name order, id order, gain-ranked candidates with
index tie-breaks).

Measured, 2-layer fixture flag-on: **D+/D- mismatch 43477 -> 30223 (2 bumps, 2/2
identical), final score EXACTLY the 989.72/2/0 gate** -- the added ~1.3 mm of trace does
not move the score and adds no violations; flag-off re-verified at exact baseline.
The residual 30223 is honest scarcity: every further (segment, amplitude, side)
candidate fails DRC validation on this dense board -- closing it needs meanders placed
during routing (wider corridors reserved up front), not post-hoc insertion.

8-layer fixture flag-on (20-min window; the job itself completes at ~12 min, which the
meander stage needs -- a 10-min window measured earlier ended the test mid-pass with the
matcher never reached): **/Debugger/D+ / D- mismatch 37112 -> 23650 (8 bumps), final
461.84/56/538 -- the exact 8-layer gate, zero added violations.** Both fixtures
gate-clean with the flag on.

## Phase 5 item 6 (paired routing): scoped, not started (2026-08-09)

Honest scope from this session: routing the pair centreline through the partition's
windowed channels (the `materialize` machinery) and emitting both offset traces needs
(a) pair-aware connection selection and terminal fan-in from the centreline to each
member's actual pads, (b) a two-polyline offset emission with both sides validated,
(c) commit coordination so a half-committed pair never survives. Each is
`materialize`-grade geometry work with its own reject modes; together they are a full
session. The windowed-channel substrate and the diff-pair detection (MeanderMatcher.
detect_pairs) are in place as the starting points.

## Materialization DRC: diagnosed, repair REFUTED both fixtures (2026-08-09)

The reject diagnostics (`BasicBoard.explain_polyline_trace_reject`, a read-only twin of
`check_polyline_trace` that names the first obstacle; logged as `[materialize-reject]`)
pinned the mechanism: **all 13 2-layer rejects are MID/LAST tile shapes clipping FOREIGN
items -- 10 vias, 2 pins, 1 trace** -- Locate's unchecked dogleg corners escaping the
channel boxes at crossings (inside one box convexity contains the dogleg; between boxes
it can clip a via just outside).

Repairs were built and work mechanically (mirror the dogleg corner c' = a + b - c;
straighten a corner to its neighbours' midpoint; every variant re-validated by the full
DRC before insertion) -- and every conversion was REFUTED by measurement:

- Inline repair (perturbs commit order): 994.85/1 -> 989.72/2.
- Deferred repair (winners commit untouched first, repairs appended after): converted 1,
  994.85/1 -> **979.47/4**.
- 8-layer deferred repair: converted 1, gate 461.84/56/538 -> **451.97/59** -- three nets
  lost to a single repaired commit.

Verdict: the pre-insert DRC check functions as an endgame-compatibility filter; the
routes it rejects are precisely the corridor-stealing ones, on both boards. Repair
machinery removed; the diagnostic layer stays (it is how any future materialization
work will be measured). Champion re-verified after removal: 994.85/1, 3 commits, 2/2.

## Meander corridor reservation: staircase meanders close the pair (2026-08-09)

Two measured findings replacing the "reserve corridors during routing" hypothesis:

1. Pass-1 insertion (the cheap reservation: meanders claim space early, SHOVE_FIXED
   reserves it) inserted **ZERO bumps in both runs** -- at pass-1 state the mismatch reads
   161466 against still-messy routes and every candidate fails DRC. Final-state insertion
   is strictly better; the pass-1 call was removed.
2. The real blocker was the v1 axis-parallel restriction: on a 45-degree-routed board the
   pair's long runs are DIAGONAL. Staircase conversion (a perfect 45-degree run becomes an
   x/y staircase, adding (2 - sqrt(2)) of its axial length while deviating at most one
   step) unlocks them.

One real bug caught by the 8-layer measurement: with the shorter member chosen once and
ceil-rounded gains, a single overshoot flipped which member was shorter and the loop then
lengthened the LONGER member, diverging (37112 -> 43435 over 62 bumps, gate still held).
Fixed by flooring per-call gains to the remaining deficit and recomputing the shorter
member every iteration.

Final results, both gates exact, meanders surviving the optimizer:

- 2-layer: **D+/D- mismatch 43477 -> 448 (33 bumps)** at exactly 989.72/2/0, 2/2
  identical.
- 8-layer: **/Debugger/D+ / D- mismatch 37112 -> 204 (39 bumps)** at exactly
  461.84/56/538, zero added violations.

Both pairs are effectively matched (residuals far below the 2000-unit threshold); the
"reserve corridors during routing" hypothesis is unnecessary at these fixtures' scale.

## Phase 5: production return-path report class (2026-08-09)

`app.freerouting.drc.ReturnPathReport` graduates the probe's [phase5-return] report:
`analyze(board, threshold)` returns structured findings (via id/net/centre/layer span/
distance to nearest same-net or reference-plane via), offenders only, worst-first,
deterministic, read-only -- no participation in routing or scoring, so no flag. The probe
now asserts the production class against its own inline computation (counts and worst
distance); verified on both fixtures: 2-layer 109 signal vias / 0 reference / 109
offenders (all "none" -- no planes), 8-layer routed state **69 signal / 10 reference /
4 plane nets / 54 offenders, worst 159164 units (15.9 mm)** -- exact agreement.

## Channel-tightening experiment: REFUTED (2026-08-09)

The one open geometry avenue after the repair refutation -- contain Locate's doglegs by
narrowing the windowed channels (channel_margin halved to half_width + tolerance + 2) --
was measured on the 2-layer negotiation gate: **rejects went UP (13 -> 14) and
conversions DOWN (3 -> 2)**; the final score stayed 994.85/1 only by the luck of a
different trajectory. Tighter channels do not contain the crossings; they degrade door
and window geometry instead. Reverted (2x margin stands, note in the code); champion
re-verified exactly (994.85/1, 3 commits, 13 rejects). The negotiation-conversion avenue
via channel geometry is closed with this; better geometry means a different realization
mechanism, not margin tuning.

## Pair corridor affinity v1 (2026-08-09)

`featureFlags.pairCorridorAffinity` (-Dfr.pairaffinity, default off): in the negotiation
dry rounds, a diff-pair member's search applies a 10 percent distance discount and a
congestion-price waiver on rooms of its partner's previous-round route -- corridor
coupling with NO geometric dual emission (the avenue the repair refutation closed).
The room-overlap metric ([pair-affinity], logged in both modes) is the A/B instrument.

Measured:
- 8-layer (both members dry-route): overlap **1 shared room (of 3/9) -> 3 shared
  (of 6/3) -- the smaller member's corridor becomes 100 percent shared**; one more dry
  route overall (110 vs 109); commit counts unchanged; **gate exact in both runs
  (461.84/56/538, zero added violations)**.
- 2-layer: inert -- D- (net 46) has no dry route in the final round for reachability
  reasons, so there is no corridor to couple; flag-on is byte-equivalent to flag-off
  (994.85/1, identical counters, 2/2).

Banked as pair corridor affinity v1. Dual offset emission stays ledgered under the
paired-routing scope; converting the shared corridor into committed pair routes is
blocked by the same materialization/commit economics as all negotiation conversion.

## DAC2020 16-layer escape benchmarks online (2026-08-09)

The two unused academic escape benchmarks are characterized, gated, and exercised.

**bm04** (Issue508-DAC2020_bm04.dsn, 16 layers): U32 = 65-pin PERIMETER_RING on a 19x19
5000-pitch footprint (regularity 0.78), U34 = 24-pin ring, 192 SMD pins total; one via
rule (600:300 um, 6000x6000, full 0-15 span, attach_smd=false), clearance 2000, trace
half-width 1000; no planes, no diff pairs, **raw board violations 0**. Post-fanout
unescaped: 39 (U32:31, U34:7, U31:1); spot scan: **35 ESCAPABLE_NOW + 4 ORDERING_VICTIM,
0 rules-impossible** -- real escape targets, unlike Issue732's rules-starved original.

- Baseline (flag-off, 10-min budget; converges in ~88 s over 8 passes):
  **979.01 / 3 unrouted / 0 violations, 2/2 identical** -- the standing 16-layer gate.
- -Dfr.partition: 979.01/3/0 -- parity (+8 s wall).
- -Dfr.escape: planner inserts 14 dogbones, fanout 157 -> 171/192 (81.8 -> 89.1%) -- its
  first positive placement yield without reflow -- but final **972.02/4/0, REGRESSION**:
  the stubs cost one routed net. The campaign law (placement success is not endgame
  benefit) holds at 16 layers; refuted for this board.
- -Dfr.negotiate + negpar: 85 connections, 51 in the final dry round, 3 clean candidates,
  0 commits (all 3 die at the materialization DRC -- the same conversion wall as both
  other fixtures), negotiation cost 0.9 s, final parity 979.01/3/0.
- -Dfr.reflow: skipped -- the board has no conduction areas to reflow.
- Combinations: not justified by the per-flag results (escape regressed; the others are
  exact parity).

**bm09** (Issue508-DAC2020_bm09.dsn, 16 layers): all THROUGH-HOLE peripheral components
(U1 2x20, U12 2x8, U32/U33 1x20 lines, 25400 pitch) -- pins span all layers, so there is
no escape problem at all; no planes, raw violations 0. Baseline **991.38 / 1 / 0 in
3.9 s** -- essentially solved by the classic engine. Partition: parity. Negotiation: all
174 connections dry-route, pricing converges in 4 rounds, 6 clean, **1 commit, parity
held** (991.38/1/0).

Verdict: the machinery is healthy at 16 layers -- overlay negotiation, partition
materialization, planner, and determinism all behave with zero added violations and no
new failure modes; the boards themselves are easier than Issue732 (the classic engine
nearly solves both), so they gate regressions rather than motivate new machinery. The
negotiation conversion wall (materialization DRC) reproduces here exactly (3/3 rejects),
now measured on three boards.

## Escape discipline on bm04: diagnosis, two refutations, the needs filter (2026-08-09)

**Diagnosis** (per-net unrouted diff via the probe's new [probe-unrouted] report, plus
per-escape position logging in the planner): baseline unrouted = {/PB7, /PB6, /MOSI};
with escapes = {GND, /PB7, /PB6, /MISO}. Confirmed against the data: (c) **none of the
planner's escapes served a baseline-unrouted net** -- every one went to a net the classic
engine routes on the surface anyway; (b) about half the nearest-spot stubs ran ALONG the
pad columns into the ring channels; (a) the west column's four vias sat at the identical
offset -- a dead-straight full-span-via wall at x=1482684.

**Detector fixes exposed by the diagnosis** (production PadArrayDetector): the 0.8
regularity gate silently excluded U32 (measured 0.78), depriving its pins of any escape
axis -- lowered to 0.7; and square-pad pins on grid-like footprints (perimeter rings)
now take the BGA-style nearest-boundary OUTWARD axis instead of the row-perpendicular
rule (which pointed east-column ring pads along their own column). 2-layer negotiation
champion re-verified outcome-neutral after both changes (994.85/1, identical counters).

**Placement A/Bs** (bm04, gate 979.01/3/0):
- v1 nearest-spot: 14 escapes, fanout 81.8 -> 89.1 percent, **972.02/4** (the original
  regression).
- Axis-outward nearest: 16 escapes, fanout 90.1 percent, healthier directions (east
  column exits east) -- still **972.02/4**.
- Axis-outward farthest (break the via wall by distance): **965.03/5 -- REFUTED**, longer
  stubs consume more corridor than the wall they avoid.
- **Needs filter** (escape only pins whose nearest unconnected same-net item is beyond
  the campaign's 150k long-connection threshold): **0 escapes inserted, exact baseline
  parity 979.01/3/0 with the identical unrouted set, 2/2** -- the planner correctly
  declines on a board where every escape is pure cost.

**Verdict on bm04**: on a board whose only via is full-span, every escape blocks all 16
layers, so escapes for classic-routable nets are strictly harmful -- the fanout
percentage is a placement metric, not an endgame one (the campaign law, again).
Endgame-positive escape machinery on bm04 means inserting NOTHING; the needs filter
encodes that.

**And the filter turns Issue732 POSITIVE -- the escape planner's first endgame win.**
Where v1 inserted 17 escapes for parity (461.84/56/538), the axis-outward placement plus
needs filter inserts **4** (the long-haul nets only), fanout 248 -> 252/303 (83.2%), and
the full 20-minute run lands at **465.13 / 55 unrouted / 538 violations, 2/2 identical**
-- one more net routed and a higher score than the standing gate, with zero added
violations. Adding negotiation (+negpar) on top: identical 465.13/55/538 (negotiation
commits 0 here, as measured before). The escape stage is now selective enough to pay for
itself exactly where via scarcity is real, and silent where it is not.

Regression checks after the detector changes: 2-layer flag-off **989.72/2/0** (exact
baseline), 2-layer negotiation champion **994.85/1/0** with identical counters, bm04
**979.01/3/0** with the identical unrouted set 2/2.

## Checked corner placement: mechanism works, conversion still loses (2026-08-10)

`featureFlags.checkedRealizer` (-Dfr.checked, default off): when a realized partition plan
fails the pre-insert DRC, the failing tile shape localizes the offending corner (shape i
sits between corners i and i+1) and that corner is retried against a bounded ordered
candidate set -- mirrored dogleg, midpoint straighten, clamp into the plan's windowed
channel box, and the channel-clamped projection onto the aim line -- with the WHOLE
polyline re-validated after each candidate (`first_failing_trace_shape`, a new
index-returning twin of the insertability predicate) and a strict-progress rule (a
candidate is kept only if the first failure moves strictly later). Unlike the refuted
unconditional repair, only failing corners are touched and candidates are confined to the
plan's own known-free space; the pre-insert check still gates the commit.

**The mechanism does what it was asked to do, and the gate still falls:**

| config | rejects | commits | score |
|---|---|---|---|
| 2-layer negotiation | 13 -> **7** | 3 -> **4** | 994.85/1 -> **989.72/2** |
| bm04 negotiation | 3 -> **2** | 0 -> **1** | 979.01/3 -> **972.02/4** |
| 2-layer partition quality mode | -- | -- | 994.85/1 (parity, inert) |
| Issue732 escape+reflow+negotiation | 3 -> 3 (0 repairable) | 0 | 465.13/55/538 (parity, inert) |

Both success conditions on rejects and commits are met on both negotiating boards, and on
both boards **each converted commit costs exactly one routed net**. This is the fourth
independent measurement of the same law (inline repair, deferred repair, 8-layer repair,
now channel-confined checked placement): the pre-insert DRC rejection set is not a
geometry-quality filter that better realization can pass -- it is, empirically, an
endgame-compatibility filter, and a partition route that displaces a classic-engine route
in these corridors is worth about one net.

**Failure taxonomy** (the banked deliverable for any future replacement):

1. **Repairable-by-local-move** -- 6 of 13 on 2-layer, 1 of 3 on bm04, 0 of 3 on
   Issue732. The dogleg excursion leaves the channel and clips a foreign via or pin; any
   of the four candidate families pulls it back. These are realizer defects, and they are
   now fixable.
2. **Unrepairable-in-channel** -- the residual 7 / 2 / 3. No candidate in any of the four
   families clears the obstacle, which means the obstruction overlaps the channel box
   INTERIOR, not merely the corner excursion: the partition's room is free in the
   partition's model but not free for a compensated trace against the live board's tree at
   that moment (items inserted since the plan was searched, plus clearance compensation
   the room cover does not model). No corner placement can fix this class -- only a plan
   whose channels are validated against the live clearance-compensated tree at
   materialization time.
3. **Endgame-toxic-but-legal** -- the class the repairs move INTO commits. Legal by
   construction, validated twice, and still net-negative. Fixing realization does not
   touch this class; only a commit policy that models corridor opportunity cost would.

Class 2 is the only one where better geometry machinery is the answer, and it needs
live-tree channel validation rather than corner repair. Class 3 caps what any realizer can
deliver, which is why the flag stays default off.

## Generalization sweep: are the tuned thresholds general? (2026-08-10)

Flag stack across eight non-gate fixtures, 2-minute budget each, maxItems 500
(score / unrouted / violations; violations are flag-off pre-existing everywhere -- **no
flag added a violation on any fixture**):

| fixture | off | partition | escape | negotiate(+negpar) | escape+reflow |
|---|---|---|---|---|---|
| DAC2020_bm02 | 999.99/0/0 | = | = (0 escapes) | = (0 commits) | = |
| DAC2020_bm05 | 831.77/18/0 | **794.39/22** | = (0 escapes) | **822.42/19** (1 commit) | = |
| DAC2020_bm10 | 999.98/0/0 | 999.99/0 | = (0 escapes) | 999.98/0 (6 commits) | = |
| DAC2020_bm11 | 981.24/3/0 | **987.49/2** | = (0 escapes) | **987.49/2** (1 commit) | = |
| CM5_MINIMA_3 | did not converge in 2 min (all configs) | | | | |
| Issue420-contribution | did not converge in 2 min (all configs) | | | | |
| caniot-tiny-arm | 495.83/96/4 | = | **501.04/95** (26 escapes) | = (0 commits) | **501.04/95** |
| ch32v-tx118s | 1000.00/0/0 | = | = (0 escapes) | = | = |

**Verdict, per threshold:**

- **Escape needs filter (150k airline): GENERAL.** It declines on five of six converging
  fixtures (0 escapes, exact parity) and fires only where long-haul nets exist -- and
  where it fires it WINS: caniot-tiny-arm gains a net (495.83/96 -> 501.04/95 with 26
  escapes, violations unchanged), the planner's second independent endgame win after
  Issue732. A conservative filter that is silent by default and positive when it speaks is
  the behaviour the campaign wanted.
- **Negotiation commit policy (150k span + winner filter): BOARD-DEPENDENT, and this
  refines the earlier reading.** bm11 gains a net from one commit (981.24/3 -> 987.49/2);
  bm05 LOSES one from one commit (831.77/18 -> 822.42/19); bm10 takes six commits with no
  change. So "a partition commit costs a net" is not universal -- it is what happens on
  CONGESTED boards, where committed corridors are contested; where slack exists the same
  machinery pays. The gate boards are all congested, which is why every previous
  conversion measurement came out negative.
- **Partition quality mode: one named regression.** bm05 drops four nets
  (831.77/18 -> 794.39/22) with no added violations -- the largest flag-off-vs-flag-on
  regression in the sweep and the clearest bug-shaped result: worth a dedicated diagnosis
  (bm05 is small and fast, so it is a cheap reproduction case). bm11 conversely gains a
  net, so the mode is not uniformly harmful.
- Two fixtures (CM5_MINIMA_3, Issue420-contribution) need more than a 2-minute budget for
  any configuration; they are size-limited, not failures, and were not diagnosed here.

## Escape quality iteration: board-relative gate replaces the tuned constant (2026-08-10)

**Firing survey** (102 fixtures screened at a 1.5-minute budget with escape+reflow, to
find where the planner speaks at all): **60 silent, 17 firing, 25 that do not reach the
planner in that budget**. The firing set is the iteration corpus: Green14SegLED,
bug-design, Project_GP8B, Board-Unrouted, Protein, smoothieboard, TeamAdapt-LinePCB,
split05, Issue214-freerouting, caniot-tiny-arm, z10_module, Natural_Tone_Preamp,
CE2632_HarryMu, CPU-85_r104, tomu-fpga7/8/11. (Escape COUNTS in that survey span code
versions changed mid-sweep and are indicative only; the per-board airline statistics it
collected are version-independent and are what the calibration below uses.)

**The tuned constant was indeed fixture-tuned.** 150k means 0.06 to 0.24 of the board
diagonal across the corpus, so a diagonal fraction cannot reproduce it. Against the MEDIAN
candidate airline it is nearly invariant on the two boards where escapes pay: 2.09x
(Issue732) and 2.32x (caniot). But the median alone collapses on boards whose airline
distribution is degenerate -- Natural_Tone_Preamp has median 2277 against max 269115, so
2.2x median = 5009, which fired 23 escapes and **regressed the board 637.61/79 ->
633.02/80**.

**Final rule (both terms required, each measured necessary):**
`threshold = max(2.2 * median_candidate_airline, 0.15 * board_diagonal)`.

| board | threshold (term that wins) | escapes | result |
|---|---|---|---|
| Issue732-microvia | 158064 (median) | 4 | **465.13/55/538 -- win held, 2/2 identical** |
| caniot-tiny-arm | 145665 (diagonal floor) | 26 | **501.04/95/4 -- win held** |
| bm04 | 386987 (median; its nets are uniformly short, median 175903 vs max 187085) | 0 | 979.01/3/0 -- exact parity |
| Natural_Tone_Preamp | 117886 (diagonal floor) | 0 | 637.61/79/0 -- **regression eliminated** |

The bm04 case is the principled version of what the absolute constant achieved by luck:
that board's nets are all about the same length, so nothing is long-haul relative to the
board and the planner declines for a stated reason rather than a tuned one.

**Stagger lanes: implemented, verified, neutral.** The earlier "staggered offsets" claim
was never actually in the code -- nearest-out ordering gives neighbouring pads the SAME
offset, which is exactly the via wall measured on bm04. Real lane alternation (pins sorted
by id within a component, alternating near/far outward lane) now exists and is
deterministic; measured on both wins it changes nothing (Issue732 4 escapes 465.13/55/538,
caniot 26 escapes 501.04/95). It is insurance against the wall geometry rather than a
measured gain, kept because it costs nothing.

**Escape planner defaults after this work**: fire when
`airline >= max(2.2 * median airline, 0.15 * board diagonal)`, place outward along the
detector's escape axis at the nearest legal spot in the pin's stagger lane. Both terms of
the gate and the axis direction are now derived from board geometry rather than from
constants tuned on one fixture.

## Phase 5 increment 1: detection/reporting layer (2026-08-09)

The probe now detects differential pairs by net-name convention (_P/_N, +/-, digitP/N) and
reports the plane census. Measured: the 8-layer board has exactly one pair
(/Debugger/D+ with D-, matching its USB_DIFF via rule) and 10 planes across all 8 layers,
all reflowable (get_is_obstacle false -- consistent with the pour-reflow findings); the
2-layer board has one pair (D+/D-) and no planes. Remaining Phase-5 builds, each
session-scale: paired routing (route the pair centreline, offset both traces -- the
partition's windowed channels are a natural substrate since a channel can carry both),
length matching (meander insertion using the partition's free-space knowledge), and
return-path reporting over the plane census.

## bm05 partition quality-mode regression: diagnosed to a single commit; geometry gate refuted (2026-08-12)

The sweep's "clearest bug-shaped result" (bm05 831.77/18/0 off -> 794.39/22/0 with
-Dfr.partition, 4 nets lost) is now diagnosed. Harness notes first: the profile harness
(`RoutingProfileTest`, maxItems 500, maxPasses 8, maxThreads 1) gained the probe's per-net
unrouted report (`[profile-unrouted]`), the quality mode gained a per-commit diagnostic
(`[partition-commit] net/length/airline/detour`, flag-gated log only), and `build.gradle`
now actually forwards `-Dfr.*` from the gradle command line into the forked test JVMs --
without that forwarding the documented `-Dfr.fixture` interface silently ran the default
fixture.

**Reproduction**: flag-off 831.77/18/0 and flag-on 794.39/22/0, both 2/2 identical on
score, violations, and the full per-net unrouted set at the 2-minute budget.

**Diagnosis**: the quality mode participates only in pass 1 (routed=1 fallback=20
drc_reject=8) and its ONE commit is /XADUIO_SCL(#12), realized at length 326560 against
airline 165380 -- **1.97x detour**. The unrouted diff is exactly {+GND x1, +/XAUDIO_SDA x1,
+/XADUIO_SCL x2}: the committed route leaves its OWN net's remaining connections
unroutable and takes the neighbouring I2C partner plus a GND connection with it. Flag-off
reaches its best board in pass 2 (831.77/18, later restored by the end-of-run best-snapshot
restore -- which works correctly in both configs); flag-on's ripup trajectory never
recovers past its own pass 1 (passes 2-7 fall to 719-747).

**The bug-shaped hypothesis -- a detour-ratio acceptance gate -- was implemented and
REFUTED.** Commit detour distributions: bm01 12 commits at 1.07-1.54 plus one at 2.79;
bm11 15 commits at 1.04-1.89; bm05's single commit at 1.97.

| max detour | bm05 | bm01 (2-layer) | bm11 |
|---|---|---|---|
| none (shipped) | 794.39/22 | 994.85/1 | 987.49/2 |
| 1.9 | 822.42/19 (a DIFFERENT commit at 1.72 slips in, -1 net) | 5 unrouted | 987.49/2 |
| 1.5 | **831.77/18 -- exact parity, 0 commits, identical unrouted set** | 984.60/3 | 987.49/2 |

bm05 needs every commit rejected to reach parity; bm01 needs every commit kept (including
the 2.79 one) for its best score. No threshold satisfies both: commit toxicity is a
property of the board's congestion where the corridor is spent, not of the commit's own
geometry. This is the fifth independent measurement of the endgame-compatibility law, and
the gate was reverted -- the 4-net magnitude on bm05 is one sprawling commit displacing
three neighbour connections plus its own net's remainder, the same law that costs one net
per commit elsewhere, amplified by where this particular route sprawls.

**One new signature banked for the future commit policy**: on bm05 BOTH toxic commits left
their own multi-terminal net incomplete (/XADUIO_SCL committed then x2 incomplete;
/XAUDIO_I2S0_DOUT under the 1.9 gate committed then x1 incomplete), while on the winning
boards every committed net completes. A policy that rips up a partition commit whose net
fails to complete by end of pass -- opportunity cost observed rather than predicted -- is
the shaped follow-up; a commit-time geometry filter is not.

**Determinism observation** (pre-existing, both configs, unchanged by this work): score,
violations, and the per-net unrouted set are 2/2 stable, but mid-pass board hashes are NOT
run-to-run stable on bm05 -- pass-2 geometry varies with identical scores, flag-on and
flag-off alike. Traced to the 1000 ms wall-clock pull-tight limit
(`TIME_LIMIT_TO_PREVENT_ENDLESS_LOOP` in `opt_changed_area` after each routed connection),
which makes post-route pull-tight load-sensitive. Hash-level gates on this fixture would
need that addressed; score-level gates are unaffected.

**Gates re-verified on the final (diagnostics-only) code**: bm05 off 831.77/18/0 and on
794.39/22/0 (both 2/2); 2-layer flag-off **989.72/2/0 exact**; 2-layer negotiation
champion **994.85/1/0 exact** (committed=3, attempt_drc_rejects=13, the champion
counters); 2-layer partition 994.85/1; bm11 off 981.24/3 and partition 987.49/2 (win
held); bm04 flag-off 10-minute **979.01/3/0 exact** with the identical unrouted set
{/PB7, /PB6, /MOSI}.

## Phase 1 item 1: region-scoped rules v1 (2026-08-12)

`featureFlags.ruleRegions` (default off) + `router.rule_regions` -- an array of
axis-aligned boxes with a layer set, each carrying a clearance override and optionally a
trace half-width override. Configured through the router settings JSON (the
max_milliseconds_per_item pattern; new `RuleRegionSettings`, documented in
docs/settings.md) or, for the measurement harness, `-Dfr.regions='[{"layers":"*",
"box_um":[x1,y1,x2,y2],"clearance_um":N,"trace_halfwidth_um":M}]'` whose presence enables
the flag. **Units**: micrometers in the DSN coordinate frame (the numbers as read in the
DSN file), resolved at install time with the board's own `coordinate_transform` (scale =
DSN resolution with the parser's overflow reduction; base offset 0; Y keeps the DSN sign
convention), so 80 um resolves to 800 board units on the 10-units/um fixtures. Regions
live on the board (`BasicBoard.rule_regions`, serialized so snapshot rollbacks preserve
them), installed by the BatchAutorouter constructor only when the flag is on -- flag-off
boards never carry regions and every region-aware code path starts with a null check.

**Which engine layers honor regions in v1** (the honest reach statement):

1. **DRC insertability check** (`check_trace_shape`, hence `check_polyline_trace` /
   `first_failing_trace_shape`): an obstacle pair failing the global-clearance query is
   re-checked at the region clearance and waived when legal there. Pair rule: min(global,
   region) applies when AT LEAST ONE shape of the pair lies fully inside a region box on
   the checked layer. The roadmap's v1 sketch ("checked shape fully inside") is this rule
   made symmetric; the STRICTER both-shapes-inside variant was implemented first and
   REFUTED by measurement: fixed copper reaching into an exception zone (the exact
   fine-pitch scenario) always straddles the region boundary, so the cap never applied
   and the region was dead. Uncompensated default tree only (the standard configuration);
   a compensation-enabled default tree skips region logic.
2. **Scored DRC** (`Item.clearance_violations`): the identical pair rule via the shared
   `rule_region_pair_clearance` helper, so the insertability check and the violation count
   cannot disagree and flag-on runs add no scored violations.
3. **The engine, via a region retry** (sequential path, same hook as the neckdown retry,
   tried before it): a connection that fails at global rules and has a terminal item
   intersecting a region box is retried ONCE at the region's rules. The maze search runs
   with a dedicated clearance class appended per region at install time (value = region
   clearance against every class, so the compensated obstacle expansion genuinely shrinks
   -- this is real maze reach, not DRC-only) and with every layer's trace half-width
   clamped to the region half-width. Region SCOPING of that board-wide retry is enforced
   after routing: every newly inserted trace shape not fully inside the region must pass
   the global-clearance check, else the board is rolled back from a snapshot (the strict
   DRC restore machinery) and the original failure stands.

**Not reached in v1, deliberately**: per-region compensation inside a single search (the
retry is all-or-nothing region rules, scoped by trigger + post-check); the parallel
autorouter path; the partition's `min_pass` admissions (only reachable behind the
partition/negotiation flags; deferred); vias (they keep the global via clearance class --
regions neck traces, not vias); the fanout stage. Known v1 exposure: a later connection's
shove can push a region-class trace outside its region without a global re-check there
(same window as any shove; strict_drc catches it when enabled).

**Flag-on demonstration** (new synthetic fixture `fixtures/RuleRegionGap.dsn` + JUnit
`RuleRegionRoutingTest`, since no gate fixture exhibits the lockout cleanly): 2 layers,
one net, a fixed wall with two identical 500-um gaps -- one inside the configured region,
one outside. Global rules (300 um width / 200 um clearance) need 700 um, region rules
(150 um width / 80 um clearance) need 310 um. Measured: (a) a 150-um-wide gap crossing
at 175 um from the wall stubs FAILS `check_polyline_trace` with no regions and PASSES
with the region installed, while the geometrically identical crossing of the
outside-region gap still fails; (b) end-to-end, flag-off finishes **0.00 / 1 unrouted /
0 violations** (wall impassable, score 0 on a 1-net board) and flag-on finishes
**999.99 / 0 unrouted / 0 violations** through the in-region gap via the region retry --
**2/2 identical final board hashes** (also re-demonstrated through the `-Dfr.regions`
harness path). Settings plumbing covered by `RuleRegionsSettingsTest` (Gson round-trip,
clone, merge, defaults).

**Gates after the change, all exact**: bm01 flag-off **989.72/2/0** (unrouted
{ADC12, TXD1}); bm01 negotiation champion **994.85/1/0** with the champion counters
(committed=3, attempt_drc_rejects=13); bm05 flag-off 2-min **831.77/18/0**; bm04
flag-off 10-min **979.01/3/0** with the identical unrouted set {/PB7, /PB6, /MOSI};
full non-slow unit suite green.

**Discovered in passing** (pre-existing, not fixed here): a board whose DSN defines no
via padstack at all NPEs in `AutorouteControl.rebuild_via_info` (null `via_rule`) on
every connection attempt -- the first fixture draft hit it; the shipped fixture defines a
normal via rule instead.
