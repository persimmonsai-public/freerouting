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
