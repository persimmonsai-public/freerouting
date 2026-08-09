# freerouting performance & BGA-routing campaign — consolidated record

One document indexing everything this campaign measured, built, refuted, and concluded.
Detail lives in `docs/free-space-partition.md` (partition rework, experiment-by-experiment)
and `docs/dense-bga-roadmap.md` (the 8-layer BGA program); this is the map.

## Verified results (gate: 3 isolated runs, --no-daemon --no-build-cache)

Reference fixture `Issue508-DAC2020_bm01.dsn` (2 layers, 578 connections), baseline
37.4 s / score 989.72 / 2 unrouted / 0 violations:

| result | measurement | where |
|---|---|---|
| ~2.5x sequential speedup | one constant: spatial-query stack 40 KB zeroed per query -> capacity 64; bit-identical output | main branch (default ON) |
| Deterministic parallel router | ~2.25x, bulk-synchronous frozen-board search + serial validated commit | `featureFlags.parallelAutorouter` (default off) |
| Partition quality mode | **994.85 / 1 unrouted / 0 violations at ~61 s** (3/3 reproduced): canonical free-space partition, maximal-rectangle room cover, windowed-channel materialization, sentinel-safe attachments, airline-length acceptance gating, pass-1-only attempts | `featureFlags.partitionRouter` (default off) |
| Via-capable multilayer partition search | works end-to-end (0 violations), below champion score on 2 layers | branch `perf/partition-via-support` |

8-layer BGA fixture `Issue732-RoyalBlue54L-Feather.dsn` (10x10 BGA U1 at 0.9 mm), gate
confirmed 3/3 deterministic: 451.97 / 59 unrouted / 538 violations in a 20-minute budget —
with the corrections below.

## The three verdicts that redefined the 8-layer problem (all measured)

1. **All 538 baseline violations are fixture artifacts** — present on the raw unrouted
   board; the router adds zero.
2. **All 57 unescaped SMD pins are rules-impossible** — the design offers one 8000-unit
   through-hole via at 1500 clearance; the candidate-spot scan (pre- and post-fanout, all
   layers, every pin) found zero legal via positions for any of them. **The fanout stage is
   perfect on this board**: 246/303 escaped is exactly the escapable set.
3. Therefore the router's only real gap on this board is the **59 unrouted connections**
   (surface routability of unescaped pins being an open, separate question), and via-escape
   machinery is unexercisable without a **microvia fixture variant** — the single
   prerequisite for the roadmap's Phase 1/2.

## Branch map (remote `persimmons` = persimmonsai-public/freerouting)

| branch | contents |
|---|---|
| `perf/query-stack-and-parallel-autorouter` | main campaign line: all shipped results, both docs, probes, detector, classifier |
| `perf/partition-via-support` | multilayer (layer,room) A* + ExpansionDrill materialization; proven sound, awaiting acceptance tuning on a via-hungry board |
| `perf/partition-incremental-refuted` | incremental cell/room maintenance; property-verified equivalent, measured SLOWER than wholesale on the 2-layer board |
| `perf/partition-sentinel-fix` | history of the sentinel fix + acceptance-policy experiments (merged into main line) |

## Methodology (what made this work)

Every change A/B-measured against a deterministic gate before any claim; every refuted
hypothesis recorded with its numbers (12+ in `free-space-partition.md` — naive retention,
incremental maintenance, validator-strictness theories, amortized invalidation, three
acceptance policies, and more) so nothing gets re-attempted; diagnostics before
implementation — on the 8-layer board, four consecutive one-run diagnostics each redirected
the plan before code was written at the wrong target (violations -> fixture artifacts;
U1 inner balls -> rules-impossible; the "2 fanout misses" -> also rules-impossible; the
whole 57 -> rules-impossible).

## Where the next session starts

1. **Microvia fixture variant** (add ~4000:2000 um layers 0-1 via to the DSN via set) —
   unlocks Phase 1 span-aware vias and the Phase 2 escape planner against a board where
   escapes are legal.
2. **Surface-routability census of the 59 unrouted** — how many are achievable without
   vias; those are Phase 3 (negotiation) targets.
3. **Production feasibility classifier** — move the candidate-spot scan from the probe into
   `PadArrayDetector`, retiring the v1 heuristic.

Tooling: probes and gates take `-Dfr.fixture` and `-Dfr.timeout`; the sandbox JDK21 shims
live in `scratchpad/apply_shims.py` (revert before committing, reapply after).
