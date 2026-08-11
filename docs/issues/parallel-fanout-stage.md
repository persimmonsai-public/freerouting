# Parallelize the `BatchFanout` Stage

## Ask

`feature_flags.parallel_autorouter` parallelizes the autoroute stage, but the fanout
stage (`autoroute/BatchFanout.java`, single `StoppableThread`) still runs one pin at a
time. On plane-heavy boards driven headless with `router.enabled=false` (fanout-only
via-drop runs), fanout is 100% of the job's cost and dominates iteration wall-time.

## Motivating measurements — mcgyver SOC/HBM test vehicle (2026-08-07..10)

150.5x152.9mm 8-layer board, 3366 SMD pins, 0.5mm-pitch 761-pad BGA, 16 plane-backed
nets (GND + 15 rails across L2/L4/L6 GND, L5 sectors, L7 ring). DSN exported from
KiCad 10, rule-area keepouts stripped, all pre-existing wires `(type fix)`.

| Run | Build | Scope | Pass timings | Result |
|---|---|---|---|---|
| stock r2 | v2.3.0 | 2694 pins, all nets | 298s / 713s / 649s | 2503/3366 escaped (74.4%), stage total 28 min |
| planes-only | v2.3.0 | 2157 pins, 16 plane nets | 324s / 583s / 588s | 1814/2157 (84.1%), stage total 25 min |
| planes-only | perf 61a2fc3 | 2157 pins, 16 plane nets | 392s (pass 1) | no speedup observed — expected, stage is serial |

Single-threaded fanout at ~2-6 pins/sec; the host has 10+ cores idle. A ~6-8x
parallel fanout would turn the 25-minute planes iteration into ~4 minutes, which is
the target cadence for iterate-and-graft workflows (export -> route -> import cycles).

## Why it should parallelize well

- Fanout work items are per-pin and mostly local: escape stub + via placement near
  the pad. Cross-pin interaction is limited to via-site contention between
  neighboring pins.
- The autoroute-stage precedent (`parallel_autorouter` racing candidates on
  independent board copies) already solved the shared-board mutation problem;
  fanout could reuse the same recipe, or partition pins spatially (grid cells with
  a serialized boundary-conflict pass) since contention radius is bounded by
  `maxEscapeLengthMm`.
- Per-pass ripup-cost escalation (100/200/300) only matters across passes, not
  within a pass, so intra-pass ordering freedom is high.

## Acceptance

- `feature_flags.parallel_fanout` (default false), honored in headless CLI jobs.
- DAC2020 bm05/bm11 fixtures: identical or better escape rates vs serial.
- mcgyver planes-only fixture: stage wall-time <= 1/4 of serial on an 8-core host,
  escape rate within 1% of serial (deterministic tie-break not required, but
  rollback storms from via-site races must not regress escape rate).
- Existing `[minEscapeLengthMm, maxEscapeLengthMm]` enforcement and
  `isPinEscaped()` semantics unchanged (see smd-pin-fanout-routing.md).

## Related

- `docs/issues/smd-pin-fanout-routing.md` — fanout correctness campaign (escape
  length enforcement, bm05 completeness). This ask is orthogonal: throughput only.
- `feature_flags.parallel_autorouter` — the stage-parallelism precedent.
