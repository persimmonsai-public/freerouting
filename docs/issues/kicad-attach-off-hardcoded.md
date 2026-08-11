# KiCad DSN Export Hardcodes `(attach off)` — Via-in-Pad Impossible From Standard Exports

## Summary

KiCad 10's Specctra exporter writes `(attach off)` on every SMD padstack
unconditionally. The Specctra format supports `(attach on)` (via placement on an
SMD pad), and Freerouting honors it, but no KiCad pad or padstack setting reaches
the exporter — so boards designed for via-in-pad (fine-pitch BGA fields, dense
decap arrays over planes) can never route that way from a stock KiCad export.

Same species as Issue558 (copper-to-edge clearance not exported): the format has
the field, KiCad never emits it, Freerouting inherits the loss.

## Evidence (KiCad 10.0.5, 2026-08-10, mcgyver board)

- Stock export: 30 SMD padstacks, all `(attach off)`, zero `(attach on)`.
- Setting `PAD_PROP_BGA` on all 761 pads of the 0.5mm-pitch BGA (the only
  plausibly relevant pad property; full list checked: BGA, CASTELLATED,
  FIDUCIAL_*, HEATSINK, MECHANICAL, PRESSFIT, TESTPOINT, NONE) and re-exporting:
  identical — still 30x `attach off`, 0x `attach on`. No pad property reaches
  the exporter's attach decision.

## Workaround (in production use)

Post-process the DSN: `txt.replace('(attach off)', '(attach on)')` before routing
(see the mcgyver harness `route_iter.py`, `ITER_VIA_IN_PAD` env knob). Global —
per-padstack selectivity would need a smarter rewrite keyed on padstack ids.

## Related observation for this fork

Permitting `attach on` did NOT accelerate the fanout stage on the mcgyver planes
workload (pass 1: 638 pins/378 s with attach on vs 622 pins/392 s without —
noise). `BatchFanout` apparently does not try a pad-centered via first even when
the padstack allows it; a via-in-pad fast path (pad over plane -> drop via at pad
center, done) would collapse most of that stage's cost on plane-heavy boards.
Candidate follow-up to docs/issues/parallel-fanout-stage.md.

## Fix direction

- KiCad side: emit `(attach on)` when a pad opts in (needs a pad/padstack UI
  property; upstream KiCad change).
- Freerouting side: nothing required for honoring the flag (already works);
  optional job setting `router.force_attach_on=true` would make the DSN
  post-processing unnecessary.
