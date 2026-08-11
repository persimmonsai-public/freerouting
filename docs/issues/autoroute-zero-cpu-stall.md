# Autoroute Items Stall With Near-Zero CPU Until TimeLimit Fires

## Symptom

On a congested production board, most items entering the autoroute stage make no
progress AND consume no CPU — they block for minutes (or until
`router.max_milliseconds_per_item` expires) rather than searching. This is not slow
maze expansion: the process is idle while wall-clock burns.

Observed identically with `feature_flags.parallel_autorouter` **on and off**, so the
defect is in shared engine code, not the parallel orchestration.

## Measurements (mcgyver SOC/HBM test vehicle, 2026-08-10, build 61a2fc39 + per-item-budget patch)

Scope: 78 FPGA<->SOC signal nets (list: `fpga-soc-nets.txt`) freshly ripped up, all
other copper `(type fix)` (~3000 frozen items), 16 plane nets pinned-out, 90 rule-area
keepouts stripped. Headless CLI, fanout+optimizer off, `-mp 1`, `-Xmx6g`.

| Run | Flags | Wall | Total CPU | Outcome |
|---|---|---|---|---|
| A | parallel on | 969 s (pass 1 done) | **10.96 s** | 9/86 routed, 77 unrouted, score 0.00 |
| B | parallel off | >1200 s (killed, pass 1 incomplete) | n/a (idle by sampling) | no .ses |
| C | parallel on + `max_milliseconds_per_item=45000` | 481 s (pass 1 done) | **12.11 s** | 7/79 routed, **63 items abandoned by budget**, one INFO each |

Supporting details:

- Run A's log has a 6-minute gap (13:03:15 -> 13:09:29) between two consecutive
  `InsertFoundConnectionAlgo: via placement blocked by clearance/shove limits`
  lines (net #465 -> net #468) — single items sitting idle for minutes.
- Memory allocation is also near-zero during stalls (run A: 24 GB total allocated
  over 969 s vs. a healthy plane-fanout run's 1209 GB over 973 s on this same
  board) — the engine is neither searching nor allocating.
- Many items also emit `MazeSearchAlgo.init: Failed - no accessible expansion doors
  found (start items: 1, start rooms: 1, completed start rooms: 0, expansion doors
  found: 0, destination doors: 0, ripup_allowed: true)` — those fail *fast*; the
  stalls are the items that get past init and then block.
- In run C the abandons are spread across the scope (UART/SPI/JTAG/TEST_MODE/
  GPIO_TEST, ~1 each) — workload-shaped, not one pathological net.

## Repro

- Fixtures: `iter-130034.dsn` (run A) and `iter-132919.dsn` (run B) — copies live in
  the mcgyver project at `mcgyver/tools/`; import into `fixtures/` as needed.
- Settings: gui off, fanout off, optimizer off, `-mp 1`; toggle
  `feature_flags.parallel_autorouter` to taste; set
  `router.max_milliseconds_per_item=45000` to keep runs bounded.
- Watch `top`/`ps` next to the log: wall advances, CPU does not.

## Investigation starting points

1. **jstack during a stall** — the one measurement not yet taken. With near-zero CPU
   in *serial* mode the thread must be parked/waiting: candidate suspects are a
   `wait()`/`join()` in the item scheduler, a lock shared with the (idle) GUI/board
   observers, TimeLimit polling that sleeps instead of checking inline, or a
   blocking queue handoff that never fills.
2. The stalls follow `via placement blocked by clearance/shove limits` — check
   whether the failed-insert path retries behind a monitor or waits on a condition
   that only another (nonexistent) worker would signal.
3. Run A vs C: with the budget on, wall for pass 1 halves (969 -> 481 s) while CPU is
   unchanged — consistent with items being expired out of a *wait*, not out of a
   computation.

## Acceptance

- Root cause identified and either fixed or documented as expected behavior with
  rationale.
- On the repro fixtures: pass-1 wall-time within ~2x of total CPU time (no
  multi-minute idle gaps), or items that cannot start fail fast at init.
- No regression on DAC2020 fixtures or the plane-fanout path.

## Related

- `docs/issues/parallel-fanout-stage.md` — fanout throughput ask from the same board.
- `router.max_milliseconds_per_item` — mitigation knob added 2026-08-10 (uncommitted
  at time of writing); turns the stall from a run-killer into a logged skip.
- Suggested companion ask (not yet written up): write the `.ses` after every
  completed pass/stage so capped or killed runs keep finished work — four runs on
  this board lost 25-65 min of fanout results to all-or-nothing output.
