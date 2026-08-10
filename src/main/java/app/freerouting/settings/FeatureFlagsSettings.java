package app.freerouting.settings;

import com.google.gson.annotations.SerializedName;
import java.io.Serializable;

public class FeatureFlagsSettings implements Serializable {

  @SerializedName("multi_threading")
  public boolean multiThreading = true;
  /**
   * Runs the auto-router itself across multiple worker threads.
   *
   * <p>The router is bulk-synchronous: within a batch every worker searches while the board is
   * frozen, then one thread commits the resulting plans, re-validating each against the live
   * board immediately before writing it. Both halves are load-bearing -- searches read live
   * {@code Item} geometry far beyond the search tree, so they cannot overlap with commits, and a
   * plan searched at the start of a batch can lose its space to an earlier commit in the same
   * batch. Plans that fail re-validation are deferred and retried on the next pass.
   *
   * <p>Measured on the reference fixture at 8 threads: <b>0 DRC violations and 0 errors across
   * 5 runs of a 500-item, 8-pass workload</b> (the previous design produced 0-3 violations in
   * roughly half of runs, plus intermittent exceptions), with byte-identical results run to run
   * -- the sequential path's determinism is preserved. Speedup ~2.25x on 8 threads.
   *
   * <p><b>Defaults to false, and the reason is routing quality, not correctness.</b> On the
   * reference fixture with identical settings (500 items, 8 passes), single-threaded routing
   * finishes at score 989.72 with <b>2 nets unrouted</b>; eight threads finish at 594.86 with
   * <b>79 unrouted</b>. It is roughly 7x faster at producing a substantially worse board.
   *
   * <p>The cause is speculation. Workers search concurrently against a board snapshot, and
   * 75-89% of those searches are then discarded because another connection took the space
   * first. Discarded attempts still consume the pass's item budget, so the router completes
   * far fewer effective ripup passes and converges much less. Benchmarks that hold the item
   * budget fixed report a ~2.25x speedup; that comparison hides the quality gap, and at equal
   * quality this path is not faster at all.
   *
   * <p>Closing that needs workers whose routes provably cannot interact. Confining each worker
   * to a disjoint region of the board was tried and does not apply here: only 12 of 233
   * connections on this fixture fit inside a single region -- the netlist is not spatially
   * local, so nets span the board and cannot be partitioned by area.
   *
   * <p>Distinct from {@link #multiThreading}, which also gates the route optimizer -- that path
   * races candidates on independent board copies and is unaffected.
   */
  @SerializedName("parallel_autorouter")
  public boolean parallelAutorouter = false;
  /**
   * Routes single-layer connections over the maintained free-space partition
   * (docs/free-space-partition.md, stage 2) instead of the lazily built expansion-room
   * decomposition, falling back to the classic engine whenever the partition route fails,
   * needs a layer change, or fails the commit-time geometry re-validation. Off by default
   * until it has passed the score-parity gate on more than one fixture.
   */
  @SerializedName("partition_router")
  public boolean partitionRouter = false;
  /**
   * Deterministic escape pre-routing (EscapePlanner) after the fanout stage: inserts nearest
   * legal dogbone via+stub for every SMD pin fanout left contactless. Default off; see
   * docs/dense-bga-roadmap.md Phase 2.
   */
  public boolean escapePlanner = false;
  /**
   * Pour-reflow modelling: non-obstacle conduction areas (copper pours) are treated as
   * regenerable -- they stop counting DRC pairs against same-board copper, because the
   * exporting CAD reflows them with clearance around whatever is routed through them.
   * Unlocks escapes/routes through pour-covered regions (dense-BGA boards). Default off.
   */
  public boolean reflowablePours = false;
  /**
   * Phase-3 congestion negotiation (PathFinder-style) over the partition room graph: dry-run
   * all pass-1 connections with congestion pricing, iterate until overuse subsides, commit
   * clean winners through the validated partition path. Default off.
   */
  public boolean negotiatedRouter = false;
  /**
   * Runs the negotiation's dry-run searches on a fixed thread pool. Sound because overlay-mode
   * dry runs never mutate the shared partition (own-net transparency is applied at contact
   * level); deterministic because results are collected per connection index and processed in
   * connection-list order, so pricing and winner selection are independent of completion
   * order. Default off; only meaningful with {@link #negotiatedRouter}.
   */
  public boolean negotiatedRouterParallelDry = false;
  /**
   * Post-route differential-pair length matching by meander insertion (Phase 5): after the
   * batch autoroute loop, 45-degree triangle meanders are inserted into the shorter member
   * of each name-convention diff pair, each candidate validated with check_polyline_trace
   * before replacing the trace (inserted SHOVE_FIXED so pull-tight keeps it). Default off.
   */
  public boolean meanderMatching = false;
  /**
   * Pair corridor affinity v1 (negotiation dry rounds): a diff-pair member's search gets a
   * 10 percent distance discount and a congestion-price waiver on rooms of its partner's
   * previous-round route, drawing the pair through adjacent corridors without any
   * geometric dual emission. Only meaningful with {@link #negotiatedRouter}. Default off.
   */
  public boolean pairCorridorAffinity = false;
  @SerializedName("inspection_mode")
  public boolean inspectionMode;
  @SerializedName("other_menu")
  public boolean otherMenu;
  @SerializedName("save_jobs")
  public boolean saveJobs;
}