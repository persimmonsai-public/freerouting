package app.freerouting.autoroute;

import static java.util.Collections.shuffle;

import app.freerouting.autoroute.events.BoardUpdatedEvent;
import app.freerouting.autoroute.events.BoardUpdatedEventListener;
import app.freerouting.autoroute.events.TaskStateChangedEvent;
import app.freerouting.board.BasicBoard;
import app.freerouting.board.ConductionArea;
import app.freerouting.board.Connectable;
import app.freerouting.board.DrillItem;
import app.freerouting.board.Item;
import app.freerouting.board.ItemIdentificationNumberGenerator;
import app.freerouting.board.Pin;
import app.freerouting.board.PolylineTrace;
import app.freerouting.board.RoutingBoard;
import app.freerouting.board.SearchTreeManager;
import app.freerouting.board.ShapeSearchTree;
import app.freerouting.board.Trace;
import app.freerouting.board.Via;
import app.freerouting.core.RouterCounters;
import app.freerouting.core.RoutingJob;
import app.freerouting.core.RoutingJobState;
import app.freerouting.core.StoppableThread;
import app.freerouting.core.scoring.BoardStatistics;
import app.freerouting.datastructures.TimeLimit;
import app.freerouting.datastructures.UndoableObjects;
import app.freerouting.drc.AirLine;
import app.freerouting.drc.DesignRulesChecker;
import app.freerouting.geometry.planar.FloatLine;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntBox;
import app.freerouting.geometry.planar.Point;
import app.freerouting.geometry.planar.Polyline;
import app.freerouting.logger.FRLogger;
import app.freerouting.rules.Net;
import app.freerouting.settings.RouterSettings;
import com.sun.management.ThreadMXBean;
import java.lang.management.ManagementFactory;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Handles the sequencing of the auto-router passes.
 */
public class BatchAutorouter extends NamedAlgorithm {

  // The lowest rank of the board to be selected to go back to.
  // Must not exceed BoardHistory.MAX_HISTORY_SIZE so the check can actually fire.
  private static final int BOARD_RANK_LIMIT = BoardHistory.MAX_HISTORY_SIZE;
  // Maximum number of tries on the same board
  private static final int MAXIMUM_TRIES_ON_THE_SAME_BOARD = 3;
  private static final int TIME_LIMIT_TO_PREVENT_ENDLESS_LOOP = 1000;
  // The minimum number of passes to complete the board, unless all items are
  // routed
  private static final int STOP_AT_PASS_MINIMUM = 8;
  // The modulo of the pass number to check if the improvements were so small that
  // process should stop despite not all items are routed
  private static final int STOP_AT_PASS_MODULO = 4;
  // Number of consecutive passes with no meaningful score improvement before
  // aborting (prevents endless looping when items cannot be routed)
  private static final int STAGNATION_PASS_LIMIT = 10;
  // Number of no-improvement passes before attempting a one-time fanout-tail cleanup.
  private static final int FANOUT_RECOVERY_STAGNATION_PASSES = 3;
  // Minimum score gain (on the 0–1000 normalized scale) that counts as a
  // meaningful improvement; gains smaller than this are treated as stagnation.
  private static final float STAGNATION_SCORE_THRESHOLD = 0.5f;

  private final boolean remove_unconnected_vias;
  private final AutorouteControl.ExpansionCostFactor[] trace_cost_arr;
  private final boolean retain_autoroute_database;
  private final int start_ripup_costs;
  private final int trace_pull_tight_accuracy;
  // Reusable collections to reduce memory churn (thread-safe as each thread has
  // its own BatchAutorouter instance)
  private final List<Item> reusable_autoroute_item_list = new ArrayList<>();
  private final Set<Item> reusable_handled_items = new TreeSet<>();
  protected RoutingJob job;
  private int totalItemsRouted = 0;
  private boolean fanoutTimedOut = false;

  public boolean isFanoutTimedOut() {
    return this.fanoutTimedOut;
  }
  /**
   * Time when the routing session started.
   */
  private Random random;
  /**
   * Used to draw the airline of the current routed incomplete.
   */
  private FloatLine air_line;
  /**
   * Initial number of unrouted nets at the start of the routing session.
   */
  private int initialUnroutedCount;
  /**
   * Time when the routing session started.
   */
  private Instant sessionStartTime;
  private long lastBoardUpdateTimestamp = 0;

  private boolean isOptimizerAutorouter = false;

  public BatchAutorouter(RoutingJob job) {
    this(job.thread, job.board, job.routerSettings, !job.routerSettings.isFanoutEnabled(), true,
        job.routerSettings.get_start_ripup_costs(), job.routerSettings.trace_pull_tight_accuracy);
    this.job = job;
  }

  public BatchAutorouter(StoppableThread p_thread, RoutingBoard board, RouterSettings settings,
      boolean p_remove_unconnected_vias, boolean p_with_preferred_directions, int p_start_ripup_costs,
      int p_pull_tight_accuracy) {
    super(p_thread, board, settings);

    this.random = new Random(0);

    this.remove_unconnected_vias = p_remove_unconnected_vias;
    if (p_with_preferred_directions) {
      this.trace_cost_arr = this.settings.get_trace_cost_arr();
    } else {
      // remove preferred direction
      this.trace_cost_arr = new AutorouteControl.ExpansionCostFactor[this.board.get_layer_count()];
      for (int i = 0; i < this.trace_cost_arr.length; i++) {
        double curr_min_cost = this.settings.get_preferred_direction_trace_costs(i);
        this.trace_cost_arr[i] = new AutorouteControl.ExpansionCostFactor(curr_min_cost, curr_min_cost);
      }
    }

    this.start_ripup_costs = p_start_ripup_costs;
    this.trace_pull_tight_accuracy = p_pull_tight_accuracy;
    // Measured 2026-08: retaining the autoroute database across connections
    // (maintain_database = true) is 2.6x SLOWER on the reference fixture, not faster --
    // retained rooms are carved with search-specific context (ignore shapes, from-doors) and
    // fragment monotonically, so later searches walk ~10x more expansion elements and some
    // fail outright. Do not flip this without redesigning room construction to be
    // context-free with periodic re-coarsening.
    this.retain_autoroute_database = false;

    // Region-scoped rule overrides (Phase 1 item 1): resolve router.rule_regions against the
    // board's coordinate system and install them on the board. Idempotent and flag-gated;
    // with the flag off (the default) the board never carries regions and every region-aware
    // code path is inert.
    if (app.freerouting.Freerouting.globalSettings.featureFlags.ruleRegions) {
      app.freerouting.board.RuleRegion.install(this.board, this.settings);
    }
  }

  /**
   * Auto-routes ripup passes until the board is completed or the auto-router is
   * stopped by the user, or if p_max_pass_count is exceeded. Is currently used in
   * the optimize via batch pass. Returns the
   * number of passes to complete the board or p_max_pass_count + 1, if the board
   * is not completed.
   */
  public static int autoroute_passes_for_optimizing_item(RoutingJob job, int p_max_pass_count, int p_ripup_costs,
      int trace_pull_tight_accuracy, boolean p_with_preferred_directions,
      RoutingBoard updated_routing_board, RouterSettings routerSettings) {
    BatchAutorouter router_instance = new BatchAutorouter(job.thread, updated_routing_board, routerSettings, true,
        p_with_preferred_directions, p_ripup_costs, trace_pull_tight_accuracy);
    router_instance.job = job;
    router_instance.isOptimizerAutorouter = true;

    boolean still_unrouted_items = true;
    int curr_pass_no = 1;
    while (still_unrouted_items && !job.thread.is_stop_auto_router_requested() && curr_pass_no <= p_max_pass_count) {
      still_unrouted_items = router_instance.autoroute_pass(curr_pass_no);
      if (still_unrouted_items && !job.thread.is_stop_auto_router_requested() && updated_routing_board == null) {
      }
      ++curr_pass_no;
    }
    router_instance.remove_tails(Item.StopConnectionOption.NONE);
    if (!still_unrouted_items) {
      --curr_pass_no;
    }
    return curr_pass_no;
  }

  private static Point[] getImpactedPoints(Item item) {
    if (item instanceof Trace trace) {
      return new Point[] { trace.first_corner(), trace.last_corner() };
    }
    if (item instanceof Via via) {
      return new Point[] { via.get_center() };
    }
    if (item instanceof Pin pin) {
      return new Point[] { pin.get_center() };
    }
    if (item instanceof DrillItem drillItem) {
      return new Point[] { drillItem.get_center() };
    }
    return new Point[0];
  }

  private static float getCpuSecondsSnapshot(RoutingJob job) {
    if (job == null || job.resourceUsage == null) {
      return 0f;
    }
    return job.resourceUsage.cpuTimeUsed;
  }

  /**
   * Auto-routes one ripup pass of all items of the board. Returns false, if the
   * board is already completely routed.
   */

  private static float getAllocatedMemoryMbSnapshot(RoutingJob job) {
    if (job == null || job.resourceUsage == null) {
      return 0f;
    }
    return job.resourceUsage.maxMemoryUsed;
  }

  private static float getPeakHeapMbSnapshot(RoutingJob job) {
    if (job == null || job.resourceUsage == null) {
      return 0f;
    }
    return job.resourceUsage.peakMemoryUsed;
  }

  private static float sampleCurrentThreadCpuSeconds() {
    try {
      ThreadMXBean threadMxBean = (ThreadMXBean) ManagementFactory.getThreadMXBean();
      long cpuNanos = threadMxBean.getThreadCpuTime(Thread.currentThread().threadId());
      return cpuNanos < 0 ? -1f : cpuNanos / 1_000_000_000.0f;
    } catch (Throwable t) {
      return -1f;
    }
  }

  private static float sampleCurrentThreadAllocatedMb() {
    try {
      ThreadMXBean threadMxBean = (ThreadMXBean) ManagementFactory.getThreadMXBean();
      threadMxBean.setThreadAllocatedMemoryEnabled(true);
      long allocatedBytes = threadMxBean.getThreadAllocatedBytes(Thread.currentThread().threadId());
      return allocatedBytes < 0 ? -1f : allocatedBytes / (1024.0f * 1024.0f);
    } catch (Throwable t) {
      return -1f;
    }
  }

  private static float sampleHeapUsageMb() {
    try {
      long heapUsed = ManagementFactory.getMemoryMXBean().getHeapMemoryUsage().getUsed();
      return heapUsed / (1024.0f * 1024.0f);
    } catch (Throwable t) {
      return 0f;
    }
  }

  private boolean shouldFireBoardUpdate() {
    long currentTime = System.currentTimeMillis();
    if (currentTime - lastBoardUpdateTimestamp > 250) { // Limit updates to 4 times per second (250ms)
      lastBoardUpdateTimestamp = currentTime;
      return true;
    }
    return false;
  }

  private List<Item> getAutorouteItems(RoutingBoard board) {
    // Reuse instance collections to reduce memory allocation
    reusable_autoroute_item_list.clear();
    return getAutorouteItems(board, reusable_autoroute_item_list);
  }

  /**
   * As above, but collects into the caller's list. The no-argument form reuses one instance
   * collection, which a caller that runs WHILE that list is being iterated (the trial-commit
   * lookahead probes inside a pass) would clear out from under the iteration -- those callers
   * pass their own list instead.
   */
  private List<Item> getAutorouteItems(RoutingBoard board, List<Item> p_target) {
    reusable_handled_items.clear();
    List<Item> autoroute_item_list = p_target;
    Set<Item> handled_items = reusable_handled_items;
    Iterator<UndoableObjects.UndoableObjectNode> it = board.item_list.start_read_object();
    for (;;) {
      UndoableObjects.Storable curr_ob = board.item_list.read_object(it);
      if (curr_ob == null) {
        break;
      }
      if (curr_ob instanceof Connectable && curr_ob instanceof Item curr_item) {
        // This is a connectable item, like PolylineTrace or Pin
        if (!curr_item.is_routable()) {
          if (!handled_items.contains(curr_item)) {

            // Let's go through all nets of this item
            for (int i = 0; i < curr_item.net_count(); i++) {
              int curr_net_no = curr_item.get_net_no(i);
              Set<Item> connected_set = curr_item.get_connected_set(curr_net_no);
              for (Item curr_connected_item : connected_set) {
                if (curr_connected_item.net_count() <= 1) {
                  handled_items.add(curr_connected_item);
                }
              }
              int net_item_count = board.connectable_item_count(curr_net_no);

              // If the item is not connected to all other items of the net, we add it to the
              // auto-router's to-do list
              if ((connected_set.size() < net_item_count) && (!curr_item.has_ignored_nets())) {
                Net net = board.rules.nets.get(curr_net_no);
                // For plane nets: skip items whose connected set already contains a
                // ConductionArea (copper pour). These items would immediately return
                // CONNECTED_TO_PLANE in autoroute_item(), wasting time and causing
                // spurious normalize_traces() failures on nearby stub geometry.
                // Items not yet connected to the plane are still enqueued so they can
                // be routed to the pour in this pass.
                if (net != null && net.contains_plane()) {
                  boolean alreadyConnectedToPlane = connected_set.stream()
                      .anyMatch(connectedItem -> connectedItem instanceof ConductionArea);
                  if (alreadyConnectedToPlane) {
                    continue;
                  }
                }
                autoroute_item_list.add(curr_item);
                String netName = (net != null) ? net.name : "net#" + curr_net_no;
                FRLogger.debug("Queuing item for routing: " + curr_item.getClass().getSimpleName() + " on net '"
                    + netName + "' (connected: " + connected_set.size() + "/" + net_item_count + ")");
              }
            }
          }
        }
      }
    }
    return autoroute_item_list;
  }

  /**
   * Multi-threaded version of the router that routes one ripup pass of all items
   * of the board. WARNING: this version is not working as intended yet. It is a
   * work in progress.
   * <p>
   * Returns false if the board is already completely routed.
   */
  private boolean autoroute_pass_multi_thread(int p_pass_no) {
    try {
      List<Item> autoroute_item_list = getAutorouteItems(this.board);

      // If there are no items to route, we're done
      if (autoroute_item_list.isEmpty()) {
        this.air_line = null;
        return false;
      }

      boolean useSlowAlgorithm = false;

      BatchAutorouterThread[] autorouterThreads = new BatchAutorouterThread[job.routerSettings.maxThreads];
      BoardHistory bh = new BoardHistory(job.routerSettings.scoring);

      // Prepare the threads
      for (int threadIndex = 0; threadIndex < job.routerSettings.maxThreads; threadIndex++) {
        // deep copy the board
        PerformanceProfiler.start("board.deepCopy");
        RoutingBoard clonedBoard = this.board.deepCopy();
        PerformanceProfiler.end("board.deepCopy");

        // clone the auto-route item list to avoid concurrent modification
        List<Item> clonedAutorouteItemList = new ArrayList<>(getAutorouteItems(clonedBoard));

        // shuffle the items to route
        shuffle(clonedAutorouteItemList, this.random);

        autorouterThreads[threadIndex] = new BatchAutorouterThread(clonedBoard, clonedAutorouteItemList, p_pass_no,
            job.routerSettings, this.start_ripup_costs,
            this.trace_pull_tight_accuracy, this.remove_unconnected_vias, true);
        autorouterThreads[threadIndex].setName("Router thread #" + p_pass_no + "." + ThreadIndexToLetter(threadIndex));
        autorouterThreads[threadIndex].setDaemon(true);
        autorouterThreads[threadIndex].setPriority(Thread.MIN_PRIORITY);
      }

      // Update the board on the GUI only based on the first thread
      autorouterThreads[0].addBoardUpdatedEventListener(new BoardUpdatedEventListener() {
        @Override
        public void onBoardUpdatedEvent(BoardUpdatedEvent event) {
          air_line = autorouterThreads[0].latest_air_line;
          fireBoardUpdatedEvent(event.getBoardStatistics(), event.getRouterCounters(), event.getBoard());
        }
      });

      // Start the threads
      for (int threadIndex = 0; threadIndex < job.routerSettings.maxThreads; threadIndex++) {
        // start the thread
        autorouterThreads[threadIndex].start();
      }

      // Wait for the threads to finish
      for (int threadIndex = 0; threadIndex < job.routerSettings.maxThreads; threadIndex++) {
        BatchAutorouterThread autorouterThread = autorouterThreads[threadIndex];

        // wait for the thread to finish
        try {
          autorouterThread.join(TIME_LIMIT_TO_PREVENT_ENDLESS_LOOP);
        } catch (InterruptedException e) {
          job.logError("Autorouter thread #" + p_pass_no + "." + ThreadIndexToLetter(threadIndex) + " was interrupted",
              e);
          this.thread.requestStop();
          break;
        }

        bh.add(autorouterThread.getBoard());

        // calculate the new board score
        BoardStatistics clonedBoardStatistics = autorouterThread
            .getBoard()
            .get_statistics();
        float clonedBoardScore = clonedBoardStatistics.getNormalizedScore(job.routerSettings.scoring);

        job.logDebug("Router thread #" + p_pass_no + "." + ThreadIndexToLetter(threadIndex) + " finished with score: "
            + FRLogger.formatScore(clonedBoardScore,
                clonedBoardStatistics.connections.incompleteCount,
                clonedBoardStatistics.clearanceViolations.totalCount));

        // Aggregate resource usage
        job.resourceUsage.cpuTimeUsed += autorouterThread.cpuTimeUsed;
        job.resourceUsage.maxMemoryUsed += autorouterThread.maxMemoryUsed;
      }

      BatchAutorouterThread bestThread = autorouterThreads[0];
      float bestScore = -Float.MAX_VALUE;

      // Find the best thread
      for (int i = 0; i < job.routerSettings.maxThreads; i++) {
        BoardStatistics stats = autorouterThreads[i].getBoard().get_statistics();
        float score = stats.getNormalizedScore(job.routerSettings.scoring);
        if (score > bestScore) {
          bestScore = score;
          bestThread = autorouterThreads[i];
        }
      }

      this.board = bh.restoreBestBoard();
      bh.clear();

      // Check if we made any progress
      boolean anyProgress = bestThread.getRoutedCount() > 0 || bestThread.getFailedCount() > 0;

      // We are done with this pass
      this.air_line = null;
      return anyProgress;
    } catch (Exception e) {
      job.logError("Something went wrong during the auto-routing", e);
      this.air_line = null;
      return false;
    }
  }

  /**
   * Auto-routes one ripup pass of all items of the board. Returns false, if the
   * board is already completely routed.
   */
  private long partition_routed_count;
  private long partition_fallback_count;
  private long partition_drc_reject_count;

  private static boolean isNegotiatedRouterEnabled() {
    return app.freerouting.Freerouting.globalSettings != null
        && app.freerouting.Freerouting.globalSettings.featureFlags.negotiatedRouter;
  }

  private record NegotiationConn(Item item, int net_no) {
  }

  /**
   * Phase-3 congestion negotiation (docs/dense-bga-roadmap.md): dry-run every queued
   * connection over the partition room cover with congestion pricing (no net-lift, no
   * commits), raise prices on overused rooms each round, and finally commit -- through the
   * fully validated partition path -- only the connections whose final routes use no
   * overused room. Everything else falls to the classic per-connection loop unchanged.
   */
  /**
   * One overlay-mode dry-run search for the negotiation rounds: computes the connection's
   * sets, runs the mutation-free partition search, stores the result at p_index and returns
   * the compensated half width of the found route's layer (1 when no route). Pure reads
   * against the warmed-up partition, so safe to run from the dry pool's worker threads.
   */
  private int dry_route(NegotiationConn p_conn, PartitionRouter.CellRoute[] p_routes, int p_index,
      java.util.Set<String> p_partner_room_keys) {
    p_routes[p_index] = null;
    Set<Item> connected = p_conn.item().get_connected_set(p_conn.net_no());
    Set<Item> unconnected = p_conn.item().get_unconnected_set(p_conn.net_no());
    if (unconnected.isEmpty()) {
      return 1;
    }
    AutorouteControl ctrl = new AutorouteControl(this.board, p_conn.net_no(), settings,
        this.settings.get_via_costs(), this.trace_cost_arr);
    PartitionRouter.CellRoute route;
    try {
      // Overlay search (p_lift=false): own-net transparency applied at contact level
      // instead of the physical lift, so the shared partition is never mutated and the
      // per-search lift -> dirty -> wholesale rebuild cycle disappears. (The original
      // lift-free mode found zero routes because start contacts required the connection
      // shape inside a room -- the overlay's own-footprint contact rule supplies that.)
      route = partition_router.try_route(connected, unconnected,
          ctrl.compensated_trace_half_width, false, p_partner_room_keys);
    } catch (Exception e) {
      route = null;
    }
    p_routes[p_index] = route;
    return route == null ? 1 : Math.max(1, ctrl.compensated_trace_half_width[route.layer]);
  }

  /**
   * The affinity input for one dry-run: the partner net's previous-round room keys, or
   * null when affinity is off, the net is not a pair member, or the partner has no route
   * yet (round 1).
   */
  private static java.util.Set<String> partner_keys_for(NegotiationConn p_conn,
      boolean p_affinity, Map<Integer, Integer> p_pair_partner,
      Map<Integer, java.util.Set<String>> p_route_keys_by_net) {
    if (!p_affinity) {
      return null;
    }
    Integer partner = p_pair_partner.get(p_conn.net_no());
    return partner == null ? null : p_route_keys_by_net.get(partner);
  }

  private void run_negotiation(List<Item> p_items) {
    long t0 = System.currentTimeMillis();
    List<NegotiationConn> conns = new ArrayList<>();
    for (Item item : p_items) {
      for (int i = 0; i < item.net_count(); i++) {
        conns.add(new NegotiationConn(item, item.get_net_no(i)));
      }
    }
    Map<String, Double> prices = new HashMap<>();
    Map<String, Integer> usage = new HashMap<>();
    Map<String, Integer> capacity = new HashMap<>();
    Map<NegotiationConn, PartitionRouter.CellRoute> routes = new HashMap<>();
    // ROUNDS=8 with monotone prices is the measured champion schedule: 16 rounds scored
    // 979.47/4 both with and without per-round price decay 0.7 (39 and 22 clean candidates
    // respectively -- more candidates, worse commits), and decay at 8 rounds was never
    // reached because both 16-round variants already fell below the 989.72/2 gate. See
    // docs/dense-bga-roadmap.md, v3 schedule refutation.
    final int ROUNDS = 8;
    final double OVERUSE_PRICE = 50000;
    if (partition_router == null) {
      if (conns.isEmpty()) {
        return;
      }
      AutorouteControl first_ctrl = new AutorouteControl(this.board, conns.get(0).net_no(),
          settings, this.settings.get_via_costs(), this.trace_cost_arr);
      partition_router = new PartitionRouter(board,
          board.search_tree_manager.get_autoroute_tree(first_ctrl.trace_clearance_class_no));
    }
    partition_router.set_room_prices(prices);
    // Overlay dry runs are pure reads once everything lazy is built; build it before the
    // rounds so the parallel mode's workers never race a lazy initialization.
    partition_router.warm_up();
    boolean parallel_dry = app.freerouting.Freerouting.globalSettings != null
        && app.freerouting.Freerouting.globalSettings.featureFlags.negotiatedRouterParallelDry;
    // Pair corridor affinity v1: diff-pair members get a distance discount and price
    // waiver on their partner's previous-round rooms (flag-gated); the room-overlap
    // metric below is logged in both modes for the A/B.
    boolean pair_affinity = app.freerouting.Freerouting.globalSettings != null
        && app.freerouting.Freerouting.globalSettings.featureFlags.pairCorridorAffinity;
    Map<Integer, Integer> pair_partner = new HashMap<>();
    for (var pair : MeanderMatcher.detect_pairs(board).entrySet()) {
      var p_nets = board.rules.nets.get(pair.getKey());
      var n_nets = board.rules.nets.get(pair.getValue());
      if (p_nets == null || n_nets == null || p_nets.isEmpty() || n_nets.isEmpty()) {
        continue;
      }
      int p_no = p_nets.iterator().next().net_number;
      int n_no = n_nets.iterator().next().net_number;
      pair_partner.put(p_no, n_no);
      pair_partner.put(n_no, p_no);
    }
    // Box keys of each net's previous-round dry route; rebuilt (fresh map) after every
    // round, so in-flight workers only ever read a completed map.
    Map<Integer, java.util.Set<String>> route_keys_by_net = new HashMap<>();
    java.util.concurrent.ExecutorService dry_pool = parallel_dry
        ? java.util.concurrent.Executors.newFixedThreadPool(
            Math.min(8, Runtime.getRuntime().availableProcessors()))
        : null;
    int rounds_run = 0;
    try {
    for (int round = 0; round < ROUNDS; round++) {
      ++rounds_run;
      usage.clear();
      routes.clear();
      // Dry routes for this round, indexed by connection-list position. Searches are
      // independent pure reads (overlay mode); results are assembled in list order below, so
      // the parallel mode is deterministic by construction and identical to sequential.
      PartitionRouter.CellRoute[] round_routes = new PartitionRouter.CellRoute[conns.size()];
      int[] round_half_widths = new int[conns.size()];
      if (dry_pool == null) {
        for (int conn_index = 0; conn_index < conns.size(); conn_index++) {
          round_half_widths[conn_index] = dry_route(conns.get(conn_index), round_routes, conn_index,
              partner_keys_for(conns.get(conn_index), pair_affinity, pair_partner, route_keys_by_net));
        }
      } else {
        List<java.util.concurrent.Future<Integer>> futures = new ArrayList<>(conns.size());
        for (int conn_index = 0; conn_index < conns.size(); conn_index++) {
          final int fi = conn_index;
          final java.util.Set<String> partner_keys =
              partner_keys_for(conns.get(fi), pair_affinity, pair_partner, route_keys_by_net);
          futures.add(dry_pool.submit(() -> dry_route(conns.get(fi), round_routes, fi, partner_keys)));
        }
        for (int conn_index = 0; conn_index < conns.size(); conn_index++) {
          try {
            round_half_widths[conn_index] = futures.get(conn_index).get();
          } catch (Exception e) {
            round_half_widths[conn_index] = 1;
            round_routes[conn_index] = null;
          }
        }
      }
      for (int conn_index = 0; conn_index < conns.size(); conn_index++) {
        PartitionRouter.CellRoute route = round_routes[conn_index];
        if (route == null) {
          continue;
        }
        routes.put(conns.get(conn_index), route);
        int half_width = round_half_widths[conn_index];
        // Terminal rooms (first/last) are exempt from congestion accounting, PathFinder's
        // source/sink rule: a route cannot avoid its own terminal room, and in the shared
        // overlay partition a pin-field room is common to EVERY connection starting there
        // (under the per-net lifted geometry each net saw its own unique terminal rooms, so
        // this sharing never arose; pricing it just poisons cleanliness without enabling
        // any reroute).
        for (int room_index = 1; room_index + 1 < route.rooms.size(); room_index++) {
          FreeSpacePartition.Room room = route.rooms.get(room_index);
          String key = PartitionRouter.box_key(room.box);
          usage.merge(key, 1, Integer::sum);
          int min_dim = Math.min(room.box.ur.x - room.box.ll.x, room.box.ur.y - room.box.ll.y);
          capacity.putIfAbsent(key, Math.max(1, min_dim / (6 * half_width)));
        }
      }
      Map<Integer, java.util.Set<String>> next_keys = new HashMap<>();
      for (Map.Entry<NegotiationConn, PartitionRouter.CellRoute> routed : routes.entrySet()) {
        java.util.Set<String> keys = next_keys.computeIfAbsent(routed.getKey().net_no(),
            k -> new java.util.TreeSet<>());
        for (FreeSpacePartition.Room room : routed.getValue().rooms) {
          keys.add(PartitionRouter.box_key(room.box));
        }
      }
      route_keys_by_net = next_keys;
      boolean overuse = false;
      for (Map.Entry<String, Integer> use : usage.entrySet()) {
        int cap = capacity.getOrDefault(use.getKey(), 1);
        if (use.getValue() > cap) {
          overuse = true;
          prices.merge(use.getKey(), OVERUSE_PRICE * (use.getValue() - cap), Double::sum);
        }
      }
      if (!overuse) {
        break;
      }
    }
    } finally {
      if (dry_pool != null) {
        dry_pool.shutdown();
      }
    }
    // Pair corridor overlap metric (logged in both affinity modes for the A/B): the
    // fraction of the smaller member's rooms shared with its partner's route.
    for (Map.Entry<Integer, Integer> pair : pair_partner.entrySet()) {
      if (pair.getKey() >= pair.getValue()) {
        continue; // each pair once
      }
      java.util.Set<String> keys_a = route_keys_by_net.get(pair.getKey());
      java.util.Set<String> keys_b = route_keys_by_net.get(pair.getValue());
      int shared_count = 0;
      if (keys_a != null && keys_b != null) {
        java.util.Set<String> shared = new java.util.TreeSet<>(keys_a);
        shared.retainAll(keys_b);
        shared_count = shared.size();
      }
      job.logInfo("[pair-affinity] nets=" + pair.getKey() + "/" + pair.getValue()
          + " affinity=" + pair_affinity
          + " rooms=" + (keys_a == null ? "none" : keys_a.size())
          + "/" + (keys_b == null ? "none" : keys_b.size())
          + " shared=" + shared_count);
    }
    // Commit phase: winners re-search WITH the final prices through the validated path.
    int committed = 0;
    int candidates = 0;
    int span_rejects = 0;
    int dirty_rejects = 0;
    int already_connected = 0;
    int attempt_failures = 0;
    long drc_rejects_before = partition_drc_reject_count;
    // Iterate the deterministic connection list, not the identity-hashed map: commit order
    // changes outcomes (measured: three different final scores across identical runs).
    this.commit_usage = usage;
    try {
    for (NegotiationConn conn_key : conns) {
      PartitionRouter.CellRoute dry_route = routes.get(conn_key);
      if (dry_route == null) {
        continue;
      }
      Map.Entry<NegotiationConn, PartitionRouter.CellRoute> entry = Map.entry(conn_key, dry_route);
      // The campaign's measured acceptance lesson applies to negotiation commits too: short
      // local connections are cheap for the classic engine and their greedy partition
      // versions fragment endgame corridors. Commit long connections only.
      var first_room = dry_route.rooms.get(0).box;
      var last_room = dry_route.rooms.get(dry_route.rooms.size() - 1).box;
      double span_x = (first_room.ll.x + first_room.ur.x) / 2.0 - (last_room.ll.x + last_room.ur.x) / 2.0;
      double span_y = (first_room.ll.y + first_room.ur.y) / 2.0 - (last_room.ll.y + last_room.ur.y) / 2.0;
      if (Math.hypot(span_x, span_y) < 150000) {
        ++span_rejects;
        continue;
      }
      boolean clean = true;
      List<FreeSpacePartition.Room> route_rooms = entry.getValue().rooms;
      for (int room_index = 1; room_index + 1 < route_rooms.size(); room_index++) {
        String key = PartitionRouter.box_key(route_rooms.get(room_index).box);
        if (usage.getOrDefault(key, 0) > capacity.getOrDefault(key, 1)) {
          clean = false;
          break;
        }
      }
      if (!clean) {
        ++dirty_rejects;
        continue;
      }
      ++candidates;
      NegotiationConn conn = entry.getKey();
      Set<Item> connected = conn.item().get_connected_set(conn.net_no());
      Set<Item> unconnected = conn.item().get_unconnected_set(conn.net_no());
      if (unconnected.isEmpty()) {
        ++already_connected;
        continue; // connected as a side effect of an earlier negotiation commit
      }
      AutorouteControl ctrl = new AutorouteControl(this.board, conn.net_no(), settings,
          this.settings.get_via_costs(), this.trace_cost_arr);
      // The airline feeds the per-commit diagnostics only (detour, board-diagonal fraction);
      // it is the same minimum terminal-pair distance the per-item path measures.
      AutorouteAttemptResult result = try_partition_route(connected, unconnected, ctrl,
          airline_distance(unconnected, connected));
      if (result != null && result.state == AutorouteAttemptState.ROUTED) {
        ++committed;
      } else {
        ++attempt_failures;
      }
    }
    } finally {
      this.commit_usage = null;
    }
    // NOTE (measured, both fixtures): retrying the DRC-rejected candidates with the
    // mirrored-dogleg/straighten repairs converts them mechanically, but every converted
    // commit was endgame-toxic -- 2-layer 994.85/1 -> 979.47/4 (repair appended AFTER the
    // untouched winner order), 8-layer 461.84/56 -> 451.97/59 from a single repaired
    // commit. The pre-insert DRC check is in effect an endgame-compatibility filter; see
    // docs/dense-bga-roadmap.md (materialization repair refutation).
    partition_router.set_room_prices(null);
    job.logInfo("[negotiation] connections=" + conns.size() + " rounds=" + rounds_run
        + " routed_in_final_round=" + routes.size() + " clean_candidates=" + candidates
        + " committed=" + committed
        + " span_rejects=" + span_rejects + " dirty_rejects=" + dirty_rejects
        + " already_connected=" + already_connected + " attempt_failures=" + attempt_failures
        + " attempt_drc_rejects=" + (partition_drc_reject_count - drc_rejects_before)
        + " checked_repairs=" + partition_checked_repair_count
        + commit_policy_report()
        + live_channel_report()
        + lookahead_report()
        + " in " + (System.currentTimeMillis() - t0) + " ms");
  }

  /**
   * The commit-policy counters, or the empty string when the flag is off (so flag-off log
   * output stays byte-identical).
   */
  private String commit_policy_report() {
    if (!isCommitPolicyEnabled()) {
      return "";
    }
    return " policy_mode=" + COMMIT_POLICY_MODE
        + " policy_declines=" + partition_policy_decline_count;
  }

  /**
   * The minimum centre distance between the drill items of two connection sets -- the same
   * airline {@code calc_airline} computes for the per-item path, without touching the
   * autorouter's {@code air_line} field (the negotiation runs outside that state).
   */
  static double airline_distance(Collection<Item> p_from_items, Collection<Item> p_to_items) {
    double min_square = Double.MAX_VALUE;
    for (Item from_item : p_from_items) {
      if (!(from_item instanceof DrillItem)) {
        continue;
      }
      FloatPoint from_corner = ((DrillItem) from_item).get_center().to_float();
      for (Item to_item : p_to_items) {
        if (!(to_item instanceof DrillItem)) {
          continue;
        }
        double distance = from_corner.distance_square(((DrillItem) to_item).get_center().to_float());
        if (distance < min_square) {
          min_square = distance;
        }
      }
    }
    return min_square == Double.MAX_VALUE ? -1 : Math.sqrt(min_square);
  }

  /**
   * The live channel validation counters, or the empty string when the flag is off (so
   * flag-off log output stays byte-identical).
   */
  private String live_channel_report() {
    if (partition_router == null || app.freerouting.Freerouting.globalSettings == null
        || !app.freerouting.Freerouting.globalSettings.featureFlags.liveChannelValidation) {
      return "";
    }
    long[] counters = partition_router.live_channel_counters();
    return " live_plans=" + counters[0] + " live_plan_rejects=" + counters[1]
        + " live_occupied_channels=" + counters[2] + " live_stale_channels=" + counters[3]
        + " live_shrinks=" + counters[4];
  }
  /**
   * Checked corner placement for a realized partition trace that failed the pre-insert DRC.
   * The failing tile shape localizes the offending corner (shape i lies between corners i
   * and i+1); each involved interior corner is retried against a bounded ordered candidate
   * set, and the WHOLE polyline is re-validated after every candidate, so nothing is
   * accepted that the board's own insertability predicate would not accept. Candidates, in
   * order: mirrored dogleg (c' = a + b - c), midpoint straighten, clamp into the plan's
   * windowed channel box (known-free space), and the channel-clamped projection of c onto
   * the aim line a->b.
   *
   * <p>Distinct from the refuted unconditional repair: only corners that actually fail are
   * touched, candidates are confined to the plan's own free space, and the pre-insert check
   * still gates the commit afterwards.
   */
  private boolean checked_corner_repair(LocateFoundConnectionAlgo.ResultItem p_trace,
      AutorouteControl p_ctrl, List<app.freerouting.geometry.planar.IntBox> p_channels) {
    app.freerouting.geometry.planar.IntPoint[] corners = p_trace.corners;
    if (corners == null || corners.length < 3) {
      return false;
    }
    int layer = p_trace.layer;
    int half_width = p_ctrl.trace_half_width[layer];
    int[] net_arr = new int[]{p_ctrl.net_no};
    for (int attempt = 0; attempt < 4; attempt++) {
      int failing_shape;
      try {
        failing_shape = board.first_failing_trace_shape(new Polyline(corners), layer, half_width,
            net_arr, p_ctrl.trace_clearance_class_no);
      } catch (Exception e) {
        return false;
      }
      if (failing_shape < 0) {
        ++partition_checked_repair_count;
        return true;
      }
      boolean progressed = false;
      // Shape i sits between corners i and i+1; only interior corners may move (the
      // terminals are the attachment points Locate aimed at).
      for (int k = failing_shape; k <= failing_shape + 1 && !progressed; k++) {
        if (k < 1 || k + 1 >= corners.length) {
          continue;
        }
        app.freerouting.geometry.planar.IntPoint a = corners[k - 1];
        app.freerouting.geometry.planar.IntPoint c = corners[k];
        app.freerouting.geometry.planar.IntPoint b = corners[k + 1];
        app.freerouting.geometry.planar.IntBox channel = channel_of(p_channels, c);
        List<app.freerouting.geometry.planar.IntPoint> candidates = new ArrayList<>(4);
        candidates.add(new app.freerouting.geometry.planar.IntPoint(a.x + b.x - c.x, a.y + b.y - c.y));
        candidates.add(new app.freerouting.geometry.planar.IntPoint((a.x + b.x) / 2, (a.y + b.y) / 2));
        if (channel != null) {
          candidates.add(clamp_into(channel, c));
          double dx = b.x - (double) a.x;
          double dy = b.y - (double) a.y;
          double len_sq = dx * dx + dy * dy;
          if (len_sq > 0) {
            double t = ((c.x - (double) a.x) * dx + (c.y - (double) a.y) * dy) / len_sq;
            t = Math.max(0, Math.min(1, t));
            candidates.add(clamp_into(channel, new app.freerouting.geometry.planar.IntPoint(
                (int) Math.round(a.x + t * dx), (int) Math.round(a.y + t * dy))));
          }
        }
        for (app.freerouting.geometry.planar.IntPoint candidate : candidates) {
          if (candidate.equals(c) || candidate.equals(a) || candidate.equals(b)) {
            continue;
          }
          corners[k] = candidate;
          int now_failing;
          try {
            now_failing = board.first_failing_trace_shape(new Polyline(corners), layer, half_width,
                net_arr, p_ctrl.trace_clearance_class_no);
          } catch (Exception e) {
            now_failing = failing_shape;
          }
          if (now_failing < 0) {
            ++partition_checked_repair_count;
            return true;
          }
          if (now_failing > failing_shape) {
            progressed = true; // strictly later failure: keep this corner and continue
            break;
          }
          corners[k] = c;
        }
      }
      if (!progressed) {
        return false;
      }
    }
    return false;
  }

  private static app.freerouting.geometry.planar.IntBox channel_of(
      List<app.freerouting.geometry.planar.IntBox> p_channels,
      app.freerouting.geometry.planar.IntPoint p_point) {
    app.freerouting.geometry.planar.IntBox best = null;
    double best_distance = Double.MAX_VALUE;
    for (app.freerouting.geometry.planar.IntBox box : p_channels) {
      if (p_point.x >= box.ll.x && p_point.x <= box.ur.x
          && p_point.y >= box.ll.y && p_point.y <= box.ur.y) {
        return box;
      }
      double cx = (box.ll.x + box.ur.x) / 2.0;
      double cy = (box.ll.y + box.ur.y) / 2.0;
      double distance = Math.hypot(cx - p_point.x, cy - p_point.y);
      if (distance < best_distance) {
        best_distance = distance;
        best = box;
      }
    }
    return best;
  }

  private static app.freerouting.geometry.planar.IntPoint clamp_into(
      app.freerouting.geometry.planar.IntBox p_box,
      app.freerouting.geometry.planar.IntPoint p_point) {
    return new app.freerouting.geometry.planar.IntPoint(
        Math.max(p_box.ll.x, Math.min(p_box.ur.x, p_point.x)),
        Math.max(p_box.ll.y, Math.min(p_box.ur.y, p_point.y)));
  }

  private long partition_checked_repair_count;

  /**
   * Pre-validates every located trace of a realized plan with the board's own insertability
   * predicate (check_polyline_trace -> check_trace_shape with contact pins) -- exactly what
   * insert_forced_trace_polyline will enforce. This carries the pad-exit/tie-pin exemptions a
   * plain clearance query cannot model, and a plan that fails it would otherwise be refused by
   * the insert itself only after destructive shove attempts (measured: ~140 shove-insert
   * failures per pass cost 58 s and left transient violations). A plan that passes inserts
   * cleanly without shoving.
   *
   * <p>p_count is false for a non-final realization attempt, whose failure is not the
   * connection's outcome and must not move the reject counters or the reject log.
   */
  private boolean insertable_plan(LocateFoundConnectionAlgo p_located, AutorouteControl p_ctrl,
      boolean p_count) {
    for (LocateFoundConnectionAlgo.ResultItem located_trace : p_located.connection_items) {
      if (located_trace.corners == null || located_trace.corners.length < 2) {
        continue;
      }
      boolean insertable;
      try {
        Polyline trace_polyline = new Polyline(located_trace.corners);
        insertable = board.check_polyline_trace(trace_polyline, located_trace.layer,
            p_ctrl.trace_half_width[located_trace.layer], new int[]{p_ctrl.net_no},
            p_ctrl.trace_clearance_class_no);
      } catch (Exception e) {
        // Degenerate corner list -- not insertable as planned.
        insertable = false;
      }
      if (!insertable && app.freerouting.Freerouting.globalSettings != null
          && app.freerouting.Freerouting.globalSettings.featureFlags.checkedRealizer) {
        insertable = checked_corner_repair(located_trace, p_ctrl,
            partition_router.last_channel_boxes());
      }
      if (!insertable) {
        if (!p_count) {
          return false;
        }
        ++partition_drc_reject_count;
        ++partition_fallback_count;
        if (isNegotiatedRouterEnabled()) {
          try {
            Polyline reject_polyline = new Polyline(located_trace.corners);
            String reason = board.explain_polyline_trace_reject(reject_polyline, located_trace.layer,
                p_ctrl.trace_half_width[located_trace.layer], new int[]{p_ctrl.net_no},
                p_ctrl.trace_clearance_class_no);
            job.logInfo("[materialize-reject] net=" + p_ctrl.net_no + " " + reason);
          } catch (Exception e) {
            job.logInfo("[materialize-reject] net=" + p_ctrl.net_no + " degenerate corners");
          }
        }
        return false;
      }
    }
    return true;
  }

  /**
   * Attempts the connection over the free-space partition. Returns a ROUTED result on success,
   * or null when the classic engine should handle the connection instead (no route found in the
   * partition, unsupported case, degenerate door chain, or the plan lost the commit-time
   * geometry re-validation because the partition was stale). p_airline_distance is the
   * connection's airline when the caller has it (per-item path) or negative (negotiation
   * path); it feeds the [partition-commit] diagnostic only. A detour-ratio acceptance gate
   * on it was measured and REFUTED 2026-08-12: no threshold separates bm05's toxic commits
   * (1.97x and 1.72x airline, -4 and -1 nets) from bm01's beneficial ones (up to 2.79x) --
   * see docs/dense-bga-roadmap.md, bm05 diagnosis.
   */
  private AutorouteAttemptResult try_partition_route(Set<Item> p_start_set, Set<Item> p_dest_set,
      AutorouteControl p_ctrl, double p_airline_distance) {
    try {
      ShapeSearchTree tree = board.search_tree_manager.get_autoroute_tree(p_ctrl.trace_clearance_class_no);
      if (partition_router == null) {
        partition_router = new PartitionRouter(board, tree);
      }
      PartitionRouter.CellRoute route = partition_router.try_route(p_start_set, p_dest_set,
          p_ctrl.compensated_trace_half_width);
      if (route == null) {
        ++partition_fallback_count;
        return null;
      }
      SortedSet<Item> no_ripped = new TreeSet<>();
      // Under live channel validation the plan is realized twice at most: first from the
      // channels the validation shrank to the live-free sub-boxes, and -- when that plan
      // cannot be realized at all -- once more from the channels as searched, so validation
      // can only ADD realizable plans, never remove one the unvalidated pipeline had. The
      // pre-insert DRC gates both.
      boolean live_validation = app.freerouting.Freerouting.globalSettings != null
          && app.freerouting.Freerouting.globalSettings.featureFlags.liveChannelValidation;
      int attempt_count = live_validation ? 2 : 1;
      LocateFoundConnectionAlgo located = null;
      for (int attempt = 0; attempt < attempt_count && located == null; attempt++) {
        boolean last_attempt = attempt + 1 == attempt_count;
        MazeSearchAlgo.Result seed = partition_router.materialize(route, p_ctrl,
            live_validation && attempt == 0);
        if (seed == null) {
          if (last_attempt) {
            ++partition_fallback_count;
            return null;
          }
          continue;
        }
        LocateFoundConnectionAlgo candidate = LocateFoundConnectionAlgo.get_instance(seed, p_ctrl, tree,
            board.rules.get_trace_angle_restriction(), no_ripped, null);
        if (candidate == null || candidate.connection_items == null
            || candidate.connection_items.isEmpty()) {
          if (last_attempt) {
            ++partition_fallback_count;
            return null;
          }
          continue;
        }
        if (!insertable_plan(candidate, p_ctrl, last_attempt)) {
          if (last_attempt) {
            return null;
          }
          continue;
        }
        located = candidate;
      }
      if (located == null) {
        return null;
      }
      AutorouteEngine commit_engine = new AutorouteEngine(board, tree, false);
      AutorouteEngine.ConnectionPlan plan = AutorouteEngine.ConnectionPlan.found(located, null);
      // Trial-commit lookahead, trial B (DECLINE): the horizon from the board WITHOUT this
      // candidate. Measured before the commit exists so both trials read a board the router
      // actually reached. A probe needs a pass to be under way and must not recurse.
      boolean lookahead = isCommitLookaheadEnabled() && !this.in_lookahead
          && this.lookahead_pass_active && !this.thread.is_stop_auto_router_requested();
      int decline_incompletes = -1;
      String pre_probe_hash = null;
      String post_probe_hash = null;
      if (lookahead) {
        // The probe routes a clone, so the live board must come out untouched; both digests
        // are logged so any leak is visible in the log rather than silently poisoning the
        // next candidate's baseline.
        pre_probe_hash = copper_digest();
        decline_incompletes = lookahead_probe();
        // Repeatability experiment (-Dfr.lookahead.repeat=N, measurement only): re-run the
        // SAME probe on the SAME board N-1 more times. If the horizon is a function of the
        // board, every repeat agrees; the spread of these numbers is the resolution limit of
        // the whole mechanism. The first value is still the one the decision uses, so the
        // experiment does not change any decision.
        for (int repeat = 1; repeat < LOOKAHEAD_REPEAT; repeat++) {
          job.logInfo("[lookahead-repeat] net=" + p_ctrl.net_no + " decline#" + (repeat + 1)
              + "=" + lookahead_probe() + " (first=" + decline_incompletes
              + ", board=" + pre_probe_hash + ")");
        }
        post_probe_hash = copper_digest();
      }
      // The pre-check above makes insert failure rare; the snapshot is a safety net so a
      // residual mid-chain insert failure cannot leave a partial connection on the board.
      board.generate_snapshot();
      AutorouteAttemptResult result = commit_engine.commit_connection(plan, p_ctrl, no_ripped, false);
      if (result.state != AutorouteAttemptState.ROUTED) {
        board.undo(null);
        partition_router.invalidate();
        ++partition_fallback_count;
        return null;
      }
      // Per-commit diagnostic (flag-gated paths only): the realized length against the
      // airline identifies which commit displaced what in an A/B unrouted-set diff.
      double located_length = 0;
      for (LocateFoundConnectionAlgo.ResultItem located_trace : located.connection_items) {
        if (located_trace.corners == null) {
          continue;
        }
        for (int i = 1; i < located_trace.corners.length; i++) {
          located_length += located_trace.corners[i].to_float()
              .distance(located_trace.corners[i - 1].to_float());
        }
      }
      // The feature vector is measured on the COMMITTED board (own-net completion is only
      // observable after the connection exists) and before the snapshot is popped, so a
      // declining policy can still roll the commit back.
      CommitFeatures features = commit_features(route, p_ctrl, p_start_set, located_length,
          p_airline_distance);
      if (isCommitPolicyEnabled() && !commit_policy_accepts(features)) {
        job.logInfo("[commit-policy] declined " + features);
        board.undo(null);
        partition_router.invalidate();
        ++partition_policy_decline_count;
        ++partition_fallback_count;
        return null;
      }
      // Trial-commit lookahead, trial A (ACCEPT): the same horizon from the committed board.
      // The post-commit pull-tight the real path runs at the end is brought forward to here so
      // the probed board is the board the pass would actually continue from; it sits inside the
      // commit snapshot, so a rolled-back commit takes its pull-tight with it.
      if (lookahead) {
        board.opt_changed_area(new int[0], null, this.trace_pull_tight_accuracy,
            p_ctrl.trace_costs, this.thread, TIME_LIMIT_TO_PREVENT_ENDLESS_LOOP);
        int accept_incompletes = lookahead_probe();
        boolean tie = accept_incompletes == decline_incompletes + LOOKAHEAD_MARGIN;
        boolean keep = lookahead_keeps(accept_incompletes, decline_incompletes,
            LOOKAHEAD_MARGIN, LOOKAHEAD_TIE);
        if (tie) {
          ++lookahead_tie_count;
        }
        job.logInfo("[lookahead] net=" + p_ctrl.net_no
            + " horizon_incompletes accept=" + accept_incompletes
            + " decline=" + decline_incompletes
            + (tie ? " (tie)" : "") + " -> " + (keep ? "keep" : "roll back")
            + " board=" + pre_probe_hash
            + (pre_probe_hash != null && pre_probe_hash.equals(post_probe_hash)
                ? " restored" : " RESTORE-MISMATCH=" + post_probe_hash));
        if (!keep) {
          board.undo(null);
          partition_router.invalidate();
          ++lookahead_decline_count;
          ++partition_fallback_count;
          return null;
        }
        ++lookahead_accept_count;
      }
      board.pop_snapshot();
      ++partition_routed_count;
      job.logInfo("[commit-feature] " + features);
      Net committed_net = board.rules.nets.get(p_ctrl.net_no);
      job.logInfo("[partition-commit] net=" + (committed_net != null ? committed_net.name : "?")
          + "(#" + p_ctrl.net_no + ") length=" + Math.round(located_length)
          + " airline=" + (p_airline_distance > 0 ? String.valueOf(Math.round(p_airline_distance)) : "?")
          + " detour=" + (p_airline_distance > 0
              ? String.format("%.2f", located_length / p_airline_distance) : "?"));
      partition_router.invalidate();
      if (!lookahead) {
        // Already done above when the lookahead ran, so the accept probe could see it.
        board.opt_changed_area(new int[0], null, this.trace_pull_tight_accuracy, p_ctrl.trace_costs,
            this.thread, TIME_LIMIT_TO_PREVENT_ENDLESS_LOOP);
      }
      return result;
    } catch (Exception e) {
      FRLogger.error("PartitionRouter attempt failed; falling back to the classic engine", e);
      if (partition_router != null) {
        partition_router.invalidate();
      }
      ++partition_fallback_count;
      return null;
    }
  }

  /**
   * Whether this pass should route items in parallel across multiple worker threads instead of
   * one at a time. Gated behind the existing {@code featureFlags.multiThreading} toggle (already
   * used to gate the route optimizer's own thread pool) and {@code settings.maxThreads > 1} so
   * that single-threaded behavior (and its exact sequential/deterministic item ordering) remains
   * the default unless a caller explicitly opts in.
   *
   * <p>Known limitations of the parallel path, kept out of scope for this first implementation:
   * no strict-DRC snapshot/restore, no width-necking retry, no per-item airline/debug-trace
   * output, and no cross-item ripup-cost accumulation ordering guarantee (results are not
   * bit-for-bit reproducible run-to-run -- see {@code autoroute_pass_parallel}'s javadoc).
   */


  /**
   * The commit-time feature vector of one partition-originated commit -- everything cheap
   * that could plausibly discriminate an endgame-positive commit from an endgame-toxic one.
   * Measured on the committed board (so {@link #completes_net} is an observation, not a
   * prediction) and logged as {@code [commit-feature]} on every flag-gated commit, which is
   * how the feature-vs-outcome table in docs/dense-bga-roadmap.md was built.
   */
  static final class CommitFeatures {
    String net_name = "?";
    int net_no;
    /** Whether the net has NO unconnected terminal left after this commit. */
    boolean completes_net;
    /** Terminal (drill item) count of the committed net. */
    int terminals;
    double length;
    double airline;
    /** Realized length / airline, or -1 when the caller has no airline. */
    double detour = -1;
    /** Airline / board diagonal. */
    double diagonal_ratio = -1;
    int rooms;
    /** Narrowest interior room capacity (min dimension / 6x compensated width). */
    double min_capacity = -1;
    /** Mean interior room capacity. */
    double mean_capacity = -1;
    /**
     * How many OTHER dry routes of the negotiation round shared this route's interior rooms
     * (sum over interior rooms of usage-1); -1 outside the negotiation commit phase.
     */
    int contention = -1;
    /** Highest dry-route usage of any interior room; -1 outside the negotiation. */
    int max_usage = -1;

    @Override
    public String toString() {
      return "net=" + net_name + "(#" + net_no + ")"
          + " completes=" + completes_net
          + " terminals=" + terminals
          + " length=" + Math.round(length)
          + " airline=" + (airline > 0 ? String.valueOf(Math.round(airline)) : "?")
          + " detour=" + (detour > 0 ? String.format("%.2f", detour) : "?")
          + " diagfrac=" + (diagonal_ratio >= 0 ? String.format("%.3f", diagonal_ratio) : "?")
          + " rooms=" + rooms
          + " mincap=" + (min_capacity >= 0 ? String.format("%.2f", min_capacity) : "?")
          + " meancap=" + (mean_capacity >= 0 ? String.format("%.2f", mean_capacity) : "?")
          + " contention=" + contention
          + " maxusage=" + max_usage;
    }
  }

  /**
   * Builds the {@link CommitFeatures} of a just-committed partition plan. Every input is
   * either already in hand (the route, the realized length, the airline) or a single cheap
   * board query, so the instrumentation is affordable on every commit.
   */
  private CommitFeatures commit_features(PartitionRouter.CellRoute p_route,
      AutorouteControl p_ctrl, Set<Item> p_start_set, double p_located_length,
      double p_airline_distance) {
    CommitFeatures f = new CommitFeatures();
    f.net_no = p_ctrl.net_no;
    Net net = board.rules.nets.get(p_ctrl.net_no);
    if (net != null) {
      f.net_name = net.name;
    }
    f.length = p_located_length;
    f.airline = p_airline_distance;
    if (p_airline_distance > 0) {
      f.detour = p_located_length / p_airline_distance;
      IntBox bounds = board.get_bounding_box();
      double diagonal = Math.hypot((double) bounds.ur.x - bounds.ll.x,
          (double) bounds.ur.y - bounds.ll.y);
      if (diagonal > 0) {
        f.diagonal_ratio = p_airline_distance / diagonal;
      }
    }
    for (Item item : board.get_connectable_items(p_ctrl.net_no)) {
      if (item instanceof DrillItem) {
        ++f.terminals;
      }
    }
    f.completes_net = true;
    for (Item item : p_start_set) {
      f.completes_net = item.get_unconnected_set(p_ctrl.net_no).isEmpty();
      break;
    }
    if (p_route != null && p_route.rooms != null) {
      f.rooms = p_route.rooms.size();
      int half_width = Math.max(1, p_ctrl.compensated_trace_half_width[p_route.layer]);
      double capacity_sum = 0;
      int interior = 0;
      int contention = 0;
      int max_usage = 0;
      for (int i = 1; i + 1 < p_route.rooms.size(); i++) {
        IntBox box = p_route.rooms.get(i).box;
        int min_dim = Math.min(box.ur.x - box.ll.x, box.ur.y - box.ll.y);
        double capacity = min_dim / (6.0 * half_width);
        capacity_sum += capacity;
        if (f.min_capacity < 0 || capacity < f.min_capacity) {
          f.min_capacity = capacity;
        }
        ++interior;
        if (commit_usage != null) {
          int usage = commit_usage.getOrDefault(PartitionRouter.box_key(box), 0);
          contention += Math.max(0, usage - 1);
          max_usage = Math.max(max_usage, usage);
        }
      }
      if (interior > 0) {
        f.mean_capacity = capacity_sum / interior;
      }
      if (commit_usage != null) {
        f.contention = contention;
        f.max_usage = max_usage;
      }
    }
    return f;
  }

  /**
   * The commit-acceptance predicate (featureFlags.commitPolicy). The shipped predicate is
   * {@code capacity} -- corridor slack: accept a commit only when every interior room of its
   * route still has capacity for another trace of the same width. {@code -Dfr.commitpolicy.mode}
   * selects a different predicate for measurement runs ({@code completes}: the banked own-net
   * completion signature, MEASURED AND REFUTED -- see docs/dense-bga-roadmap.md; {@code slack}:
   * both; {@code contention}: completion plus an uncontended negotiation corridor); it is a
   * diagnostic override, not a supported setting.
   */
  static boolean commit_policy_accepts(CommitFeatures p_features, String p_mode) {
    return switch (p_mode) {
      case "completes" -> p_features.completes_net;
      case "contention" -> p_features.completes_net && p_features.contention <= 0;
      case "slack" -> p_features.completes_net && p_features.min_capacity >= MIN_COMMIT_CAPACITY;
      default -> p_features.min_capacity >= MIN_COMMIT_CAPACITY;
    };
  }

  private boolean commit_policy_accepts(CommitFeatures p_features) {
    return commit_policy_accepts(p_features, COMMIT_POLICY_MODE);
  }

  /**
   * The corridor-slack threshold: an interior room admits a second trace of the committed
   * width when its narrow dimension exceeds six compensated half widths (the negotiation's
   * own room capacity formula), so {@code min_capacity >= 1} means every room the commit
   * passes through still has room for someone else.
   */
  static final double MIN_COMMIT_CAPACITY = 1.0;

  private static final String COMMIT_POLICY_MODE =
      System.getProperty("fr.commitpolicy.mode", "capacity");

  private boolean isCommitPolicyEnabled() {
    return app.freerouting.Freerouting.globalSettings != null
        && app.freerouting.Freerouting.globalSettings.featureFlags.commitPolicy;
  }

  private long partition_policy_decline_count;

  // ---------------------------------------------------------------------------------------
  // Trial-commit lookahead (featureFlags.commitLookahead, -Dfr.lookahead)
  //
  // The commit-acceptance increment REFUTED every commit-local predicate and concluded that a
  // commit's value is a property of the TRAJECTORY it starts, not of the commit. So this does
  // not predict; it measures. At a candidate commit the same horizon is routed twice from the
  // same board state -- once with the candidate declined, once with it committed -- and the
  // candidate is kept only if its horizon ends with no more incomplete connections.
  //
  // Trial ORDER is load-bearing: the decline probe runs BEFORE the commit and the accept probe
  // runs INSIDE the commit's own snapshot, so a kept commit is never rolled back and re-applied
  // (re-committing a plan onto a board restored by undo would hand InsertFoundConnectionAlgo
  // start/target Item references that undo may have replaced with their restored copies).
  // ---------------------------------------------------------------------------------------

  /** 0 = the whole horizon list; otherwise the probe stops after K (item, net) attempts. */
  private static final int LOOKAHEAD_K =
      Integer.getInteger("fr.lookahead.k", 0);
  /**
   * How many passes each probe routes; 1 = finish the current pass only. Defaults to the
   * router's own pass budget, i.e. the probe simulates the whole REMAINING RUN, because
   * shorter horizons were measured to pick badly: on bm01 a one-pass horizon is a precise
   * measurement of the wrong thing (958.95/8, worse than not looking ahead at all), while the
   * full-run horizon holds the champion (994.85/1).
   */
  private static final int LOOKAHEAD_PASSES =
      Integer.getInteger("fr.lookahead.passes", 8);
  /**
   * What to do when both trials end with the same incomplete count: "accept" (default) leaves
   * the underlying configuration's behaviour alone wherever the horizon sees no difference,
   * "decline" suppresses every commit the horizon cannot show a gain for.
   */
  private static final String LOOKAHEAD_TIE =
      System.getProperty("fr.lookahead.tie", "accept");
  /**
   * How many incomplete connections the accept horizon may be WORSE by and still be kept.
   *
   * <p>0 is the untuned rule -- keep only what the horizon does not show losing -- and it is
   * measurably too strict, because each candidate is probed against a continuation in which NO
   * later candidate commits, so a set of commits that only pays TOGETHER is killed at its first
   * member. bm04 shows this exactly: probed alone the three candidates score 5, 5 and 2 against
   * a decline-all continuation of 3, so a zero margin declines two of them and the board falls
   * to 972.02/4 -- below flag-off -- while keeping all three gives 986.01/2, and with the
   * margin the SECOND probe (measured from the board that already carries the first commit)
   * flips to accept=3 vs decline=5.
   *
   * <p><b>2 is a constant fitted to four boards</b> (it is the largest single-commit loss the
   * measured complementary chains show) and its generalization is unmeasured. It is the value
   * at which all four gate boards meet their targets; at 0, three of four do.
   */
  private static final int LOOKAHEAD_MARGIN =
      Integer.getInteger("fr.lookahead.margin", 2);
  /** Measurement only: how many times each decline probe is repeated from the same board. */
  private static final int LOOKAHEAD_REPEAT =
      Integer.getInteger("fr.lookahead.repeat", 1);
  /**
   * The pull-tight budget (ms, 0 = unlimited) used INSIDE a probe. The router's own budget is
   * a 1000 ms WALL-CLOCK limit, and the campaign already traced run-to-run geometry variance
   * to it; inside a probe that variance becomes measurement noise, so this exists to test
   * whether removing the deadline makes the horizon reproducible.
   */
  private static final int LOOKAHEAD_PULLTIGHT =
      Integer.getInteger("fr.lookahead.pulltight", TIME_LIMIT_TO_PREVENT_ENDLESS_LOOP);

  /**
   * The pull-tight deadline for the code running right now: the router's own constant, or the
   * probe's budget while a lookahead probe is routing. Identical to the constant whenever the
   * lookahead flag is off.
   */
  private int pull_tight_limit() {
    return this.in_lookahead ? LOOKAHEAD_PULLTIGHT : TIME_LIMIT_TO_PREVENT_ENDLESS_LOOP;
  }

  private boolean isCommitLookaheadEnabled() {
    return app.freerouting.Freerouting.globalSettings != null
        && app.freerouting.Freerouting.globalSettings.featureFlags.commitLookahead;
  }

  /** True while a lookahead probe is routing, which disables nested partition attempts. */
  private boolean in_lookahead;
  /**
   * Set once a routing pass is under way, which is the precondition for probing: a probe
   * simulates the continuation of a pass, so there has to be one.
   */
  private boolean lookahead_pass_active;
  /** The pass number the probes route as. */
  private int lookahead_pass_no = 1;
  private long lookahead_probe_count;
  private long lookahead_accept_count;
  private long lookahead_decline_count;
  private long lookahead_tie_count;
  private long lookahead_ms;

  /**
   * Routes one bounded horizon from a THROWAWAY CLONE of the board and returns the number of
   * incomplete connections the horizon ended with (lower is better). The live board is not
   * touched at all.
   *
   * <p>The clone, not a snapshot/undo bracket, is the load-bearing choice, and it was forced by
   * measurement. An undo-based probe restores every last piece of copper -- the copper digest
   * proves it -- and STILL leaves the next probe of the identical board measuring something
   * else: repeating one probe three times gave 47/54/49, 52/48/50, 61/50/48. Rewinding the item
   * id counter changed nothing, and removing the wall-clock pull-tight deadline changed nothing,
   * so the residue is the board's own rebuilt-in-a-different-order internal structure (the
   * search trees are rebuilt item by item as undo restores them). {@code BasicBoard.readObject}
   * rebuilds the search trees from the canonical item order instead, so two clones of the same
   * bytes are the same board in every respect a search can observe.
   *
   * <p>The metric is CONNECTIVITY, not geometry: the campaign traced run-to-run geometry
   * variance to the 1000 ms wall-clock pull-tight limit, and an incomplete count cannot see
   * where pull-tight stopped.
   *
   * <p>Nested partition/negotiation commits are disabled inside the probe ({@link
   * #in_lookahead}), so the horizon is routed by the classic engine only: a probe cannot
   * recurse, and its cost stays bounded by the pass it simulates. That is also the mechanism's
   * main fidelity gap -- the real continuation keeps making partition commits, the probed one
   * does not.
   */
  private int lookahead_probe() {
    long t0 = System.currentTimeMillis();
    ++lookahead_probe_count;
    RoutingBoard live_board = this.board;
    boolean saved_in_lookahead = this.in_lookahead;
    RoutingBoard probe_board = (RoutingBoard) BasicBoard.deserialize(live_board.serialize(false));
    if (probe_board == null) {
      FRLogger.error("Lookahead probe could not clone the board; treating the horizon as unmeasured", null);
      return Integer.MAX_VALUE;
    }
    this.board = probe_board;
    this.in_lookahead = true;
    try {
      int attempts = 0;
      // The router is scored on the BEST board it reaches, not the last one (the end-of-run
      // best-board restore), so a multi-pass horizon is scored the same way.
      int best = Integer.MAX_VALUE;
      for (int horizon_pass = 0; horizon_pass < Math.max(1, LOOKAHEAD_PASSES); horizon_pass++) {
        // Rebuilt from the clone every pass: the live board's Item objects do not exist on it.
        // This is also why the horizon is "every connection still incomplete" rather than "the
        // rest of the pass list from the cursor" -- on the clone the two coincide for the
        // negotiation commit phase, which runs before the pass routes anything.
        List<Item> horizon = getAutorouteItems(this.board, new ArrayList<>());
        if (horizon.isEmpty()) {
          break;
        }
        for (Item curr_item : horizon) {
          for (int net_index = 0; net_index < curr_item.net_count(); net_index++) {
            if (this.thread.is_stop_auto_router_requested()
                || (LOOKAHEAD_K > 0 && attempts >= LOOKAHEAD_K)) {
              return Math.min(best, calculateIncompleteCount(this.board));
            }
            ++attempts;
            this.board.start_marking_changed_area();
            autoroute_item(curr_item, curr_item.get_net_no(net_index), new TreeSet<>(),
                new LinkedHashMap<>(), this.lookahead_pass_no + horizon_pass);
          }
        }
        remove_tails(this.remove_unconnected_vias
            ? Item.StopConnectionOption.NONE : Item.StopConnectionOption.FANOUT_VIA);
        best = Math.min(best, calculateIncompleteCount(this.board));
      }
      return best;
    } catch (Exception e) {
      FRLogger.error("Lookahead probe failed; treating the horizon as unmeasured", e);
      return Integer.MAX_VALUE;
    } finally {
      this.board = live_board;
      this.in_lookahead = saved_in_lookahead;
      lookahead_ms += System.currentTimeMillis() - t0;
    }
  }

  /**
   * The lookahead counters, or the empty string when the flag is off (so flag-off log output
   * stays byte-identical).
   */
  private String lookahead_report() {
    if (!isCommitLookaheadEnabled()) {
      return "";
    }
    return " lookahead_probes=" + lookahead_probe_count
        + " lookahead_accepts=" + lookahead_accept_count
        + " lookahead_declines=" + lookahead_decline_count
        + " lookahead_ties=" + lookahead_tie_count
        + " lookahead_ms=" + lookahead_ms;
  }

  /**
   * The lookahead's decision rule: keep a candidate whose accept horizon ends with at most
   * {@code p_decline_incompletes + p_margin} incomplete connections.
   *
   * <p>Exactly on the threshold the candidate is kept unless {@code p_tie} is "decline", which
   * is what leaves the underlying configuration's behaviour alone wherever the horizon sees no
   * difference at all. Both horizons are incomplete COUNTS, so lower is better and the rule is
   * a plain comparison -- there is no scoring function to tune, only the margin.
   */
  static boolean lookahead_keeps(int p_accept_incompletes, int p_decline_incompletes,
      int p_margin, String p_tie) {
    // A probe that failed reports Integer.MAX_VALUE. An unmeasured horizon is not evidence,
    // so the candidate goes to the classic engine rather than being kept on a guess -- and the
    // threshold is computed in long so the sentinel cannot wrap negative and read as an accept.
    if (p_accept_incompletes == Integer.MAX_VALUE || p_decline_incompletes == Integer.MAX_VALUE) {
      return false;
    }
    long threshold = (long) p_decline_incompletes + p_margin;
    if (p_accept_incompletes == threshold) {
      return !"decline".equals(p_tie);
    }
    return p_accept_incompletes < threshold;
  }

  /**
   * A canonical digest of the board's COPPER -- every trace and via as (net, layer, corners),
   * sorted, so the digest is independent of container iteration order.
   *
   * <p>Deliberately not {@code BasicBoard.get_hash()}: that serializes {@code item_list}
   * itself, so it also digests the undo stack and therefore always differs after a probe even
   * when the probe restored every last piece of copper. This is the predicate the probe's
   * rollback actually has to satisfy.
   */
  private String copper_digest() {
    List<String> lines = new ArrayList<>();
    for (Trace trace : board.get_traces()) {
      StringBuilder line = new StringBuilder("T");
      for (int i = 0; i < trace.net_count(); i++) {
        line.append(':').append(trace.get_net_no(i));
      }
      line.append('@').append(trace.get_layer());
      if (trace instanceof PolylineTrace polyline_trace) {
        for (int i = 0; i < polyline_trace.corner_count(); i++) {
          line.append('|').append(polyline_trace.polyline().corner_approx(i));
        }
      }
      lines.add(line.toString());
    }
    for (Via via : board.get_vias()) {
      StringBuilder line = new StringBuilder("V");
      for (int i = 0; i < via.net_count(); i++) {
        line.append(':').append(via.get_net_no(i));
      }
      line.append('@').append(via.get_center());
      lines.add(line.toString());
    }
    java.util.Collections.sort(lines);
    return lines.size() + "/" + Integer.toHexString(String.join(";", lines).hashCode());
  }


  /**
   * The negotiation round's dry-route room usage, published for the duration of the commit
   * phase so {@link #commit_features} can price corridor contention; null everywhere else
   * (the per-item quality-mode path has no dry-route round).
   */
  private Map<String, Integer> commit_usage;

  /**
   * Partition-based single-layer router (docs/free-space-partition.md stage 2), created on
   * first use when the feature flag is on. Null otherwise.
   */
  private PartitionRouter partition_router;

  private boolean isPartitionRouterEnabled() {
    return app.freerouting.Freerouting.globalSettings != null
        && app.freerouting.Freerouting.globalSettings.featureFlags.partitionRouter;
  }

  /**
   * Creates the per-connection {@link TimeLimit} for one routing attempt. The base limit is the
   * historical escalating schedule (100 s, doubled on every ripup pass). When the
   * {@code router.max_milliseconds_per_item} budget is set ({@code > 0}), it caps that schedule
   * so a single connection can never consume more than the configured budget. A budget of 0
   * (the default) leaves the schedule unchanged.
   */
  private TimeLimit create_connection_time_limit(int p_ripup_pass_no) {
    double max_milliseconds = 100000 * Math.pow(2, p_ripup_pass_no - 1);
    max_milliseconds = Math.min(max_milliseconds, Integer.MAX_VALUE);
    int per_item_budget = this.settings.getMaxMillisecondsPerItem();
    if (per_item_budget > 0) {
      max_milliseconds = Math.min(max_milliseconds, per_item_budget);
    }
    return new TimeLimit((int) max_milliseconds);
  }

  /**
   * Logs one INFO line when a failed connection attempt ran out of its per-item time budget
   * ({@code router.max_milliseconds_per_item}). Silent when the budget is unlimited (0) so
   * default runs keep today's log output; the abandoned connection is simply counted as
   * unrouted for this pass.
   */
  private void log_per_item_budget_exceeded(TimeLimit p_time_limit, int p_route_net_no) {
    int per_item_budget = this.settings.getMaxMillisecondsPerItem();
    if (per_item_budget <= 0 || p_time_limit == null || !p_time_limit.limit_exceeded()) {
      return;
    }
    Net route_net = board.rules.nets.get(p_route_net_no);
    FRLogger.info("Autoroute time budget of " + per_item_budget + " ms exceeded for net '"
        + (route_net != null ? route_net.name : "#" + p_route_net_no)
        + "'; the connection is left unrouted in this pass.");
  }

  private boolean isParallelAutoroutingEnabled() {
    return app.freerouting.Freerouting.globalSettings != null
        // Opt-in and OFF by default -- see FeatureFlagsSettings#parallelAutorouter for the
        // known commit-time race that has to be closed before this can be enabled generally.
        && app.freerouting.Freerouting.globalSettings.featureFlags.parallelAutorouter
        && app.freerouting.Freerouting.globalSettings.featureFlags.multiThreading
        && this.settings.maxThreads != null
        && this.settings.maxThreads > 1;
  }

  private boolean autoroute_pass(int p_pass_no) {
    if (isParallelAutoroutingEnabled()) {
      return autoroute_pass_parallel(p_pass_no);
    }
    long passStartTime = System.currentTimeMillis();
    try {
      List<Item> autoroute_item_list = getAutorouteItems(this.board);

      // If there are no items to route, we're done
      if (autoroute_item_list.isEmpty()) {
        this.air_line = null;
        return false;
      }

      // The trial-commit lookahead probes the continuation of THIS pass.
      if (isCommitLookaheadEnabled()) {
        this.lookahead_pass_active = true;
        this.lookahead_pass_no = p_pass_no;
      }

      if (isNegotiatedRouterEnabled() && p_pass_no == 1) {
        run_negotiation(autoroute_item_list);
      }
      int items_to_go_count = autoroute_item_list.size();
      int ripped_item_count = 0;
      int not_routed = 0;
      int routed = 0;
      int skipped = 0;
      // One incompletes analysis per pass start, shared between the statistics object and the
      // counters. get_statistics() used to run its own identical DesignRulesChecker internally
      // and discard it, so the same full-board scan (item scan + per-net Delaunay) ran twice.
      DesignRulesChecker tempDrc = new DesignRulesChecker(board, null);
      tempDrc.calculateAllIncompletes();
      BoardStatistics stats = new BoardStatistics(board, null, true, false);
      stats.connections.maximumCount = tempDrc.max_connections;
      stats.connections.incompleteCount = tempDrc.getIncompleteCount();
      RouterCounters routerCounters = new RouterCounters();
      routerCounters.phase = "autoroute";
      routerCounters.passCount = p_pass_no;
      routerCounters.queuedToBeRoutedCount = items_to_go_count;
      routerCounters.skippedCount = skipped;
      routerCounters.rippedCount = ripped_item_count;
      routerCounters.failedToBeRoutedCount = not_routed;
      routerCounters.routedCount = routed;
      routerCounters.incompleteCount = tempDrc.getIncompleteCount();

      // Log incomplete details for debugging
      if (routerCounters.incompleteCount > 0) {
        job.logDebug("Pass #" + p_pass_no + ": " + routerCounters.incompleteCount + " incompletes across "
            + items_to_go_count + " items to route");
        for (int netNo = 1; netNo <= board.rules.nets.max_net_no(); netNo++) {
          int netIncompletes = tempDrc.getIncompleteCount(netNo);
          if (netIncompletes > 0) {
            Net net = board.rules.nets.get(netNo);
            String netName = (net != null) ? net.name : "net#" + netNo;
            job.logDebug("  Net '" + netName + "' has " + netIncompletes + " incomplete(s)");
          }
        }
      }

      this.fireBoardUpdatedEvent(stats, routerCounters, this.board);

      // Sort items by airline distance (shortest first) for deterministic routing
      // This prioritizes local connections which typically route faster
      // NOTE: Disabled in v2.3 because it negatively impacts convergence compared to
      // v1.9 (natural order)
      // autoroute_item_list.sort(Comparator.comparingDouble(this::calculateItemDistance));

      // Let's go through all items to route
      for (Item curr_item : autoroute_item_list) {
        // If the user requested to stop the auto-router, we stop it
        if (this.thread.is_stop_auto_router_requested()) {
          break;
        }

        // Let's go through all nets of this item
        for (int i = 0; i < curr_item.net_count(); i++) {
          // If the user requested to stop the auto-router, we stop it
          if (this.thread.is_stop_auto_router_requested()) {
            break;
          }

          if (this.settings.maxItems != null && this.settings.maxItems > 0 && this.totalItemsRouted >= this.settings.maxItems) {
            job.logInfo("Max items limit reached (" + this.settings.maxItems + "). Stopping auto-router.");
            // Call requestStop() (sets ALL) instead of request_stop_auto_router() (sets
            // AUTO_ROUTER_ONLY) so the optimization stage is also skipped.  maxItems is a
            // debugging/test ceiling meant to bound the entire routing job; running the
            // optimizer on a deliberately-incomplete board is not useful and prevents the
            // process from terminating promptly.
            this.thread.requestStop();
            break;
          }
          this.totalItemsRouted++;

          // We visually mark the area of the board, which is changed by the auto-router
          board.start_marking_changed_area();

          // Do the auto-routing step for this item (typically PolylineTrace or Pin)
          // Use a fresh set per item to mirror v1.9 behavior and avoid cross-item side effects.
          SortedSet<Item> ripped_item_list = new TreeSet<>();
          Map<Item, Integer> ripped_item_costs = new LinkedHashMap<>();
          // Full O(N) board scan whose only consumer is the trace-log block below, so it is
          // only worth paying for when trace logging is actually on.
          int netItemsBefore = FRLogger.isTraceEnabled()
              ? board.get_connectable_items(curr_item.get_net_no(i)).size()
              : 0;
          PerformanceProfiler.start("autoroute_item");
          var autorouterResult = autoroute_item(curr_item, curr_item.get_net_no(i), ripped_item_list, ripped_item_costs, p_pass_no);
          PerformanceProfiler.end("autoroute_item");
          if (!ripped_item_list.isEmpty()) {
            for (Item rippedItem : ripped_item_list) {
              StringBuilder rippedNets = new StringBuilder();
              for (int netIx = 0; netIx < rippedItem.net_count(); netIx++) {
                if (netIx > 0) {
                  rippedNets.append('|');
                }
                rippedNets.append(rippedItem.get_net_no(netIx));
              }
              int ripupCost = ripped_item_costs.getOrDefault(rippedItem, -1);
              FRLogger.trace(
                  "BatchAutorouter.autoroute_pass",
                  "compare_trace_ripped_item",
                  "source_item=" + curr_item.get_id_no()
                      + ", source_net=" + curr_item.get_net_no(i)
                      + ", ripped_id=" + rippedItem.get_id_no()
                      + ", ripped_type=" + rippedItem.getClass().getSimpleName()
                      + ", ripped_net_count=" + rippedItem.net_count()
                      + ", ripped_nets=" + rippedNets
                      + ", ripup_cost=" + ripupCost,
                  "Net #" + curr_item.get_net_no(i) + ",Item #" + curr_item.get_id_no(),
                  getImpactedPoints(rippedItem));
            }
          }
          if (FRLogger.isTraceEnabled()) {
            DesignRulesChecker innerDrc = new DesignRulesChecker(board, null);
            innerDrc.calculateAllIncompletes();
            int tempIncomp = innerDrc.getIncompleteCount();
            int tempNetIncomp = innerDrc.getIncompleteCount(curr_item.get_net_no(i));
            int netItemsAfter = board.get_connectable_items(curr_item.get_net_no(i)).size();
            int maxItemId = board.communication.id_no_generator.max_generated_no();
            FRLogger.trace(
                "BatchAutorouter.autoroute_pass",
                "compare_trace_route_item",
                "Routing " + curr_item.getClass().getSimpleName() + " -> result=" + autorouterResult.state
                    + ", details=" + autorouterResult.details
                    + ", incompletes=" + tempIncomp + ", netIncomplete=" + tempNetIncomp
                    + ", ripped=" + ripped_item_list.size() + ", netItems="
                    + netItemsBefore + "->" + netItemsAfter
                    + ", maxItemId=" + maxItemId,
                "Net #" + curr_item.get_net_no(i) + ",Item #" + curr_item.get_id_no() + ",Type="
                    + curr_item.getClass().getSimpleName(),
                getImpactedPoints(curr_item));
          }

          // Everything in this block only emits trace output, but it walks all of the net's
          // items to do so -- skip the walk entirely when nothing will be logged.
          if (FRLogger.isTraceEnabled() && curr_item.get_net_no(i) == 94) {
            FRLogger.trace(
                "BatchAutorouter.autoroute_pass",
                "compare_trace_dump_net_items",
                "Dump net 94 items",
                "Net #94",
                new Point[0]);
            for (Item nItem : board.get_connectable_items(94)) {
              if (nItem instanceof Trace) {
                Trace t = (Trace) nItem;
                FRLogger.trace(
                    "BatchAutorouter.autoroute_pass",
                    "compare_trace_dump_net_item",
                    "Trace layer=" + t.get_layer() + " corners=" + t.first_corner() + " to " + t.last_corner(),
                    "Net #94,Item #" + t.get_id_no() + ",Type=Trace",
                    new Point[] { t.first_corner(), t.last_corner() });
              } else if (nItem instanceof Via) {
                Via v = (Via) nItem;
                FRLogger.trace(
                    "BatchAutorouter.autoroute_pass",
                    "compare_trace_dump_net_item",
                    "Via center=" + v.get_center(),
                    "Net #94,Item #" + v.get_id_no() + ",Type=Via",
                    new Point[] { v.get_center() });
              } else if (nItem instanceof Pin) {
                Pin p = (Pin) nItem;
                FRLogger.trace(
                    "BatchAutorouter.autoroute_pass",
                    "compare_trace_dump_net_item",
                    "Pin center=" + p.get_center() + " name=" + p.name() + " comp=" + p.component_name(),
                    "Net #94,Item #" + p.get_id_no() + ",Type=Pin",
                    new Point[] { p.get_center() });
              } else {
                FRLogger.trace(
                    "BatchAutorouter.autoroute_pass",
                    "compare_trace_dump_net_item",
                    "Item " + nItem.getClass().getSimpleName(),
                    "Net #94,Item #" + nItem.get_id_no() + ",Type=" + nItem.getClass().getSimpleName(),
                    getImpactedPoints(nItem));
              }
            }
          }

          if (autorouterResult.state == AutorouteAttemptState.ROUTED) {
            // The item was successfully routed
            ++routed;
          } else if ((autorouterResult.state == AutorouteAttemptState.ALREADY_CONNECTED)
              || (autorouterResult.state == AutorouteAttemptState.NO_UNCONNECTED_NETS)
              || (autorouterResult.state == AutorouteAttemptState.CONNECTED_TO_PLANE)) {
            // The item doesn't need to be routed
            ++skipped;
          } else {
            Net net = board.rules.nets.get(curr_item.get_net_no(i));
            String netName = (net != null) ? net.name : "net#" + curr_item.get_net_no(i);

            // Record the failure
            board.failureLog.recordFailure(curr_item, p_pass_no, autorouterResult.state, autorouterResult.details);

            job.logDebug("Autorouter " + autorouterResult.details);
            // Log details when we're down to last few items or item has many failures
            int failureCount = board.failureLog.getFailureCount(curr_item);
            if (items_to_go_count <= 5 || failureCount >= 3) {
              job.logDebug("Pass #" + p_pass_no + ": Failed to route " + curr_item.getClass().getSimpleName()
                  + " on net '" + netName + "' (" + items_to_go_count + " items remaining, "
                  + failureCount + " failures). State: " + autorouterResult.state);
            }
            ++not_routed;
          }
          --items_to_go_count;
          ripped_item_count += ripped_item_list.size();

          if (shouldFireBoardUpdate()) {
            BoardStatistics boardStatistics = board.get_statistics();
            routerCounters.passCount = p_pass_no;
            routerCounters.queuedToBeRoutedCount = items_to_go_count;
            routerCounters.skippedCount = skipped;
            routerCounters.rippedCount = ripped_item_count;
            routerCounters.failedToBeRoutedCount = not_routed;
            routerCounters.routedCount = routed;
            // get_statistics() already ran the identical incompletes analysis; recomputing it
            // with calculateIncompleteCount() doubled the cost of every progress event.
            routerCounters.incompleteCount = boardStatistics.connections.incompleteCount;
            this.fireBoardUpdatedEvent(boardStatistics, routerCounters, this.board);
          }
        }
      }

      // These two counts feed trace logging only, and each is a full-board incompletes
      // analysis -- skip both when trace logging is off.
      if (FRLogger.isTraceEnabled()) {
        int incompletesBefore = calculateIncompleteCount(board);
        FRLogger.trace(
            "BatchAutorouter.autoroute_pass",
            "compare_trace_remove_tails",
            "Incompletes before remove_tails=" + incompletesBefore,
            "Autorouter pass #" + p_pass_no,
            new Point[0]);
      }

      if (this.remove_unconnected_vias) {
        remove_tails(Item.StopConnectionOption.NONE);
      } else {
        remove_tails(Item.StopConnectionOption.FANOUT_VIA);
      }

      if (FRLogger.isTraceEnabled()) {
        int incompletesAfter = calculateIncompleteCount(board);
        FRLogger.trace(
            "BatchAutorouter.autoroute_pass",
            "compare_trace_remove_tails",
            "Incompletes after remove_tails=" + incompletesAfter,
            "Autorouter pass #" + p_pass_no,
            new Point[0]);
      }

      // Fire final update for this pass
      BoardStatistics boardStatistics = board.get_statistics();
      routerCounters.passCount = p_pass_no;
      routerCounters.queuedToBeRoutedCount = items_to_go_count;
      routerCounters.skippedCount = skipped;
      routerCounters.rippedCount = ripped_item_count;
      routerCounters.failedToBeRoutedCount = not_routed;
      routerCounters.routedCount = routed;
      routerCounters.incompleteCount = boardStatistics.connections.incompleteCount;
      this.fireBoardUpdatedEvent(boardStatistics, routerCounters, this.board);

      long passDuration = System.currentTimeMillis() - passStartTime;
      int currentRipupCost = this.start_ripup_costs * p_pass_no;
      PerformanceProfiler.recordPass(p_pass_no, routerCounters.incompleteCount, passDuration, currentRipupCost);


      if (isPartitionRouterEnabled()) {
        job.logInfo("[partition-router] pass=" + p_pass_no + " routed=" + partition_routed_count
            + " fallback=" + partition_fallback_count
            + " drc_reject=" + partition_drc_reject_count
            + live_channel_report()
            + lookahead_report());
        partition_routed_count = 0;
        partition_fallback_count = 0;
        partition_drc_reject_count = 0;
        if (partition_router != null) {
          partition_router.reset_live_channel_counters();
        }
      }

      // We are done with this pass
      this.air_line = null;
      return routed > 0 || not_routed > 0;
    } catch (Exception e) {
      job.logError("Something went wrong during the auto-routing", e);
      this.air_line = null;
      return false;
    }
  }

  /**
   * One (item, net) unit of work for the parallel pass -- the flattened equivalent of one
   * iteration of the sequential pass's {@code for (Item curr_item ...) for (int i ...)} loop.
   */
  private record ItemNetWorkUnit(Item item, int net_no) {
  }

  /**
   * Parallel counterpart of {@link #autoroute_pass}: routes the items of this pass across a
   * pool of {@code settings.maxThreads} worker threads instead of one at a time.
   *
   * <p>Each worker searches against its own private, unshared search-tree snapshot (see
   * {@link SearchTreeManager#build_scratch_search_tree}) built once per worker at the start of
   * the pass -- so the CPU-heavy maze-search wavefront expansion runs fully lock-free and in
   * parallel. Only the short commit step (rip-up removal + trace/via insertion, plus the
   * subsequent pull-tight optimization) touches the real, shared board, and that is serialized
   * under {@link SearchTreeManager}'s write lock with a re-validation check (see
   * {@link AutorouteEngine#autoroute_connection(Set, Set, AutorouteControl, SortedSet, Map, SearchTreeManager)}):
   * if another worker already ripped up an item this attempt depended on, the attempt is
   * abandoned as a conflict and the item is left for a later pass, exactly like any other
   * routing failure.
   *
   * <p>Because each worker's private tree is a point-in-time snapshot taken once at the start of
   * the pass, it grows increasingly stale relative to the real board as the pass progresses --
   * later items routed by a given worker are more likely to hit a conflict (and be deferred) than
   * they would be sequentially. This trades a somewhat higher same-pass failure/retry rate for
   * avoiding the cost of rebuilding a worker's tree before every single item; the normal
   * escalating-ripup-cost multi-pass loop absorbs the deferred items on the next pass, the same
   * way it already absorbs any other routing failure today.
   */
  private boolean autoroute_pass_parallel(int p_pass_no) {
    long passStartTime = System.currentTimeMillis();
    try {
      List<Item> autoroute_item_list = getAutorouteItems(this.board);

      if (autoroute_item_list.isEmpty()) {
        this.air_line = null;
        return false;
      }

      List<ItemNetWorkUnit> work_units = new ArrayList<>();
      for (Item curr_item : autoroute_item_list) {
        for (int i = 0; i < curr_item.net_count(); i++) {
          work_units.add(new ItemNetWorkUnit(curr_item, curr_item.get_net_no(i)));
        }
      }

      BoardStatistics stats = board.get_statistics();
      RouterCounters routerCounters = new RouterCounters();
      routerCounters.phase = "autoroute";
      routerCounters.passCount = p_pass_no;
      routerCounters.queuedToBeRoutedCount = work_units.size();
      DesignRulesChecker tempDrc = new DesignRulesChecker(board, null);
      tempDrc.calculateAllIncompletes();
      routerCounters.incompleteCount = tempDrc.getIncompleteCount();
      this.fireBoardUpdatedEvent(stats, routerCounters, this.board);

      int threadCount = Math.max(1,
          Math.min(this.settings.maxThreads, Runtime.getRuntime().availableProcessors()));
      // Daemon threads: a worker stuck in a pathologically long maze search (bounded by the
      // same per-connection time_limit the sequential path already uses) must never keep the
      // whole JVM alive on its own.
      ExecutorService pool = Executors.newFixedThreadPool(threadCount, r -> {
        Thread t = new Thread(r, "parallel-autoroute-worker");
        t.setDaemon(true);
        return t;
      });

      AtomicInteger routed = new AtomicInteger();
      AtomicInteger notRouted = new AtomicInteger();
      AtomicInteger skipped = new AtomicInteger();
      AtomicInteger rippedItemCount = new AtomicInteger();
      // Mirrors the sequential pass's per-unit "this.totalItemsRouted++" (counts every attempt,
      // not just successes) so the maxItems debug ceiling means the same thing in both paths.
      AtomicInteger attempted = new AtomicInteger();

      // One private scratch tree per worker THREAD, lazily built and cached the first time that
      // thread encounters a given clearance class, then reused for the rest of this pass -- not
      // rebuilt per item. See the class javadoc above for the staleness trade-off this implies.
      ThreadLocal<Map<Integer, ShapeSearchTree>> workerScratchTrees = ThreadLocal.withInitial(HashMap::new);

      // Bulk-synchronous batches: every worker searches, then a barrier, then this thread alone
      // commits. Nothing mutates the board while searches are in flight, which is what makes
      // the searches safe -- they read live Item geometry (trace shape counts, contacts) well
      // beyond the search tree, so a concurrent commit would otherwise tear that state out from
      // under them. Batching (rather than one barrier per pass) keeps each plan's view of the
      // board at most one batch stale, which keeps the conflict rate low.
      // One batch per worker: a sweep over 1x/2x/4x/8x the thread count showed no measurable
      // difference in wall time, so the smallest is used -- it keeps each plan's view of the
      // board the least stale, which is what the commit-time re-validation has to cope with.
      int batchSize = threadCount;
      for (int batch_start = 0; batch_start < work_units.size(); batch_start += batchSize) {
        if (this.thread.is_stop_auto_router_requested()) {
          break;
        }
        if (this.settings.maxItems != null && this.settings.maxItems > 0
            && (this.totalItemsRouted + attempted.get()) >= this.settings.maxItems) {
          this.thread.requestStop();
          break;
        }
        int batch_end = Math.min(batch_start + batchSize, work_units.size());
        List<ItemNetWorkUnit> batch = work_units.subList(batch_start, batch_end);

        // ---- phase 1: parallel search, read-only with respect to the board ----
        List<Callable<SearchAttempt>> tasks = new ArrayList<>(batch.size());
        for (ItemNetWorkUnit unit : batch) {
          tasks.add(() -> {
            attempted.incrementAndGet();
            SortedSet<Item> ripped_item_list = new TreeSet<>();
            SearchOutcome outcome = autoroute_item_parallel(unit.item(), unit.net_no(),
                ripped_item_list, p_pass_no, workerScratchTrees.get());
            return new SearchAttempt(unit, outcome, ripped_item_list);
          });
        }
        List<Future<SearchAttempt>> futures = pool.invokeAll(tasks);

        // ---- phase 2: serial commit on this thread only ----
        // A plan was searched against the board as it stood at the start of this batch, so each
        // one is re-validated against the live board immediately before it is written. Plans
        // that lost their space simply fail this pass and are retried on the next, exactly like
        // any other routing failure.
        for (Future<SearchAttempt> future : futures) {
          // Surface worker exceptions instead of silently swallowing them.
          SearchAttempt attempt = future.get();
          AutorouteAttemptResult result = commit_search_attempt(attempt);
          rippedItemCount.addAndGet(attempt.ripped_item_list().size());

          if (result.state == AutorouteAttemptState.ROUTED) {
            routed.incrementAndGet();
          } else if (result.state == AutorouteAttemptState.ALREADY_CONNECTED
              || result.state == AutorouteAttemptState.NO_UNCONNECTED_NETS
              || result.state == AutorouteAttemptState.CONNECTED_TO_PLANE) {
            skipped.incrementAndGet();
          } else {
            board.failureLog.recordFailure(attempt.unit().item(), p_pass_no, result.state, result.details);
            notRouted.incrementAndGet();
          }
        }
      }
      pool.shutdown();

      this.totalItemsRouted += attempted.get();

      int incompletesBefore = calculateIncompleteCount(board);
      FRLogger.trace(
          "BatchAutorouter.autoroute_pass_parallel",
          "compare_trace_remove_tails",
          "Incompletes before remove_tails=" + incompletesBefore,
          "Autorouter pass #" + p_pass_no,
          new Point[0]);

      if (this.remove_unconnected_vias) {
        remove_tails(Item.StopConnectionOption.NONE);
      } else {
        remove_tails(Item.StopConnectionOption.FANOUT_VIA);
      }

      BoardStatistics finalStats = board.get_statistics();
      routerCounters.passCount = p_pass_no;
      routerCounters.queuedToBeRoutedCount = 0;
      routerCounters.skippedCount = skipped.get();
      routerCounters.rippedCount = rippedItemCount.get();
      routerCounters.failedToBeRoutedCount = notRouted.get();
      routerCounters.routedCount = routed.get();
      routerCounters.incompleteCount = calculateIncompleteCount(board);
      this.fireBoardUpdatedEvent(finalStats, routerCounters, this.board);

      long passDuration = System.currentTimeMillis() - passStartTime;
      int currentRipupCost = this.start_ripup_costs * p_pass_no;
      PerformanceProfiler.recordPass(p_pass_no, routerCounters.incompleteCount, passDuration, currentRipupCost);

      this.air_line = null;
      return routed.get() > 0 || notRouted.get() > 0;
    } catch (Exception e) {
      job.logError("Something went wrong during the parallel auto-routing", e);
      this.air_line = null;
      return false;
    }
  }

  /**
   * Parallel-safe counterpart of {@link #autoroute_item}: routes a single (item, net) work unit
   * against a private, worker-owned search tree, committing under {@link SearchTreeManager}'s
   * write lock. Deliberately narrower than {@link #autoroute_item} -- see
   * {@link #autoroute_pass_parallel}'s javadoc for what is intentionally left out.
   */
  private SearchOutcome autoroute_item_parallel(Item p_item, int p_route_net_no,
      SortedSet<Item> p_ripped_item_list, int p_ripup_pass_no, Map<Integer, ShapeSearchTree> p_worker_scratch_trees) {
    try {
      boolean contains_plane = false;
      Net route_net = board.rules.nets.get(p_route_net_no);
      if (route_net != null) {
        contains_plane = route_net.contains_plane();
      }

      int curr_via_costs = contains_plane ? this.settings.get_plane_via_costs() : this.settings.get_via_costs();

      AutorouteControl autoroute_control = new AutorouteControl(this.board, p_route_net_no, settings, curr_via_costs,
          this.trace_cost_arr);
      autoroute_control.ripup_allowed = true;
      autoroute_control.ripup_costs = this.start_ripup_costs * p_ripup_pass_no;
      autoroute_control.remove_unconnected_vias = this.remove_unconnected_vias;

      // Both connectivity queries walk item contacts through the board's REAL shared search
      // tree (Item.get_normal_contacts -> BasicBoard.overlapping_objects -> get_default_tree),
      // NOT through this worker's private scratch tree. They must therefore hold the read lock:
      // without it they can traverse MinAreaTree while another worker's commit is mid-way
      // through structurally relinking nodes under the write lock. Taken once around both calls
      // rather than per call, since they are two halves of one consistent connectivity snapshot.
      Set<Item> unconnected_set;
      Set<Item> connected_set;
      long connectivity_stamp = board.search_tree_manager.acquireReadLock();
      try {
        unconnected_set = p_item.get_unconnected_set(p_route_net_no);
        connected_set = p_item.get_connected_set(p_route_net_no);
      } finally {
        board.search_tree_manager.releaseReadLock(connectivity_stamp);
      }

      if (unconnected_set.isEmpty()) {
        return SearchOutcome.failed(AutorouteAttemptState.NO_UNCONNECTED_NETS);
      }

      if (contains_plane) {
        for (Item curr_item : connected_set) {
          if (curr_item instanceof ConductionArea) {
            return SearchOutcome.failed(AutorouteAttemptState.CONNECTED_TO_PLANE);
          }
        }
      }
      Set<Item> route_start_set = contains_plane ? connected_set : unconnected_set;
      Set<Item> route_dest_set = contains_plane ? unconnected_set : connected_set;

      TimeLimit time_limit = create_connection_time_limit(p_ripup_pass_no);

      ShapeSearchTree scratch_tree = p_worker_scratch_trees.computeIfAbsent(
          autoroute_control.trace_clearance_class_no,
          board.search_tree_manager::build_scratch_search_tree);
      AutorouteEngine autoroute_engine = new AutorouteEngine(this.board, scratch_tree, false);
      autoroute_engine.init_connection(p_route_net_no, this.thread, time_limit);

      // Search only -- no board mutation. The caller commits on a single thread once every
      // worker in this batch has finished searching.
      AutorouteEngine.ConnectionPlan plan = autoroute_engine.search_connection(route_start_set,
          route_dest_set, autoroute_control, p_ripped_item_list, null);
      if (plan != null && plan.failure != null) {
        log_per_item_budget_exceeded(time_limit, p_route_net_no);
      }
      return new SearchOutcome(plan, autoroute_control, autoroute_engine);
    } catch (Exception e) {
      FRLogger.error("Error during parallel routing search", e);
      return new SearchOutcome(null, null, null);
    }
  }

  /**
   * What one worker produced for one work unit during the parallel search phase.
   */
  private record SearchOutcome(AutorouteEngine.ConnectionPlan plan, AutorouteControl ctrl,
      AutorouteEngine engine) {

    static SearchOutcome failed(AutorouteAttemptState p_state) {
      return new SearchOutcome(AutorouteEngine.ConnectionPlan.failed(new AutorouteAttemptResult(p_state)),
          null, null);
    }
  }

  /**
   * One work unit paired with the plan the search phase produced for it.
   */
  private record SearchAttempt(ItemNetWorkUnit unit, SearchOutcome outcome,
      SortedSet<Item> ripped_item_list) {
  }






  /**
   * Commits one searched plan, if it is still valid against the board as it now stands.
   *
   * <p>Runs on a single thread with no worker searching, so it needs no locking. Two staleness
   * checks stand between a plan and the board: its rip-up victims must still exist, and the
   * space its route wants must still be free. The second is delegated to
   * {@code commit_connection}, which tests the located geometry against the live board -- a
   * direct legality check rather than a guess about which regions earlier commits disturbed.
   */
  private AutorouteAttemptResult commit_search_attempt(SearchAttempt p_attempt) {
    SearchOutcome outcome = p_attempt.outcome();
    if (outcome.plan() == null) {
      return new AutorouteAttemptResult(AutorouteAttemptState.FAILED, "The search phase failed.");
    }
    if (outcome.plan().failure != null) {
      return outcome.plan().failure;
    }

    if (!AutorouteEngine.ripped_items_still_present(p_attempt.ripped_item_list())) {
      return new AutorouteAttemptResult(AutorouteAttemptState.FAILED,
          "CONCURRENT_CONFLICT: an item planned for rip-up was already removed by an earlier "
              + "commit in this batch.");
    }

    AutorouteAttemptResult result = outcome.engine()
        .commit_connection(outcome.plan(), outcome.ctrl(), p_attempt.ripped_item_list(), true);

    if (result.state == AutorouteAttemptState.ROUTED) {
      board.start_marking_changed_area();
      board.opt_changed_area(new int[0], null, this.trace_pull_tight_accuracy, outcome.ctrl().trace_costs,
          this.thread, TIME_LIMIT_TO_PREVENT_ENDLESS_LOOP);
    }
    return result;
  }

  @Override
  public String getId() {
    return "freerouting-router";
  }

  @Override
  public String getName() {
    return "Freerouting Auto-router";
  }

  @Override
  public String getVersion() {
    return "1.0";
  }

  @Override
  public String getDescription() {
    return "Freerouting Auto-router v1.0";
  }

  /**
   * Builds a human-readable summary of all unrouted connections on the current board,
   * grouped by net. For each unrouted connection the component and pin names of both
   * endpoints are listed so that the user can identify exactly which connections are
   * missing and address them in their design.
   *
   * <p>Example output:
   * <pre>
   *   Net 'GND' (1 unrouted connection):
   *     - J2-A1  ->  U1-1
   *   Net '/MIPI_CSI_D0_N' (1 unrouted connection):
   *     - J2-A2  ->  U1-2
   * </pre>
   *
   * @return a formatted, multi-line string describing every unrouted airline
   */

  @Override
  public NamedAlgorithmType getType() {
    return NamedAlgorithmType.ROUTER;
  }

  /**
   * Returns the initial number of unrouted nets at the start of the routing
   * session.
   */
  public int getInitialUnroutedCount() {
    return this.initialUnroutedCount;
  }

  /**
   * Returns the time when the routing session started.
   */
  public Instant getSessionStartTime() {
    return this.sessionStartTime;
  }

  /**
   * Autoroutes ripup passes until the board is completed or the autorouter is
   * stopped by the user. Returns true if the board is completed.
   */
  public boolean runBatchLoop() {
    boolean anyRoutable = false;
    for (int i = 0; i < this.settings.getLayerCount(); i++) {
      if (this.settings.get_layer_active(i) && this.board.layer_structure.arr[i].is_signal) {
        anyRoutable = true;
        break;
      }
    }
    if (!anyRoutable) {
      FRLogger.warn("Cannot start autorouter: all layers are disabled.");
      this.fireTaskStateChangedEvent(new TaskStateChangedEvent(this, TaskState.CANCELLED, 0, this.board.get_hash()));
      throw new IllegalArgumentException("Cannot start autorouter: all layers are disabled.");
    }

    this.fireTaskStateChangedEvent(new TaskStateChangedEvent(this, TaskState.STARTED, 0, this.board.get_hash()));

    // Capture initial state for session summary
    this.sessionStartTime = Instant.now();
    this.initialUnroutedCount = calculateIncompleteCount(this.board);

    boolean continueAutorouting = true;
    BoardHistory bh = new BoardHistory(job.routerSettings.scoring);

    // Record configuration for profiler
    if (this.settings.getLayerCount() > 0) {
      int layerCount = this.settings.getLayerCount();
      double[] prefCosts = new double[layerCount];
      double[] againstCosts = new double[layerCount];
      for (int i = 0; i < layerCount; i++) {
        prefCosts[i] = this.settings.get_preferred_direction_trace_costs(i);
        againstCosts[i] = this.settings.get_against_preferred_direction_trace_costs(i);
      }
      PerformanceProfiler.recordConfiguration(
          this.settings.get_via_costs(),
          this.settings.get_plane_via_costs(),
          prefCosts,
          againstCosts);
    }

    job.logDebug("Checking fanout pre-pass. settings.fanout.enabled=" + this.settings.isFanoutEnabled() + ", smd_pins=" + this.board.get_smd_pins().size());
    // Run SMD fanout pre-pass when the board has SMD pins and fanout is enabled
    if (this.settings.isFanoutEnabled()) {
      if (this.board.get_smd_pins().isEmpty()) {
        job.logInfo("Fanout stage is enabled but skipped because the board has no SMD pins.");
      } else {
        float fanoutCpuSecondsStart = sampleCurrentThreadCpuSeconds();
        float fanoutAllocatedMbStart = sampleCurrentThreadAllocatedMb();
        float fanoutPeakHeapMbAtStart = sampleHeapUsageMb();
        final float[] fanoutPeakHeapMbObserved = new float[] { fanoutPeakHeapMbAtStart };
        // Count pins that actually need fanout. BatchFanout only processes SMD pins that
        // belong to a net, so exclude netless pins from the total. Among net-connected
        // pins, count those that are already fully connected (empty unconnected set).
         int netConnectedSmdPins = 0;
         int alreadyConnectedAtStart = 0;
         for (app.freerouting.board.Pin pin : this.board.get_smd_pins()) {
           if (pin.net_count() > 0) {
             netConnectedSmdPins++;
             if (pin.get_unconnected_set(pin.get_net_no(0)).isEmpty()) {
               alreadyConnectedAtStart++;
             }
           }
         }
         int pinsToFanout = netConnectedSmdPins - alreadyConnectedAtStart;
         job.logInfo("Fanout stage started on board '" + this.board.get_hash() + "' with "
             + pinsToFanout + " of " + this.board.get_smd_pins().size() + " SMD pins needing fanout ("
             + alreadyConnectedAtStart + " already connected, "
             + (this.board.get_smd_pins().size() - netConnectedSmdPins) + " netless).");
        BatchFanout.FanoutRunSummary fanoutSummary = BatchFanout.fanout_board(this.board, this.settings, this.thread,
            status -> {
          fanoutPeakHeapMbObserved[0] = Math.max(fanoutPeakHeapMbObserved[0], sampleHeapUsageMb());
          RouterCounters fanoutCounters = new RouterCounters();
          fanoutCounters.phase = "fanout";
          fanoutCounters.passCount = status.passNo();
          fanoutCounters.queuedToBeRoutedCount = status.pinsToGo();
          fanoutCounters.routedCount = status.routedCount();
          fanoutCounters.skippedCount = 0;
          fanoutCounters.rippedCount = 0;
          fanoutCounters.failedToBeRoutedCount = status.notRoutedCount() + status.insertErrorCount();
          fanoutCounters.incompleteCount = status.boardStatistics().connections.incompleteCount;
          fanoutCounters.fanoutExtraViasCount = status.extraViasThisPass();
          this.fireBoardUpdatedEvent(status.boardStatistics(), fanoutCounters, this.board);

          if (status.passCompleted()) {
            String boardHash = this.board.get_hash();
            String fanoutMessage = String.format(java.util.Locale.US,
                "Fanout pass #%d on board '%s' completed in %.2f seconds with %d SMD pin%s fanouted, %d not routed, %d insert error%s, +%d extra via%s (%d SMD pin%s still to check in pass, ripup costs=%d).",
                status.passNo(), boardHash,
                status.passDurationMillis() / 1000.0,
                status.routedCount(), status.routedCount() == 1 ? "" : "s",
                status.notRoutedCount(),
                status.insertErrorCount(), status.insertErrorCount() == 1 ? "" : "s",
                status.extraViasThisPass(), status.extraViasThisPass() == 1 ? "" : "s",
                status.pinsToGo(), status.pinsToGo() == 1 ? "" : "s",
                status.ripupCosts());
            job.logInfo(fanoutMessage);
          }
        });
        this.fanoutTimedOut = fanoutSummary.isTimedOut();

        float fanoutCpuSecondsEnd = sampleCurrentThreadCpuSeconds();
        float fanoutAllocatedMbEnd = sampleCurrentThreadAllocatedMb();

        float fanoutCpuSecondsUsed;
        if (fanoutCpuSecondsStart >= 0f && fanoutCpuSecondsEnd >= fanoutCpuSecondsStart) {
          fanoutCpuSecondsUsed = fanoutCpuSecondsEnd - fanoutCpuSecondsStart;
        } else {
          fanoutCpuSecondsUsed = Math.max(0f, getCpuSecondsSnapshot(job));
        }

        float fanoutAllocatedMb;
        if (fanoutAllocatedMbStart >= 0f && fanoutAllocatedMbEnd >= fanoutAllocatedMbStart) {
          fanoutAllocatedMb = fanoutAllocatedMbEnd - fanoutAllocatedMbStart;
        } else {
          fanoutAllocatedMb = Math.max(0f, getAllocatedMemoryMbSnapshot(job));
        }

        float fanoutPeakHeapMb = Math.max(fanoutPeakHeapMbObserved[0], sampleHeapUsageMb());
        fanoutPeakHeapMb = Math.max(fanoutPeakHeapMb, getPeakHeapMbSnapshot(job));
        BatchFanout.EscapeStatistics finalEscape = fanoutSummary.escapeStatistics();
        String fanoutCompletionStatus = fanoutSummary.isTimedOut() ? "completed with timeout:"
            : (this.thread.is_stop_auto_router_requested() ? "interrupted:" : "completed:");
        String fanoutSummaryMessage = String.format(java.util.Locale.US,
            "Fanout stage %s started with %d total SMD pins, completed in %.2f seconds, escaped pins: %d/%d (%.1f%%), using %.2f total CPU seconds, %.2f GB total allocated, and %.1f MB peak heap usage.",
            fanoutCompletionStatus,
            finalEscape.totalSmdPins(),
            fanoutSummary.totalDurationMillis() / 1000.0,
            finalEscape.escapedCount(),
            finalEscape.totalSmdPins(),
            finalEscape.escapedPercentage(),
            fanoutCpuSecondsUsed,
            fanoutAllocatedMb / 1024.0f,
            fanoutPeakHeapMb);
        job.logInfo(fanoutSummaryMessage);
      }
    }

    // Phase-5 paired routing: runs once, after fanout and before the first routing pass, so both
    // members of a differential pair are still unrouted and the pair gets first claim on its
    // corridor. Every commit is atomic across the pair (see PairedRouter).
    if (app.freerouting.Freerouting.globalSettings != null
        && app.freerouting.Freerouting.globalSettings.featureFlags.pairedRouting) {
      try {
        PairedRouter paired = new PairedRouter(this.board, this.settings, this.trace_cost_arr);
        for (String line : paired.route_pairs()) {
          job.logInfo("[paired-route] " + line);
        }
        long[] counters = paired.counters();
        job.logInfo("[paired-route] attempted=" + counters[0] + " committed=" + counters[1]
            + " search_failures=" + counters[2] + " validation_rejects=" + counters[3]
            + " pair_rollbacks=" + counters[4] + " skipped=" + counters[5]);
      } catch (Exception e) {
        job.logError("Paired routing failed", e);
      }
    }

    int currentUnrouted = calculateIncompleteCount(this.board);
    boolean isRouterEnabled = this.settings.getRunRouter() && (this.settings.maxPasses == null || this.settings.maxPasses >= 0);
    if (isRouterEnabled) {
      job.logInfo("Auto-routing stage started on board '" + this.board.get_hash() + "' for "
          + currentUnrouted + " unrouted item" + (currentUnrouted == 1 ? "" : "s") + ".");
    }
    continueAutorouting = isRouterEnabled;

    int currentPass = 1;
    int consecutiveNoImprovementPasses = 0;
    boolean fanoutRecoveryApplied = false;
    float lastBestScore = Float.NEGATIVE_INFINITY;   // score at last board-restore or improvement
    float globalBestScore = Float.NEGATIVE_INFINITY; // best score seen across all passes
    int passOfBestScore = 0;                         // pass where globalBestScore was achieved
    int incompleteCountAtBestScore = 0;              // incomplete count when globalBestScore was recorded
    // Track board hashes that have already been routed. If the board does not change between
    // two consecutive passes (same hash at pass start), the router is making no progress and
    // would produce identical decisions with identical ripup budgets — stop immediately rather
    // than waiting for the full stagnation window. This mirrors the v1.9 behaviour and catches
    // the degenerate case where plane-net items repeatedly fail or are inserted+removed each
    // pass without updating the board state.
    Set<String> alreadyRoutedBoardHashes = new java.util.HashSet<>();
    while (continueAutorouting && !this.thread.is_stop_auto_router_requested()) {
      if (job != null && job.state == RoutingJobState.TIMED_OUT) {
        this.thread.request_stop_auto_router();
      }

      String currentBoardHash = this.board.get_hash();

      // Same-hash stop disabled because ripup budgets and random seeds change per-pass, making progress possible in later passes.
      // if (alreadyRoutedBoardHashes.contains(currentBoardHash)) {
      //   job.logInfo("Board state has not changed since pass #" + (currentPass - 1)
      //       + " (hash " + currentBoardHash + "). The auto-router cannot make further progress; stopping.");
      //   thread.request_stop_auto_router();
      //   break;
      // }
      // alreadyRoutedBoardHashes.add(currentBoardHash);

      if (this.settings.maxPasses != null && this.settings.maxPasses > 0 && currentPass > this.settings.maxPasses) {
        thread.request_stop_auto_router();
        break;
      }

      if (job != null) {
        job.setCurrentPass(currentPass);
      }

      this.fireTaskStateChangedEvent(
          new TaskStateChangedEvent(this, TaskState.RUNNING, currentPass, currentBoardHash));

      float boardScoreBefore = new BoardStatistics(this.board).getNormalizedScore(job.routerSettings.scoring);
      bh.add(this.board);

      FRLogger.traceEntry("BatchAutorouter.autoroute_pass #" + currentPass + " on board '" + currentBoardHash + "'");

      continueAutorouting = autoroute_pass(currentPass);

      // NOTE (measured): running the meander matcher after pass 1 as a corridor
      // reservation inserted ZERO bumps in both runs (pass-1 mismatch 161466, every
      // candidate failing DRC against the still-messy pass-1 routes) -- final-state
      // insertion is strictly better; see docs/dense-bga-roadmap.md.

      BoardStatistics boardStatisticsAfter = new BoardStatistics(this.board);
      float boardScoreAfter = boardStatisticsAfter.getNormalizedScore(job.routerSettings.scoring);

      if ((bh.size() >= STOP_AT_PASS_MINIMUM) || (this.thread.is_stop_auto_router_requested())) {
        if (((currentPass % STOP_AT_PASS_MODULO == 0) && (currentPass >= STOP_AT_PASS_MINIMUM))
            || (this.thread.is_stop_auto_router_requested())) {
          // Check if the score improved compared to the previous passes, restore a
          // previous board if not. Use strict ">" so that equally-scored boards do NOT
          // trigger a restore — if every board has the same (possibly zero) score the old
          // ">=" test would restore on every check cycle, growing the history unboundedly
          // and never stopping.
          if (bh.getMaxScore() > boardScoreAfter) {
            var boardToRestore = bh.restoreBoard(MAXIMUM_TRIES_ON_THE_SAME_BOARD);
            if (boardToRestore == null) {
              job.logInfo("The router was not able to improve the board, stopping the auto-router.");
              thread.request_stop_auto_router();
              break;
            }

            int boardToRestoreRank = bh.getRank(boardToRestore);

            if (boardToRestoreRank > BOARD_RANK_LIMIT) {
              thread.request_stop_auto_router();
              break;
            }

            this.board = boardToRestore;
            var boardStatistics = this.board.get_statistics();
            // Reset pass-local stagnation counter when restoring a previous board state
            consecutiveNoImprovementPasses = 0;
            boardStatisticsAfter = boardStatistics;
            boardScoreAfter = boardStatisticsAfter.getNormalizedScore(job.routerSettings.scoring);
            lastBestScore = boardScoreAfter;
            currentBoardHash = this.board.get_hash();
            // Reset the same-hash set after a board restore: the restored board will be
            // routed with a higher ripup budget on subsequent passes, so earlier routing
            // decisions from the same hash may no longer apply.
            alreadyRoutedBoardHashes.clear();
            job.logDebug(
                "Restoring an earlier board that has the score of "
                    + FRLogger.formatScore(boardScoreAfter,
                        boardStatisticsAfter.connections.incompleteCount,
                        boardStatisticsAfter.clearanceViolations.totalCount)
                    + ".");
          }
        }
      }
      double autorouter_pass_duration = FRLogger
          .traceExit("BatchAutorouter.autoroute_pass #" + currentPass + " on board '" + currentBoardHash + "'");

      String passCompletedMessage = String.format(java.util.Locale.US,
          "Auto-routing pass #%d on board '%s' was completed in %.2f seconds with the score of %s",
          currentPass, currentBoardHash, autorouter_pass_duration,
          FRLogger.formatScore(boardScoreAfter, boardStatisticsAfter.connections.incompleteCount,
              boardStatisticsAfter.clearanceViolations.totalCount));
      if (job.resourceUsage.cpuTimeUsed > 0) {
        passCompletedMessage += String.format(java.util.Locale.US, ", using %.2f CPU seconds and the job allocated %.2f GB of memory so far.",
            job.resourceUsage.cpuTimeUsed, job.resourceUsage.maxMemoryUsed / 1024.0f);
      } else {
        passCompletedMessage += ".";
      }
      if (!isOptimizerAutorouter) {
        job.logInfo(passCompletedMessage);
      }
      log_rule_region_summary("pass #" + currentPass);

      DesignRulesChecker tempDrc = new DesignRulesChecker(this.board, null);
      tempDrc.calculateAllIncompletes();
      StringBuilder perNetBreakdown = new StringBuilder();
      for (int netNo = 1; netNo <= this.board.rules.nets.max_net_no(); netNo++) {
        int netIncomplete = tempDrc.getIncompleteCount(netNo);
        if (netIncomplete > 0) {
          FRLogger.trace(
              "BatchAutorouter.autoroute_pass",
              "compare_unrouted_net",
              "pass=" + currentPass + ", net=" + netNo + ", incomplete=" + netIncomplete,
              "Net #" + netNo,
              new Point[0]);
          if (!perNetBreakdown.isEmpty()) {
            perNetBreakdown.append(',');
          }
          perNetBreakdown.append(netNo).append('=').append(netIncomplete);
        }
      }
      FRLogger.trace("BatchAutorouter.autoroute_pass", "compare_unrouted_breakdown",
          "pass=" + currentPass
              + ", total=" + tempDrc.getIncompleteCount()
              + ", breakdown=" + perNetBreakdown,
          "",
          new Point[0]);

      if (this.settings.save_intermediate_stages) {
        fireBoardSnapshotEvent(this.board);
      }

      // Stagnation detection: abort when the normalized score hasn't improved by
      // at least STAGNATION_SCORE_THRESHOLD over STAGNATION_PASS_LIMIT consecutive
      // passes. This now fires whenever the router is still actively running
      // (continueAutorouting == true) after the mandatory minimum passes, regardless
      // of incompleteCount.  The old condition guarded on incompleteCount > 0, which
      // caused the check to be bypassed — and the counter to be silently reset — for
      // boards where DRC shows 0 incompletes but the router keeps cycling (e.g. when
      // plane-net false-work items kept autoroute_pass() returning true).  If the
      // board is genuinely done (continueAutorouting == false) the while-loop exits
      // naturally and we never reach this block.
      if (currentPass >= STOP_AT_PASS_MINIMUM && continueAutorouting) {

        // --- Pass-local counter (resets after board restores) ---
        if (boardScoreAfter > lastBestScore + STAGNATION_SCORE_THRESHOLD) {
          consecutiveNoImprovementPasses = 0;
          lastBestScore = boardScoreAfter;
        } else {
          consecutiveNoImprovementPasses++;

          // One-time recovery for fanout-enabled jobs: aggressively remove tails, including
          // fanout vias, when score plateaus with remaining incompletes. This gives the
          // autorouter a chance to escape local dead-ends introduced by pre-fanout geometry
          // while keeping fanout enabled as the default behavior.
          if (this.settings.isFanoutEnabled()
              && !fanoutRecoveryApplied
              && boardStatisticsAfter.connections.incompleteCount > 0
              && consecutiveNoImprovementPasses >= FANOUT_RECOVERY_STAGNATION_PASSES) {
            int incompletesBeforeRecovery = boardStatisticsAfter.connections.incompleteCount;
            remove_tails(Item.StopConnectionOption.NONE);
            boardStatisticsAfter = new BoardStatistics(this.board);
            boardScoreAfter = boardStatisticsAfter.getNormalizedScore(job.routerSettings.scoring);
            lastBestScore = boardScoreAfter;
            consecutiveNoImprovementPasses = 0;
            fanoutRecoveryApplied = true;
            alreadyRoutedBoardHashes.clear();
            job.logDebug("Applied one-time fanout recovery cleanup (removed fanout tails/vias). "
                + "Incompletes: " + incompletesBeforeRecovery + " -> "
                + boardStatisticsAfter.connections.incompleteCount + ".");
          }

          if (consecutiveNoImprovementPasses >= STAGNATION_PASS_LIMIT) {
            String report = buildUnroutedConnectionsReport();
            job.logInfo("The router's score (" + FRLogger.defaultFloatFormat.format(boardScoreAfter)
                + ") has not improved by more than " + STAGNATION_SCORE_THRESHOLD
                + " points in the last " + STAGNATION_PASS_LIMIT + " passes ("
                + boardStatisticsAfter.connections.incompleteCount + " item"
                + (boardStatisticsAfter.connections.incompleteCount == 1 ? "" : "s")
                + " still unconnected). Stopping the auto-router.\n"
                + "The following connections could not be routed -- please review your design "
                + "(e.g. check pad clearances, trace width rules, and available routing space):\n"
                + report);
            thread.request_stop_auto_router();
            break;
          }
        }

        // --- Global best tracker (not reset by board restores) ---
        // Stops the router if no pass anywhere has meaningfully improved the score
        // in the last STAGNATION_PASS_LIMIT passes, even across board-restore cycles.
        if (boardScoreAfter > globalBestScore + STAGNATION_SCORE_THRESHOLD) {
          globalBestScore = boardScoreAfter;
          passOfBestScore = currentPass;
          incompleteCountAtBestScore = boardStatisticsAfter.connections.incompleteCount;
        } else if ((currentPass - passOfBestScore) >= STAGNATION_PASS_LIMIT) {
          String report = buildUnroutedConnectionsReport();
          job.logInfo("The router's best score (" + FRLogger.defaultFloatFormat.format(globalBestScore)
              + ") has not improved by more than " + STAGNATION_SCORE_THRESHOLD
              + " points since pass #" + passOfBestScore
              + ". Stopping the auto-router after " + currentPass + " passes ("
              + incompleteCountAtBestScore + " item"
              + (incompleteCountAtBestScore == 1 ? "" : "s")
              + " still unconnected).\n"
              + "The following connections could not be routed -- please review your design "
              + "(e.g. check pad clearances, trace width rules, and available routing space):\n"
              + report);
          thread.request_stop_auto_router();
          break;
        }

      } else if (boardStatisticsAfter.connections.incompleteCount == 0 && boardScoreAfter > STAGNATION_SCORE_THRESHOLD) {
        // Board is fully routed AND has a positive score (genuine success).
        // A fully-routed board with score == 0 (e.g. caused by clearance violations
        // from plane routing) must NOT reset the stagnation counter; it should keep
        // accumulating until the global tracker fires.
        consecutiveNoImprovementPasses = 0;
        lastBestScore = boardScoreAfter;
      }

      // check if there are still unrouted items
      if (continueAutorouting && !this.thread.is_stop_auto_router_requested()) {
        currentPass++;
      }
    }

    log_rule_region_summary("stage");

    // Ensure we finish with the best board ever seen during this routing session.
    // When stagnation or the max-pass limit fires, the loop exits with the board from the last
    // completed pass, which may be worse than an earlier pass that was recorded in the history.
    float currentFinalScore = new BoardStatistics(this.board).getNormalizedScore(job.routerSettings.scoring);
    float bestHistoryScore = bh.getMaxScore();
    if (bestHistoryScore > currentFinalScore) {
      RoutingBoard bestBoard = bh.restoreBestBoard();
      if (bestBoard != null) {
        BoardStatistics currentStats = new BoardStatistics(this.board);
        this.board = bestBoard;
        BoardStatistics bestStats = new BoardStatistics(this.board);
        job.logDebug("The final board state (score "
            + FRLogger.formatScore(currentFinalScore,
                currentStats.connections.incompleteCount,
                currentStats.clearanceViolations.totalCount)
            + ") is worse than the best board seen during routing (score "
            + FRLogger.formatScore(bestStats.getNormalizedScore(job.routerSettings.scoring),
                bestStats.connections.incompleteCount,
                bestStats.clearanceViolations.totalCount)
            + "). Restoring the best board as the final result.");
      }
    }

    job.board = this.board;

    boolean wasRouterRun = this.settings.getRunRouter() && (this.settings.maxPasses == null || this.settings.maxPasses >= 0);
    if (wasRouterRun && !(this.remove_unconnected_vias || continueAutorouting || this.thread.is_stop_auto_router_requested())) {
      // clean up the route if the board is completed and if fanout is used.
      remove_tails(Item.StopConnectionOption.NONE);
    }

    bh.clear();

    if (app.freerouting.Freerouting.globalSettings != null
        && app.freerouting.Freerouting.globalSettings.featureFlags.meanderMatching) {
      // Phase-5 length matching: runs once, after the routing passes, so meanders are not
      // ripped by later passes; SHOVE_FIXED insertion protects them from pull-tight.
      try {
        for (String line : MeanderMatcher.match(this.board)) {
          job.logInfo("[meander] " + line);
        }
      } catch (Exception e) {
        job.logError("Meander matching failed", e);
      }
    }

    // Final per-pair routed-length report -- the A/B metric for paired routing. Emitted with the
    // paired flag on, and on demand (-Dfr.pairlength) so the unpaired baseline of the same
    // fixture can be measured without turning any routing behaviour on.
    if ((app.freerouting.Freerouting.globalSettings != null
        && app.freerouting.Freerouting.globalSettings.featureFlags.pairedRouting)
        || Boolean.getBoolean("fr.pairlength")) {
      try {
        for (var pair : MeanderMatcher.detect_pairs(this.board).entrySet()) {
          double len_p = PairedRouter.net_length(this.board, pair.getKey());
          double len_n = PairedRouter.net_length(this.board, pair.getValue());
          job.logInfo("[pair-length] " + pair.getKey() + "/" + pair.getValue()
              + ": len+=" + Math.round(len_p) + " len-=" + Math.round(len_n)
              + " mismatch=" + Math.round(Math.abs(len_p - len_n)));
        }
      } catch (Exception e) {
        job.logError("Pair length report failed", e);
      }
    }

    // Print all profiling results at the end of session
    PerformanceProfiler.printResults();
    PerformanceProfiler.reset();

    if (!this.thread.is_stop_auto_router_requested()) {
      this.fireTaskStateChangedEvent(new TaskStateChangedEvent(this, TaskState.FINISHED,
          currentPass, this.board.get_hash()));
    } else {
      // Distinguish between a user-requested cancellation and a job timeout so that
      // API consumers can tell the two apart via TaskStateChangedEvent.
      boolean isTimedOut = (job != null) && (job.state == RoutingJobState.TIMED_OUT);
      this.fireTaskStateChangedEvent(new TaskStateChangedEvent(this,
          isTimedOut ? TaskState.TIMED_OUT : TaskState.CANCELLED,
          currentPass, this.board.get_hash()));
    }

    return !this.thread.is_stop_auto_router_requested();
  }

  private String buildUnroutedConnectionsReport() {
    DesignRulesChecker tempDrc = new DesignRulesChecker(this.board, null);
    tempDrc.calculateAllIncompletes();
    AirLine[] airlines = tempDrc.getAllAirlines();

    if (airlines == null || airlines.length == 0) {
      return "  (no unrouted connections found)";
    }

    // Group airlines by net name for a cleaner report
    Map<String, List<String>> byNet = new LinkedHashMap<>();
    for (AirLine al : airlines) {
      String netName = al.net != null ? al.net.name : "(unknown net)";
      String fromDesc = describeItem(al.from_item);
      String toDesc   = describeItem(al.to_item);
      byNet.computeIfAbsent(netName, k -> new ArrayList<>())
           .add("    - " + fromDesc + "  ->  " + toDesc);
    }

    StringBuilder sb = new StringBuilder();
    for (Map.Entry<String, List<String>> entry : byNet.entrySet()) {
      int count = entry.getValue().size();
      sb.append("  Net '").append(entry.getKey()).append("' (")
        .append(count).append(" unrouted connection").append(count == 1 ? "" : "s").append("):\n");
      for (String line : entry.getValue()) {
        sb.append(line).append('\n');
      }
    }
    return sb.toString().stripTrailing();
  }

  /**
   * Returns a short, user-friendly description of a board item suitable for the
   * stagnation report.  For pins the format is {@code ComponentName-PinName}
   * (e.g. {@code J2-A3}); for all other item types a generic fallback is used.
   */
  private String describeItem(Item item) {
    if (item instanceof Pin pin) {
      try {
        app.freerouting.board.Component comp = board.components.get(pin.get_component_no());
        if (comp != null) {
          app.freerouting.core.Package pkg = comp.get_package();
          if (pkg != null) {
            app.freerouting.core.Package.Pin pkgPin = pkg.get_pin(pin.pin_no);
            if (pkgPin != null) {
              return comp.name + "-" + pkgPin.name;
            }
          }
          return comp.name + " (pin #" + pin.pin_no + ")";
        }
      } catch (Exception e) {
        // fall through to generic
      }
    }
    return item != null ? item.toString() : "(unknown)";
  }

  private void remove_tails(Item.StopConnectionOption p_stop_connection_option) {
    board.start_marking_changed_area();
    board.remove_trace_tails(-1, p_stop_connection_option);
    board.opt_changed_area(new int[0], null, this.trace_pull_tight_accuracy, this.trace_cost_arr, this.thread,
        pull_tight_limit());
  }

  // Tries to route an item on a specific net. Returns true, if the item is
  // routed.
  private AutorouteAttemptResult autoroute_item(Item p_item, int p_route_net_no, SortedSet<Item> p_ripped_item_list,
      Map<Item, Integer> p_ripup_costs, int p_ripup_pass_no) {
    try {
      boolean contains_plane = false;

      // Get the net
      Net route_net = board.rules.nets.get(p_route_net_no);
      if (route_net != null) {
        contains_plane = route_net.contains_plane();
      }

      // Get the current via costs based on auto-router settings
      int curr_via_costs;
      if (contains_plane) {
        curr_via_costs = this.settings.get_plane_via_costs();
      } else {
        curr_via_costs = this.settings.get_via_costs();
      }

      // Get and calculate the auto-router settings based on the board and net we are
      // working on
      AutorouteControl autoroute_control = new AutorouteControl(this.board, p_route_net_no, settings, curr_via_costs,
          this.trace_cost_arr);
      autoroute_control.ripup_allowed = true;
      autoroute_control.ripup_costs = this.start_ripup_costs * p_ripup_pass_no;
      autoroute_control.remove_unconnected_vias = this.remove_unconnected_vias;

      // Check if the item is already routed
      Set<Item> unconnected_set = p_item.get_unconnected_set(p_route_net_no);
      if (unconnected_set.isEmpty()) {
        return new AutorouteAttemptResult(AutorouteAttemptState.NO_UNCONNECTED_NETS);
      }

      Set<Item> connected_set = p_item.get_connected_set(p_route_net_no);
      Set<Item> route_start_set;
      Set<Item> route_dest_set;
      if (contains_plane) {
        for (Item curr_item : connected_set) {
          if (curr_item instanceof ConductionArea) {
            return new AutorouteAttemptResult(AutorouteAttemptState.CONNECTED_TO_PLANE);
          }
        }
      }
      if (contains_plane) {
        route_start_set = connected_set;
        route_dest_set = unconnected_set;
      } else {
        route_start_set = unconnected_set;
        route_dest_set = connected_set;
      }

      // Calculate the shortest distance between the two sets of items
      calc_airline(route_start_set, route_dest_set);

      // Calculate the maximum time for this autoroute pass, capped by the optional
      // per-item budget (router.max_milliseconds_per_item).
      TimeLimit time_limit = create_connection_time_limit(p_ripup_pass_no);

      // Stage-2 partition router: try the cheap canonical-partition search first; any
      // failure, unsupported case, or commit-time conflict falls through to the classic
      // engine below, so the worst case is the status quo plus a fast failed attempt.
      // Early passes only: partition successes concentrate where the board is still open
      // (measured: 11 of 12 in pass 1, the rest in pass 2, none later), while every attempt
      // pays the per-attempt freshness cost -- late-pass attempts are pure overhead.
      // Acceptance policy under test: only LONG connections go to the partition router.
      // Short local connections are cheap for the classic engine, and their greedy partition
      // versions were measured to fragment corridors the endgame needs.
      double airline_distance = (this.air_line != null && this.air_line.a != null
          && this.air_line.b != null) ? this.air_line.a.distance(this.air_line.b) : -1;
      boolean long_connection = airline_distance >= 150000;
      // Inside a lookahead probe the horizon is routed by the classic engine only: a probe
      // that could itself commit partition routes would recurse into further probes, and its
      // cost would stop being bounded by the pass it simulates.
      if (isPartitionRouterEnabled() && !this.in_lookahead && !contains_plane
          && p_ripup_pass_no <= 1 && long_connection) {
        AutorouteAttemptResult partition_result = try_partition_route(route_start_set, route_dest_set,
            autoroute_control, airline_distance);
        if (partition_result != null) {
          return partition_result;
        }
      }

      // Initialize the auto-router engine
      AutorouteEngine autoroute_engine = board.init_autoroute(p_route_net_no,
          autoroute_control.trace_clearance_class_no, this.thread, time_limit, this.retain_autoroute_database);

      int maxItemIdBeforeRoute = board.communication.id_no_generator.max_generated_no();

      byte[] strictDrcBoardSnapshot = this.settings.isStrictDrc() ? board.serialize(false) : null;

      // Do the auto-routing between the two sets of items
      AutorouteAttemptResult autoroute_result = autoroute_engine.autoroute_connection(route_start_set, route_dest_set,
          autoroute_control, p_ripped_item_list, p_ripup_costs);

      // Update the changed area of the board
      if (autoroute_result.state == AutorouteAttemptState.ROUTED) {
        int maxItemIdBeforeOpt = board.communication.id_no_generator.max_generated_no();
        FRLogger.trace("compare_trace_opt_changed_area_before net=" + p_route_net_no + ", maxItemId=" + maxItemIdBeforeOpt);
        board.opt_changed_area(new int[0], null, this.trace_pull_tight_accuracy, autoroute_control.trace_costs,
            this.thread, pull_tight_limit());
        int maxItemIdAfterOpt = board.communication.id_no_generator.max_generated_no();
        FRLogger.trace("compare_trace_opt_changed_area_after net=" + p_route_net_no + ", maxItemId=" + maxItemIdAfterOpt + ", delta=" + (maxItemIdAfterOpt - maxItemIdBeforeOpt));
      }

      if ((autoroute_result.state == AutorouteAttemptState.FAILED
          || autoroute_result.state == AutorouteAttemptState.INSERT_ERROR)
          && isRuleRegionsEnabled()) {
        AutorouteAttemptResult region_result = retryConnectionInRegion(p_route_net_no,
            curr_via_costs, route_start_set, route_dest_set, p_ripped_item_list, p_ripup_costs,
            p_ripup_pass_no, time_limit);
        if (region_result != null) {
          if (region_result.state == AutorouteAttemptState.ROUTED) {
            AutorouteAttemptResult strict_result = applyStrictDrcAfterRoute(p_route_net_no,
                maxItemIdBeforeRoute, strictDrcBoardSnapshot);
            if (strict_result != null) {
              return strict_result;
            }
            return region_result;
          }
          // The retry mutated the board and was rolled back through the undo stack. Report the
          // original failure without chaining further retries: the connection has already been
          // attempted at both the global and the region rules on this board state, and the
          // later retries in this method (neck width, etc.) are not meaningful after a rip-up
          // rollback.
          return autoroute_result;
        }
      }

      if ((autoroute_result.state == AutorouteAttemptState.FAILED
          || autoroute_result.state == AutorouteAttemptState.INSERT_ERROR)
          && this.settings.getNeckWidthUm() > 0) {
        AutorouteAttemptResult necked_result = retryConnectionNecked(p_route_net_no, autoroute_control,
            curr_via_costs, route_start_set, route_dest_set, p_ripped_item_list, p_ripup_costs,
            p_ripup_pass_no, time_limit);
        if (necked_result != null) {
          AutorouteAttemptResult strict_result = applyStrictDrcAfterRoute(p_route_net_no,
              maxItemIdBeforeRoute, strictDrcBoardSnapshot);
          if (strict_result != null) {
            return strict_result;
          }
          return necked_result;
        }
      }

      if (autoroute_result.state == AutorouteAttemptState.ROUTED) {
        AutorouteAttemptResult strict_result = applyStrictDrcAfterRoute(p_route_net_no,
            maxItemIdBeforeRoute, strictDrcBoardSnapshot);
        if (strict_result != null) {
          return strict_result;
        }
      }

      if (autoroute_result.state != AutorouteAttemptState.ROUTED) {
        log_per_item_budget_exceeded(time_limit, p_route_net_no);
      }

      return autoroute_result;
    } catch (Exception e) {
      FRLogger.error("Error during routing passes", e);
      return new AutorouteAttemptResult(AutorouteAttemptState.FAILED);
    } finally {
      if (partition_router != null) {
        // Every path through here may have mutated the board (classic engine commit, ripup,
        // necked retry, pull-tight); a partition that misses freshly committed copper plans
        // routes through occupied space and later attempts fail the insertability check.
        // Refresh every attempt: amortizing to every 8th attempt was measured to save almost
        // no wall-clock (140.9 s vs 145 s -- the rebuild is not the dominant cost) while
        // dropping the final score below the gate (979.47/4 vs 994.85/1): stale plans that
        // still pass the pre-insert check commit worse routes.
        partition_router.invalidate();
      }
    }
  }


  /**
   * Width-necking retry: when a connection failed at its net-class trace width and the
   * neck_width_um setting is enabled, retry it ONCE with every layer's trace half-width
   * clamped to the neck width. Fine-pitch pads whose pitch is below (class width +
   * clearance) are unroutable at class width and fail as generic congestion; the operator
   * supplies a legal manufacturable neck width (e.g. the project's densest net class).
   * Returns the retry result when it routed, else null (keep the original failure).
   */
  private AutorouteAttemptResult retryConnectionNecked(int p_route_net_no,
      AutorouteControl p_original_control, int p_via_costs, Set<Item> p_route_start_set,
      Set<Item> p_route_dest_set, SortedSet<Item> p_ripped_item_list,
      Map<Item, Integer> p_ripup_costs, int p_ripup_pass_no, TimeLimit p_time_limit) {
    int boardResolution = Math.max(1, board.communication.resolution);
    int neck_width = (int) Math.round(app.freerouting.board.Unit.scale(
        this.settings.getNeckWidthUm() * boardResolution, app.freerouting.board.Unit.UM,
        board.communication.unit));
    int neck_half_width = Math.max(1, neck_width / 2);
    boolean narrower_somewhere = false;
    for (int i = 0; i < p_original_control.layer_count; i++) {
      if (p_original_control.layer_active[i]
          && p_original_control.trace_half_width[i] > neck_half_width) {
        narrower_somewhere = true;
        break;
      }
    }
    if (!narrower_somewhere) {
      return null;
    }
    AutorouteControl neck_control = new AutorouteControl(this.board, p_route_net_no, settings,
        p_via_costs, this.trace_cost_arr);
    neck_control.ripup_allowed = true;
    neck_control.ripup_costs = this.start_ripup_costs * p_ripup_pass_no;
    neck_control.remove_unconnected_vias = this.remove_unconnected_vias;
    for (int i = 0; i < neck_control.layer_count; i++) {
      int compensation = neck_control.compensated_trace_half_width[i] - neck_control.trace_half_width[i];
      neck_control.trace_half_width[i] = Math.min(neck_control.trace_half_width[i], neck_half_width);
      neck_control.compensated_trace_half_width[i] = neck_control.trace_half_width[i] + compensation;
    }
    AutorouteEngine neck_engine = board.init_autoroute(p_route_net_no,
        neck_control.trace_clearance_class_no, this.thread, p_time_limit, this.retain_autoroute_database);
    AutorouteAttemptResult neck_result = neck_engine.autoroute_connection(p_route_start_set,
        p_route_dest_set, neck_control, p_ripped_item_list, p_ripup_costs);
    if (neck_result.state != AutorouteAttemptState.ROUTED) {
      return null;
    }
    board.opt_changed_area(new int[0], null, this.trace_pull_tight_accuracy, neck_control.trace_costs,
        this.thread, pull_tight_limit());
    Net route_net = board.rules.nets.get(p_route_net_no);
    FRLogger.info("Necked retry routed net '"
        + (route_net != null ? route_net.name : "#" + p_route_net_no)
        + "' at " + this.settings.getNeckWidthUm() + " um trace width.");
    return neck_result;
  }

  /**
   * Region-retry accounting. v1 logged only the two terminal outcomes (kept / rolled back),
   * which made "the trigger never fired" indistinguishable from "the retry ran and failed
   * again" -- on a real board both look like silence. These separate the stages: how many
   * failed connections reached the retry at all, how many had no region to trigger on, how
   * many actually ran, and how each run ended. All are plain longs bumped on the sequential
   * router path only and reported once per routing stage under the flag, so flag-off output
   * and cost are unchanged.
   */
  private long region_retry_candidates;
  /** Candidates where findTriggeringRegion returned null: no retry was attempted. */
  private long region_retry_no_region;
  /** Candidates where the retry actually ran (a region triggered). */
  private long region_retry_attempted;
  /** Attempts where the maze search still did not route the connection. */
  private long region_retry_route_failed;
  /** Attempts that routed but were rolled back by the outside-region global-rule scope check. */
  private long region_retry_scope_rollback;
  /** Attempts that routed and were kept (the strict-DRC gate may still reject them later). */
  private long region_retry_kept;

  /**
   * One summary line with the full region-retry funnel (counters are cumulative over the
   * stage), or nothing at all when region rules are off, so flag-off log output stays
   * byte-identical.
   *
   * <p>Emitted at the end of every pass as well as at the end of the stage. On a big fixture
   * the routing budget usually expires mid-pass and the pass loop is never left, so an
   * end-of-stage-only line is exactly the line you do not get on the boards where you need it.
   *
   * @param p_scope "pass #N" or "stage", so the two emission sites are distinguishable.
   */
  private void log_rule_region_summary(String p_scope) {
    if (!isRuleRegionsEnabled()) {
      return;
    }
    job.logInfo("[rule-region] " + p_scope
        + " retry_candidates=" + region_retry_candidates
        + " no_region=" + region_retry_no_region
        + " attempted=" + region_retry_attempted
        + " route_failed=" + region_retry_route_failed
        + " scope_rollback=" + region_retry_scope_rollback
        + " kept=" + region_retry_kept
        + " regions=" + (this.board.rule_regions != null ? this.board.rule_regions.size() : 0));
  }

  /** Region-scoped rules are active only when the flag is on AND regions were installed. */
  private boolean isRuleRegionsEnabled() {
    return this.board.rule_regions != null
        && app.freerouting.Freerouting.globalSettings.featureFlags.ruleRegions;
  }

  /**
   * Region-rules retry (docs/dense-bga-roadmap.md Phase 1 item 1): when a connection failed
   * at its global rules and one of its terminals touches a rule region, retry it ONCE at the
   * region's rules -- the region's clearance class (so the maze search's compensated
   * obstacle expansion shrinks to the region clearance) and, when the region overrides it,
   * every layer's trace half-width clamped to the region half-width.
   *
   * <p>Region scoping is enforced AFTER the retry routes: every newly inserted trace shape
   * that is not fully inside the triggering region must still pass the global-clearance
   * insertability check, otherwise the whole retry is rolled back via a board snapshot and
   * the original failure stands. Inside the region the inserted traces carry the region's
   * clearance class, so the scored DRC (clearance matrix driven) agrees that they are legal.
   *
   * <p>v1 limits, deliberate: vias keep their global clearance class (regions neck traces,
   * not vias); the retry runs in the sequential router path only; and a post-retry shove by a
   * LATER connection could in principle push a region-class trace outside its region without
   * re-checking the global rule there (same exposure window as any shove; strict_drc catches
   * it when enabled).
   *
   * @return a ROUTED result when the retry succeeded and was kept; a non-ROUTED result when
   *         the retry ran and was rolled back through the board's undo stack (the board
   *         object and its search trees survive the rollback, but the caller still reports
   *         the original failure rather than chaining another retry); or null when no region
   *         triggered and the board was not touched.
   */
  private AutorouteAttemptResult retryConnectionInRegion(int p_route_net_no, int p_via_costs,
      Set<Item> p_route_start_set, Set<Item> p_route_dest_set, SortedSet<Item> p_ripped_item_list,
      Map<Item, Integer> p_ripup_costs, int p_ripup_pass_no, TimeLimit p_time_limit) {
    ++region_retry_candidates;
    app.freerouting.board.RuleRegion region = findTriggeringRegion(p_route_start_set, p_route_dest_set);
    if (region == null) {
      ++region_retry_no_region;
      return null;
    }
    ++region_retry_attempted;
    Net route_net = board.rules.nets.get(p_route_net_no);
    String net_name = (route_net != null) ? route_net.name : ("#" + p_route_net_no);
    int max_item_id_before = board.communication.id_no_generator.max_generated_no();
    // Roll back through the board's own undo stack, NOT through serialize/deserialize.
    // Measured on the 8-layer repro (docs/dense-bga-roadmap.md): a byte[]-snapshot rollback
    // replaces this.board with a freshly deserialized copy, whose search_tree_manager is
    // transient -- so every rolled-back retry silently threw away EVERY compensated search
    // tree and the next connection rebuilt them from all ~12k board items. 83 region retries
    // in one pass caused 145 full-board tree builds (73 of them rebuilds of the plain global
    // tree) and OOM-ed a 2 GB heap inside ObjectInputStream. undo() keeps the same board and
    // the same trees, updating them incrementally for the handful of items the retry touched
    // -- the same rollback mechanism the partition commit path and the lookahead already use.
    //
    // Two preconditions make the now-SURVIVING region tree safe. (1) The engine is created
    // with retain_autoroute_database = false, so AutorouteEngine.clear() runs at the end of
    // every autoroute_connection and removes that connection's expansion rooms from the tree
    // it searched -- nothing stale is left behind in a tree that outlives the attempt.
    // (2) undo() routes its removals and insertions through search_tree_manager, which
    // updates EVERY compensated tree, so the region tree stays in sync with the board.
    board.generate_snapshot();

    AutorouteControl region_control = new AutorouteControl(this.board, p_route_net_no, settings,
        p_via_costs, this.trace_cost_arr);
    region_control.ripup_allowed = true;
    region_control.ripup_costs = this.start_ripup_costs * p_ripup_pass_no;
    region_control.remove_unconnected_vias = this.remove_unconnected_vias;
    int global_clearance_class = region_control.trace_clearance_class_no;
    region_control.trace_clearance_class_no = region.clearance_class_no;
    for (int i = 0; i < region_control.layer_count; i++) {
      if (region.trace_half_width > 0) {
        region_control.trace_half_width[i] = Math.min(region_control.trace_half_width[i],
            region.trace_half_width);
      }
      region_control.compensated_trace_half_width[i] = region_control.trace_half_width[i]
          + board.rules.clearance_matrix.clearance_compensation_value(region.clearance_class_no, i);
    }

    AutorouteEngine region_engine = board.init_autoroute(p_route_net_no, region.clearance_class_no,
        this.thread, p_time_limit, this.retain_autoroute_database);
    AutorouteAttemptResult region_result = region_engine.autoroute_connection(p_route_start_set,
        p_route_dest_set, region_control, p_ripped_item_list, p_ripup_costs);
    if (region_result.state != AutorouteAttemptState.ROUTED) {
      // The failed attempt may have ripped or shoved items: restore the exact pre-retry
      // board so the retry is invisible unless it succeeds.
      ++region_retry_route_failed;
      board.undo(null);
      return region_result;
    }
    board.opt_changed_area(new int[0], null, this.trace_pull_tight_accuracy, region_control.trace_costs,
        this.thread, TIME_LIMIT_TO_PREVENT_ENDLESS_LOOP);
    String reject_reason = checkRegionScopedInsertion(p_route_net_no, max_item_id_before, region,
        global_clearance_class);
    if (reject_reason != null) {
      ++region_retry_scope_rollback;
      FRLogger.info("[rule-region] retry for net '" + net_name + "' rolled back: " + reject_reason);
      board.undo(null);
      return new AutorouteAttemptResult(AutorouteAttemptState.FAILED,
          "rule_region: " + reject_reason);
    }
    // Kept: drop the undo snapshot so the stack does not grow per kept retry.
    board.pop_snapshot();
    ++region_retry_kept;
    FRLogger.info("[rule-region] routed net '" + net_name + "' at region rules ("
        + region.name + ", clearance=" + region.clearance
        + ", trace_half_width=" + region.trace_half_width + ").");
    return region_result;
  }

  /**
   * The first configured region (settings order, deterministic) whose box intersects the
   * bounding box of any terminal item of the connection, or null. v1 trigger semantics: a
   * terminal touching the region is what marks a connection as region-relevant.
   */
  private app.freerouting.board.RuleRegion findTriggeringRegion(Set<Item> p_route_start_set,
      Set<Item> p_route_dest_set) {
    if (board.rule_regions == null) {
      return null;
    }
    for (app.freerouting.board.RuleRegion region : board.rule_regions) {
      for (Item curr_item : p_route_start_set) {
        if (region.box.intersects(curr_item.bounding_box())) {
          return region;
        }
      }
      for (Item curr_item : p_route_dest_set) {
        if (region.box.intersects(curr_item.bounding_box())) {
          return region;
        }
      }
    }
    return null;
  }

  /**
   * Region scoping check for a kept region retry: every tile shape of every newly inserted
   * trace (item id above p_max_item_id_before) that is NOT fully inside the triggering
   * region must pass the insertability check at the global clearance class. Returns a
   * human-readable reason for the first failure, or null when the insertion is properly
   * region-scoped. Vias are not checked: they keep their global clearance class throughout.
   */
  private String checkRegionScopedInsertion(int p_route_net_no, int p_max_item_id_before,
      app.freerouting.board.RuleRegion p_region, int p_global_clearance_class) {
    for (Item curr_item : board.get_connectable_items(p_route_net_no)) {
      if (curr_item.get_id_no() <= p_max_item_id_before
          || !(curr_item instanceof PolylineTrace curr_trace)) {
        continue;
      }
      int check_class = (curr_trace.clearance_class_no() == p_region.clearance_class_no)
          ? p_global_clearance_class
          : curr_trace.clearance_class_no();
      int failing_shape = board.first_trace_shape_outside_region_failing_global_rule(
          curr_trace, p_region, check_class);
      if (failing_shape >= 0) {
        return "trace shape " + failing_shape + " of item #" + curr_item.get_id_no()
            + " outside region " + p_region.name + " fails the global clearance rule";
      }
    }
    return null;
  }

  /**
   * When {@code strict_drc} rejects a routed connection, restore the board snapshot taken
   * before {@link AutorouteEngine#autoroute_connection} so rip-up victims removed during
   * routing are not left torn up.
   */
  private AutorouteAttemptResult applyStrictDrcAfterRoute(int p_route_net_no, int p_max_item_id_before,
      byte[] p_board_snapshot_before_route) {
    if (!this.settings.isStrictDrc()) {
      return null;
    }
    AutorouteAttemptResult rejection = enforceStrictDrc(board, p_route_net_no, p_max_item_id_before);
    if (rejection != null && p_board_snapshot_before_route != null) {
      this.board = (RoutingBoard) BasicBoard.deserialize(p_board_snapshot_before_route);
    }
    return rejection;
  }

  /**
   * Strict-DRC enforcement: if any trace/via inserted by the connection that just routed
   * (item id above {@code p_max_item_id_before}) carries a clearance violation, rip the
   * whole set of new items and report the connection FAILED, so the pass counts it as not
   * routed and later passes (higher ripup costs) retry it. Returns null when the connection
   * is clean and may be kept.
   */
  static AutorouteAttemptResult enforceStrictDrc(app.freerouting.board.RoutingBoard board,
      int p_route_net_no, int p_max_item_id_before) {
    List<Item> new_items = new ArrayList<>();
    boolean has_violation = false;
    for (Item curr_item : board.get_connectable_items(p_route_net_no)) {
      if (curr_item.get_id_no() <= p_max_item_id_before
          || !(curr_item instanceof Trace || curr_item instanceof app.freerouting.board.Via)) {
        continue;
      }
      new_items.add(curr_item);
      if (!has_violation && !curr_item.clearance_violations().isEmpty()) {
        has_violation = true;
      }
    }
    if (!has_violation) {
      return null;
    }
    board.remove_items(new_items);
    return new AutorouteAttemptResult(AutorouteAttemptState.FAILED,
        "strict_drc: connection ripped because " + new_items.size()
            + " new item(s) included clearance violations");
  }

  /**
   * Returns the airline of the current autorouted connection or null, if no such
   * airline exists
   */
  public FloatLine get_air_line() {
    if (this.air_line == null) {
      return null;
    }
    if (this.air_line.a == null || this.air_line.b == null) {
      return null;
    }
    return this.air_line;
  }

  // Calculates the shortest distance between two sets of items, specifically
  // between Pin and Via items (pins and vias are connectable DrillItems)
  private void calc_airline(Collection<Item> p_from_items, Collection<Item> p_to_items) {
    FloatPoint from_corner = null;
    FloatPoint to_corner = null;
    double min_distance = Double.MAX_VALUE;
    for (Item curr_from_item : p_from_items) {
      if (!(curr_from_item instanceof DrillItem)) {
        continue;
      }
      FloatPoint curr_from_corner = ((DrillItem) curr_from_item).get_center().to_float();

      for (Item curr_to_item : p_to_items) {
        if (!(curr_to_item instanceof DrillItem)) {
          continue;
        }
        FloatPoint curr_to_corner = ((DrillItem) curr_to_item).get_center().to_float();
        double curr_distance = curr_from_corner.distance_square(curr_to_corner);
        if (curr_distance < min_distance) {
          min_distance = curr_distance;
          from_corner = curr_from_corner;
          to_corner = curr_to_corner;
        }
      }
    }
    this.air_line = new FloatLine(from_corner, to_corner);
  }

  /**
   * Finds the nearest point on a trace to the given point
   */
  private FloatPoint nearest_point_on_trace(PolylineTrace p_trace, FloatPoint p_point) {
    double min_distance = Double.MAX_VALUE;
    FloatPoint nearest_point = null;

    // Get endpoints
    FloatPoint first_corner = p_trace
        .first_corner()
        .to_float();
    FloatPoint last_corner = p_trace
        .last_corner()
        .to_float();

    // Check distance to endpoints first
    double distance_to_first = p_point.distance(first_corner);
    double distance_to_last = p_point.distance(last_corner);

    if (distance_to_first < min_distance) {
      min_distance = distance_to_first;
      nearest_point = first_corner;
    }

    if (distance_to_last < min_distance) {
      min_distance = distance_to_last;
      nearest_point = last_corner;
    }

    // Check distances to line segments
    for (int i = 0; i < p_trace.corner_count() - 1; i++) {
      FloatPoint segment_start = p_trace
          .polyline()
          .corner_approx(i);
      FloatPoint segment_end = p_trace
          .polyline()
          .corner_approx(i + 1);
      FloatLine segment = new FloatLine(segment_start, segment_end);

      FloatPoint projection = segment.perpendicular_projection(p_point);
      if (projection.is_contained_in_box(segment_start, segment_end, 0.01)) {
        double distance = p_point.distance(projection);
        if (distance < min_distance) {
          min_distance = distance;
          nearest_point = projection;
        }
      }
    }

    return nearest_point;
  }

  /**
   * Finds the closest points between two traces
   *
   * @return an array with two FloatPoints: [point_on_first_trace,
   *         point_on_second_trace]
   */
  private FloatPoint[] find_closest_points_between_traces(PolylineTrace p_first_trace, PolylineTrace p_second_trace) {
    double min_distance = Double.MAX_VALUE;
    FloatPoint[] result = new FloatPoint[2];

    // Check endpoints to endpoints
    FloatPoint first_trace_start = p_first_trace
        .first_corner()
        .to_float();
    FloatPoint first_trace_end = p_first_trace
        .last_corner()
        .to_float();
    FloatPoint second_trace_start = p_second_trace
        .first_corner()
        .to_float();
    FloatPoint second_trace_end = p_second_trace
        .last_corner()
        .to_float();

    // Check all endpoint combinations
    double distance = first_trace_start.distance(second_trace_start);
    if (distance < min_distance) {
      min_distance = distance;
      result[0] = first_trace_start;
      result[1] = second_trace_start;
    }

    distance = first_trace_start.distance(second_trace_end);
    if (distance < min_distance) {
      min_distance = distance;
      result[0] = first_trace_start;
      result[1] = second_trace_end;
    }

    distance = first_trace_end.distance(second_trace_start);
    if (distance < min_distance) {
      min_distance = distance;
      result[0] = first_trace_end;
      result[1] = second_trace_start;
    }

    distance = first_trace_end.distance(second_trace_end);
    if (distance < min_distance) {
      min_distance = distance;
      result[0] = first_trace_end;
      result[1] = second_trace_end;
    }

    // Check all segment combinations for closest points
    for (int i = 0; i < p_first_trace.corner_count() - 1; i++) {
      FloatPoint first_segment_start = p_first_trace
          .polyline()
          .corner_approx(i);
      FloatPoint first_segment_end = p_first_trace
          .polyline()
          .corner_approx(i + 1);
      FloatLine first_segment = new FloatLine(first_segment_start, first_segment_end);

      for (int j = 0; j < p_second_trace.corner_count() - 1; j++) {
        FloatPoint second_segment_start = p_second_trace
            .polyline()
            .corner_approx(j);
        FloatPoint second_segment_end = p_second_trace
            .polyline()
            .corner_approx(j + 1);
        FloatLine second_segment = new FloatLine(second_segment_start, second_segment_end);

        // Find closest points between these two line segments
        FloatPoint point_on_first = first_segment.nearest_segment_point(second_segment_start);
        FloatPoint point_on_second = second_segment.perpendicular_projection(point_on_first);

        // Check if projection is on the segment
        if (!point_on_second.is_contained_in_box(second_segment_start, second_segment_end, 0.01)) {
          // If not, use the nearest endpoint
          double dist_to_start = point_on_first.distance(second_segment_start);
          double dist_to_end = point_on_first.distance(second_segment_end);
          point_on_second = dist_to_start < dist_to_end ? second_segment_start : second_segment_end;
        }

        // Recalculate the point on first segment based on the point on second segment
        point_on_first = first_segment.nearest_segment_point(point_on_second);

        distance = point_on_first.distance(point_on_second);
        if (distance < min_distance) {
          min_distance = distance;
          result[0] = point_on_first;
          result[1] = point_on_second;
        }
      }
    }

    return result;
  }

  /**
   * Return an uppercase one-letter, two-letter or three-letter string based on
   * the thread index (0 = A, 1 = B, 2 = C, ..., 26 = AA, 27 = AB, ...).
   *
   * @param threadIndex
   * @return
   */
  private String ThreadIndexToLetter(int threadIndex) {
    if (threadIndex < 0) {
      return "";
    }
    if (threadIndex < 26) {
      return String.valueOf((char) ('A' + threadIndex));
    } else if (threadIndex < 26 * 26) {
      int firstLetterIndex = threadIndex / 26;
      int secondLetterIndex = threadIndex % 26;
      return String.valueOf((char) ('A' + firstLetterIndex)) + (char) ('A' + secondLetterIndex);
    } else {
      int firstLetterIndex = threadIndex / (26 * 26);
      int secondLetterIndex = (threadIndex / 26) % 26;
      int thirdLetterIndex = threadIndex % 26;
      return String.valueOf((char) ('A' + firstLetterIndex)) + (char) ('A' + secondLetterIndex)
          + (char) ('A' + thirdLetterIndex);
    }
  }

  /**
   * Calculates the airline distance for an item to be routed.
   * Returns the shortest distance from the item to any item in its incomplete
   * connections.
   *
   * @param p_item The item to calculate distance for
   * @return The shortest airline distance, or Double.MAX_VALUE if no connections
   *         exist
   */
  private double calculateItemDistance(Item p_item) {
    if (p_item.net_count() == 0) {
      return Double.MAX_VALUE;
    }

    // Get the first net number (items typically have one net)
    int net_no = p_item.get_net_no(0);

    // Get incomplete items for this net
    Set<Item> unconnected_set = p_item.get_unconnected_set(net_no);
    Set<Item> connected_set = p_item.get_connected_set(net_no);

    if (unconnected_set.isEmpty()) {
      return 0; // Already connected, prioritize
    }

    // Calculate minimum distance from connected items to unconnected items
    return calculateMinDistance(connected_set.isEmpty() ? Set.of(p_item) : connected_set, unconnected_set);
  }

  /**
   * Helper method to calculate the minimum distance between two sets of items.
   */
  private double calculateMinDistance(Collection<Item> p_from_items, Collection<Item> p_to_items) {
    double min_distance = Double.MAX_VALUE;

    for (Item from_item : p_from_items) {
      FloatPoint from_point = getItemReferencePoint(from_item);
      if (from_point == null)
        continue;

      for (Item to_item : p_to_items) {
        FloatPoint to_point = getItemReferencePoint(to_item);
        if (to_point == null)
          continue;

        double distance = from_point.distance(to_point);
        if (distance < min_distance) {
          min_distance = distance;
        }
      }
    }

    return min_distance;
  }

  /**
   * Gets a representative point for an item (center for DrillItems, midpoint for
   * traces).
   */
  private FloatPoint getItemReferencePoint(Item p_item) {
    if (p_item instanceof DrillItem drillItem) {
      return drillItem.get_center().to_float();
    } else if (p_item instanceof PolylineTrace trace) {
      // Use the midpoint of the trace as a reference
      FloatPoint first = trace.first_corner().to_float();
      FloatPoint last = trace.last_corner().to_float();
      return new FloatPoint((first.x + last.x) / 2, (first.y + last.y) / 2);
    }
    return null;
  }

  private int calculateIncompleteCount(RoutingBoard board) {
    DesignRulesChecker tempDrc = new DesignRulesChecker(board, null);
    tempDrc.calculateAllIncompletes();
    return tempDrc.getIncompleteCount();
  }
}