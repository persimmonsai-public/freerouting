package app.freerouting.interactive;

import app.freerouting.Freerouting;
import app.freerouting.board.Item;
import app.freerouting.board.RoutingBoard;
import app.freerouting.board.ShapeSearchTree;
import app.freerouting.core.RoutingJob;
import app.freerouting.core.RoutingJobState;
import app.freerouting.geometry.planar.IntBox;
import app.freerouting.geometry.planar.TileShape;
import app.freerouting.management.RoutingJobScheduler;
import app.freerouting.management.SessionManager;
import app.freerouting.settings.GlobalSettings;
import app.freerouting.settings.SettingsMerger;
import app.freerouting.settings.sources.DefaultSettings;
import app.freerouting.settings.sources.DsnFileSettings;
import app.freerouting.settings.sources.TestingSettings;
import app.freerouting.util.TextManager;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Feasibility gate for a maintained free-space partition (the "context-free, re-coarsening
 * decomposition" rework). Before any engine surgery, this measures the two quantities the
 * design lives or dies by, on the fixture board in its WORST state -- fully routed:
 *
 * <ol>
 *   <li><b>Cell count per layer.</b> A retained decomposition means every search traverses a
 *       board-wide graph instead of a lazily-built local one. The failed retention experiment
 *       showed what happens when that graph is huge (~50k rooms: 10x pops, 2.6x slower). If a
 *       clean partition of the routed board is in the low thousands of cells per layer, the
 *       graph is comparable to what hard searches already visit; if it is tens of thousands,
 *       the rework inherits the same blowup and dies here.
 *   <li><b>Build and update cost.</b> The partition must be maintained across commits. A full
 *       per-layer rebuild cost bounds the worst case; the per-slab locality of a vertical-slab
 *       decomposition bounds the incremental case.
 * </ol>
 *
 * <p>The decomposition measured is a vertical-slab trapezoidalization over the bounding boxes
 * of the compensated tree shapes (boxes over-approximate 45-degree shapes, so cell counts here
 * are an over-estimate -- conservative for the gate), followed by a maximal horizontal merge of
 * slab-adjacent cells with identical y-intervals, which is the corner-stitching-style
 * coarsening the real structure would use.
 */
class PartitionFeasibilityTest {

  private static final String FIXTURE = "Issue508-DAC2020_bm01.dsn";

  private RoutingJobScheduler scheduler;

  @BeforeEach
  void setUp() {
    Freerouting.globalSettings = new GlobalSettings();
    InteractiveSettings.resetForTesting();
    scheduler = RoutingJobScheduler.getInstance();
    synchronized (scheduler.jobs) {
      scheduler.jobs.clear();
    }
  }

  @Test
  @Timeout(value = 300, unit = TimeUnit.SECONDS)
  void measurePartitionOnRoutedBoard() {
    TestingSettings settings = new TestingSettings();
    settings.setMaxPasses(8);
    settings.setMaxItems(500);
    settings.setJobTimeoutString("00:03:00");
    RoutingJob job = createRoutingJob(FIXTURE, settings);
    job.routerSettings.maxThreads = 1;
    runRoutingJob(job);

    RoutingBoard board = job.board;
    ShapeSearchTree tree = board.search_tree_manager.get_default_tree();
    IntBox bounds = board.get_bounding_box();

    long totalRaw = 0;
    long totalMerged = 0;
    long totalBuildMs = 0;
    for (int layer = 0; layer < board.get_layer_count(); layer++) {
      if (!board.layer_structure.arr[layer].is_signal) {
        continue;
      }
      List<IntBox> obstacles = collectObstacleBoxes(board, tree, layer);

      long t0 = System.nanoTime();
      long[] counts = slabPartitionCounts(obstacles, bounds);
      long buildMs = (System.nanoTime() - t0) / 1_000_000;

      totalRaw += counts[0];
      totalMerged += counts[1];
      totalBuildMs += buildMs;
      System.out.println("[partition] layer=" + layer + " obstacles=" + obstacles.size()
          + " raw_cells=" + counts[0] + " merged_strips=" + counts[1] + " build_ms=" + buildMs);
    }
    System.out.println("[partition] TOTAL raw_cells=" + totalRaw + " merged_strips=" + totalMerged
        + " build_ms_all_layers=" + totalBuildMs);

    // Locality probe: cost of re-partitioning only the slabs a typical committed trace crosses.
    // A trace commit dirties the slabs its bounding box spans; everything else is untouched.
    List<IntBox> layer0 = collectObstacleBoxes(board, tree, 0);
    long t0 = System.nanoTime();
    int probes = 200;
    java.util.Random rnd = new java.util.Random(7);
    long touchedSlabs = 0;
    for (int i = 0; i < probes; i++) {
      int x1 = bounds.ll.x + rnd.nextInt(Math.max(1, bounds.ur.x - bounds.ll.x));
      int width = (bounds.ur.x - bounds.ll.x) / 10; // a trace spanning ~10% of the board width
      IntBox dirty = new IntBox(x1, bounds.ll.y, Math.min(bounds.ur.x, x1 + width), bounds.ur.y);
      List<IntBox> affected = new ArrayList<>();
      for (IntBox b : layer0) {
        if (b.intersects(dirty)) {
          affected.add(b);
        }
      }
      touchedSlabs += slabPartitionCounts(affected, dirty)[0];
    }
    long probeUs = (System.nanoTime() - t0) / 1_000 / probes;
    System.out.println("[partition] local_update_probe: avg_us_per_10pct_board_repartition="
        + probeUs + " avg_cells_touched=" + (touchedSlabs / probes));
  }

  /**
   * Bounding boxes of every compensated tree shape of every item on the layer.
   */
  private List<IntBox> collectObstacleBoxes(RoutingBoard board, ShapeSearchTree tree, int layer) {
    List<IntBox> result = new ArrayList<>();
    for (Item item : board.get_items()) {
      int shapeCount = item.tree_shape_count(tree);
      for (int i = 0; i < shapeCount; i++) {
        if (item.shape_layer(i) != layer) {
          continue;
        }
        TileShape shape = item.get_tree_shape(tree, i);
        if (shape != null && !shape.is_empty()) {
          result.add(shape.bounding_box());
        }
      }
    }
    return result;
  }

  /**
   * Vertical-slab decomposition of the free space (bounds minus obstacle boxes).
   *
   * @return {raw cell count, cell count after maximal horizontal merge}
   */
  private long[] slabPartitionCounts(List<IntBox> obstacles, IntBox bounds) {
    // Slab edges: every distinct obstacle x-boundary inside the bounds.
    java.util.TreeSet<Integer> xs = new java.util.TreeSet<>();
    xs.add(bounds.ll.x);
    xs.add(bounds.ur.x);
    for (IntBox b : obstacles) {
      if (b.ur.x > bounds.ll.x && b.ll.x < bounds.ur.x) {
        xs.add(Math.max(b.ll.x, bounds.ll.x));
        xs.add(Math.min(b.ur.x, bounds.ur.x));
      }
    }
    Integer[] edges = xs.toArray(new Integer[0]);

    long raw = 0;
    long merged = 0;
    // y-interval lists of the previous slab, for the horizontal merge count.
    List<int[]> prevGaps = null;
    // Sort obstacles by ll.x once; sweep a working set per slab.
    obstacles.sort((a, b) -> Integer.compare(a.ll.x, b.ll.x));
    for (int s = 0; s + 1 < edges.length; s++) {
      int slabFrom = edges[s];
      int slabTo = edges[s + 1];
      if (slabTo <= slabFrom) {
        continue;
      }
      // Obstacles overlapping this slab.
      List<int[]> yIntervals = new ArrayList<>();
      for (IntBox b : obstacles) {
        if (b.ll.x < slabTo && b.ur.x > slabFrom) {
          yIntervals.add(new int[]{Math.max(b.ll.y, bounds.ll.y), Math.min(b.ur.y, bounds.ur.y)});
        }
      }
      yIntervals.sort((a, b) -> Integer.compare(a[0], b[0]));
      // Free gaps = complement of merged y-intervals.
      List<int[]> gaps = new ArrayList<>();
      int cursor = bounds.ll.y;
      for (int[] iv : yIntervals) {
        if (iv[0] > cursor) {
          gaps.add(new int[]{cursor, iv[0]});
        }
        cursor = Math.max(cursor, iv[1]);
      }
      if (cursor < bounds.ur.y) {
        gaps.add(new int[]{cursor, bounds.ur.y});
      }
      raw += gaps.size();
      // Horizontal merge: a gap continues the strip from the previous slab if an identical
      // y-interval existed there; only NEW strips count.
      for (int[] g : gaps) {
        boolean continuation = false;
        if (prevGaps != null) {
          for (int[] pg : prevGaps) {
            if (pg[0] == g[0] && pg[1] == g[1]) {
              continuation = true;
              break;
            }
          }
        }
        if (!continuation) {
          ++merged;
        }
      }
      prevGaps = gaps;
    }
    return new long[]{raw, merged};
  }

  // ── Harness (mirrors RoutingProfileTest) ───────────────────────────────────

  private RoutingJob createRoutingJob(String filename, TestingSettings testingSettings) {
    UUID userId = UUID.randomUUID();
    var session = SessionManager.getInstance().createSession(userId, "test/1.0");
    RoutingJob job = new RoutingJob(session.id);

    Path testDirectory = Path.of(".").toAbsolutePath();
    File testFile = Path.of(testDirectory.toString(), "fixtures", filename).toFile();
    while (!testFile.exists()) {
      testDirectory = testDirectory.getParent();
      if (testDirectory == null) {
        break;
      }
      testFile = Path.of(testDirectory.toString(), "fixtures", filename).toFile();
    }

    try {
      job.setInput(testFile);
      SettingsMerger merger = new SettingsMerger(new DefaultSettings(),
          new DsnFileSettings(job.input.getData(), job.input.getFilename()));
      merger.addOrReplaceSources(testingSettings);
      Freerouting.globalSettings.settingsMergerProtype.addOrReplaceSources(testingSettings);
      job.routerSettings = merger.merge();
    } catch (IOException e) {
      throw new RuntimeException(testFile + " not found.", e);
    }
    return job;
  }

  private void runRoutingJob(RoutingJob job) {
    scheduler.enqueueJob(job);
    job.state = RoutingJobState.READY_TO_START;
    long startTime = System.currentTimeMillis();
    long timeoutInMillis = TextManager.parseTimespanString(job.routerSettings.jobTimeoutString) * 1000;
    while ((job.state != RoutingJobState.COMPLETED) && (job.state != RoutingJobState.CANCELLED)
        && (job.state != RoutingJobState.TERMINATED)) {
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        throw new RuntimeException(e);
      }
      if (System.currentTimeMillis() - startTime > timeoutInMillis) {
        if (job.thread != null) {
          job.thread.requestStop();
        }
        break;
      }
    }
  }
}
