package app.freerouting.interactive;

import app.freerouting.Freerouting;
import app.freerouting.board.Item;
import app.freerouting.board.Pin;
import app.freerouting.board.RoutingBoard;
import app.freerouting.core.RoutingJob;
import app.freerouting.core.RoutingJobState;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntBox;
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
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Measurement probe for the planned BGA-escape mode: detects array-arranged pad fields
 * (2D grids = BGA-like, 1-2 line groups = peripheral/connector-like) per component and
 * reports the quantities an escape optimizer would key on -- grid dimensions, pitch and its
 * regularity, fill ratio, pad size, SMD vs through-hole, and the channel capacity between
 * adjacent pads at the board's default trace width and clearance.
 */
class ArrayPadFieldProbeTest {

  private static final String FIXTURE = System.getProperty("fr.fixture", "Issue508-DAC2020_bm01.dsn");

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
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void detectArrayPadFields() {
    TestingSettings settings = new TestingSettings();
    settings.setMaxPasses(1);
    settings.setMaxItems(1); // load the board; route (nearly) nothing
    settings.setJobTimeoutString("00:01:30");
    RoutingJob job = createRoutingJob(FIXTURE, settings);
    job.routerSettings.maxThreads = 1;
    runRoutingJob(job);

    RoutingBoard board = job.board;
    Map<Integer, List<Pin>> pins_by_component = new HashMap<>();
    for (Item item : board.get_items()) {
      if (item instanceof Pin pin) {
        pins_by_component.computeIfAbsent(pin.get_component_no(), k -> new ArrayList<>()).add(pin);
      }
    }

    for (Map.Entry<Integer, List<Pin>> entry : pins_by_component.entrySet()) {
      List<Pin> pins = entry.getValue();
      if (pins.size() < 8) {
        continue;
      }
      String name = board.components.get(entry.getKey()).name;
      List<FloatPoint> centers = new ArrayList<>(pins.size());
      for (Pin pin : pins) {
        centers.add(pin.get_center().to_float());
      }
      // Cluster distinct coordinates with a tolerance of 1/4 of the minimum spacing.
      List<Double> xs = cluster(centers.stream().map(p -> p.x).sorted().toList());
      List<Double> ys = cluster(centers.stream().map(p -> p.y).sorted().toList());
      double pitch_x = median_delta(xs);
      double pitch_y = median_delta(ys);
      double regularity_x = delta_regularity(xs);
      double regularity_y = delta_regularity(ys);
      double fill = pins.size() / (double) Math.max(1, xs.size() * ys.size());
      Pin sample = pins.get(0);
      IntBox pad_box = sample.get_tile_shape_on_layer(sample.first_layer()).bounding_box();
      boolean smd = sample.first_layer() == sample.last_layer();
      String kind;
      if (xs.size() >= 4 && ys.size() >= 4 && fill >= 0.45) {
        kind = "BGA_GRID";
      } else if (xs.size() <= 2 || ys.size() <= 2) {
        kind = "PERIPHERAL_LINES";
      } else if (fill < 0.45 && xs.size() >= 4 && ys.size() >= 4) {
        kind = "PERIMETER_RING"; // grid coordinates but hollow middle (QFP-style ring)
      } else {
        kind = "IRREGULAR";
      }
      System.out.println("[pad-array] component=" + name
          + " pins=" + pins.size()
          + " kind=" + kind
          + " grid=" + xs.size() + "x" + ys.size()
          + " fill=" + String.format("%.2f", fill)
          + " pitch=(" + Math.round(pitch_x) + "," + Math.round(pitch_y) + ")"
          + " regularity=(" + String.format("%.2f", regularity_x) + "," + String.format("%.2f", regularity_y) + ")"
          + " pad=" + (pad_box.ur.x - pad_box.ll.x) + "x" + (pad_box.ur.y - pad_box.ll.y)
          + (smd ? " SMD layer=" + sample.first_layer() : " TH"));
    }

    // Violations already present on the (nearly) unrouted board: constant late-pass
    // violation counts on the 8-layer fixture suggested they might be pre-existing.
    int raw_violations = 0;
    for (Item item : board.get_items()) {
      raw_violations += item.clearance_violation_count();
    }
    System.out.println("[pad-array] raw_board_violation_count=" + raw_violations);
    // Escape-failure map: SMD pins with a net but no contacts after the fanout stage,
    // grouped by component -- tells Phase 1 exactly where escapes fail.
    Map<String, Integer> unescaped_by_component = new java.util.TreeMap<>();
    for (Item item : board.get_items()) {
      if (item instanceof Pin pin && pin.first_layer() == pin.last_layer()
          && pin.net_count() > 0 && pin.get_normal_contacts().isEmpty()) {
        String comp = board.components.get(pin.get_component_no()).name;
        unescaped_by_component.merge(comp, 1, Integer::sum);
      }
    }
    System.out.println("[pad-array] unescaped_smd_by_component=" + unescaped_by_component);
    // Board-level context an escape planner needs.
    int default_half_width = board.rules.get_default_net_class().get_trace_half_width(0);
    System.out.println("[pad-array] default_trace_half_width=" + default_half_width
        + " layers=" + board.get_layer_count());
  }

  /**
   * Clusters ascending values whose gaps are below 1/4 of the largest common spacing into
   * single coordinates (pad rows are never perfectly aligned in DSN data).
   */
  private static List<Double> cluster(List<Double> p_sorted) {
    if (p_sorted.isEmpty()) {
      return List.of();
    }
    double min_gap = Double.MAX_VALUE;
    for (int i = 1; i < p_sorted.size(); i++) {
      double gap = p_sorted.get(i) - p_sorted.get(i - 1);
      if (gap > 1) {
        min_gap = Math.min(min_gap, gap);
      }
    }
    double tolerance = min_gap == Double.MAX_VALUE ? 1 : min_gap / 4;
    List<Double> result = new ArrayList<>();
    double cluster_sum = p_sorted.get(0);
    int cluster_count = 1;
    for (int i = 1; i < p_sorted.size(); i++) {
      if (p_sorted.get(i) - p_sorted.get(i - 1) <= tolerance) {
        cluster_sum += p_sorted.get(i);
        ++cluster_count;
      } else {
        result.add(cluster_sum / cluster_count);
        cluster_sum = p_sorted.get(i);
        cluster_count = 1;
      }
    }
    result.add(cluster_sum / cluster_count);
    return result;
  }

  private static double median_delta(List<Double> p_coords) {
    if (p_coords.size() < 2) {
      return 0;
    }
    List<Double> deltas = new ArrayList<>();
    for (int i = 1; i < p_coords.size(); i++) {
      deltas.add(p_coords.get(i) - p_coords.get(i - 1));
    }
    deltas.sort(Double::compare);
    return deltas.get(deltas.size() / 2);
  }

  /**
   * Fraction of consecutive coordinate deltas within 10% of the median delta -- 1.0 means a
   * perfectly regular grid axis.
   */
  private static double delta_regularity(List<Double> p_coords) {
    if (p_coords.size() < 3) {
      return 1.0;
    }
    double median = median_delta(p_coords);
    if (median <= 0) {
      return 0;
    }
    int regular = 0;
    int total = p_coords.size() - 1;
    for (int i = 1; i < p_coords.size(); i++) {
      double delta = p_coords.get(i) - p_coords.get(i - 1);
      if (Math.abs(delta - median) <= median * 0.1) {
        ++regular;
      }
    }
    return regular / (double) total;
  }

  // ── Harness (mirrors PartitionFeasibilityTest) ─────────────────────────────

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
