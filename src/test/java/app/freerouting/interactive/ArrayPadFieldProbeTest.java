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

    java.util.Set<Integer> unescaped_ids = new java.util.HashSet<>();
    for (Item item : board.get_items()) {
      if (item instanceof Pin pin && pin.first_layer() == pin.last_layer()
          && pin.net_count() > 0 && pin.get_normal_contacts().isEmpty()) {
        unescaped_ids.add(pin.get_id_no());
      }
    }
    int via_radius = Integer.MAX_VALUE; // feasibility uses the SMALLEST available via
    int via_from_layer = 0;
    int via_to_layer = board.get_layer_count() - 1;
    for (int v = 0; v < board.rules.via_infos.count(); v++) {
      var ps = board.rules.via_infos.get(v).get_padstack();
      IntBox vb = ps.get_shape(ps.from_layer()).bounding_box();
      int r = (vb.ur.x - vb.ll.x) / 2;
      if (r < via_radius) {
        via_radius = r;
        via_from_layer = ps.from_layer();
        via_to_layer = ps.to_layer();
      }
    }
    int clearance = board.rules.clearance_matrix.get_value(1, 1, 0, false);
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
      // Escape-feasibility classification of this component's unescaped pins:
      // interior grid balls can only host a via at the grid diagonal; boundary balls can
      // always dogbone outward into open space (-> algorithm-missed if unescaped).
      int rules_impossible = 0;
      int algorithm_missed = 0;
      double diagonal_reach = Math.hypot(pitch_x, pitch_y) / 2;
      double x_lo = xs.isEmpty() ? 0 : xs.get(0);
      double x_hi = xs.isEmpty() ? 0 : xs.get(xs.size() - 1);
      double y_lo = ys.isEmpty() ? 0 : ys.get(0);
      double y_hi = ys.isEmpty() ? 0 : ys.get(ys.size() - 1);
      for (Pin pin : pins) {
        if (!unescaped_ids.contains(pin.get_id_no())) {
          continue;
        }
        FloatPoint c = pin.get_center().to_float();
        double tol = Math.max(1, Math.min(pitch_x, pitch_y) / 4);
        boolean interior = c.x > x_lo + tol && c.x < x_hi - tol && c.y > y_lo + tol && c.y < y_hi - tol;
        IntBox own_pad = pin.get_tile_shape_on_layer(pin.first_layer()).bounding_box();
        int pad_half = Math.min(own_pad.ur.x - own_pad.ll.x, own_pad.ur.y - own_pad.ll.y) / 2;
        double required = via_radius + clearance + pad_half;
        if (interior && diagonal_reach < required) {
          ++rules_impossible;
        } else {
          ++algorithm_missed;
          System.out.println("[pad-array] missed pin=" + pin.get_id_no() + " comp=" + name
              + " center=(" + Math.round(c.x) + "," + Math.round(c.y) + ")"
              + " interior=" + interior + " net=" + (pin.net_count() > 0 ? pin.get_net_no(0) : -1));
        }
      }
      if (rules_impossible + algorithm_missed > 0) {
        System.out.println("[pad-array] escape-feasibility component=" + name
            + " rules_impossible=" + rules_impossible
            + " algorithm_missed=" + algorithm_missed
            + " (diagonal_reach=" + Math.round(diagonal_reach)
            + " vs required=via_r+clr+pad_half)");
      }
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
    // Via rules: is the U1 inner-ball escape geometrically possible under this design?
    for (int v = 0; v < board.rules.via_infos.count(); v++) {
      var info = board.rules.via_infos.get(v);
      var padstack = info.get_padstack();
      IntBox via_box = padstack.get_shape(padstack.from_layer()) == null ? null
          : padstack.get_shape(padstack.from_layer()).bounding_box();
      System.out.println("[pad-array] via name=" + info.get_name()
          + " layers=" + padstack.from_layer() + ".." + padstack.to_layer()
          + " size=" + (via_box == null ? "?" : (via_box.ur.x - via_box.ll.x) + "x" + (via_box.ur.y - via_box.ll.y))
          + " attach_smd=" + info.attach_smd_allowed());
    }
    System.out.println("[pad-array] default_clearance=" + board.rules.clearance_matrix.get_value(1, 1, 0, false));
    // Candidate-spot legality scan for the two algorithm-missed U1 balls: does ANY legal
    // via position exist within dogbone reach? Binary outcome: fanout bug vs rules-impossible.
    var scan_tree = board.search_tree_manager.get_default_tree();
    Map<String, Integer> scan_verdicts = new java.util.TreeMap<>();
    int dbg_diag_logged = 0;
    for (Item item : board.get_items()) {
      if (!(item instanceof Pin pin) || !unescaped_ids.contains(pin.get_id_no())) {
        continue;
      }
      FloatPoint c = pin.get_center().to_float();
      int legal_spots = 0;
      int legal_spots_pre = 0;
      double nearest = Double.MAX_VALUE;
      for (int dx = -13500; dx <= 13500; dx += 1500) {
        for (int dy = -13500; dy <= 13500; dy += 1500) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          int cx = (int) c.x + dx;
          int cy = (int) c.y + dy;
          IntBox via_box = new IntBox(cx - via_radius, cy - via_radius, cx + via_radius, cy + via_radius);
          boolean legal_post = true;
          boolean legal_pre = true; // ignoring fanout-added traces/vias: the pre-fanout board
          for (int layer = via_from_layer; layer <= via_to_layer && legal_pre; layer++) {
            for (var entry : scan_tree.overlapping_tree_entries_with_clearance(
                via_box, layer, new int[0], 1)) {
              if (entry.object instanceof Item blocking && !blocking.shares_net(pin)) {
                if (blocking instanceof app.freerouting.board.ConductionArea pour
                    && !pour.get_is_obstacle()) {
                  continue; // pours reflow around new copper; the router treats them as passable
                }
                legal_post = false;
                if (!(blocking instanceof app.freerouting.board.Trace)
                    && !(blocking instanceof app.freerouting.board.Via)) {
                  legal_pre = false;
                  break;
                }
              }
            }
          }
          if (legal_post) {
            ++legal_spots;
            nearest = Math.min(nearest, Math.hypot(dx, dy));
          }
          if (legal_pre) {
            ++legal_spots_pre;
          }
        }
      }
      String comp_name = board.components.get(pin.get_component_no()).name;
      if (comp_name.equals("U1") && dbg_diag_logged < 1) {
        ++dbg_diag_logged;
        for (int[] d : new int[][]{{4500, 4500}, {-4500, 4500}, {4500, -4500}, {-4500, -4500}}) {
          int cx = (int) c.x + d[0];
          int cy = (int) c.y + d[1];
          IntBox via_box = new IntBox(cx - via_radius, cy - via_radius, cx + via_radius, cy + via_radius);
          for (int layer = via_from_layer; layer <= via_to_layer; layer++) {
            for (var entry : scan_tree.overlapping_tree_entries_with_clearance(via_box, layer, new int[0], 1)) {
              if (entry.object instanceof Item blocking && !blocking.shares_net(pin)) {
                System.out.println("[pad-array] diag-blocker pin=" + pin.get_id_no()
                    + " cand=(" + d[0] + "," + d[1] + ") layer=" + layer
                    + " item=" + blocking.getClass().getSimpleName() + "#" + blocking.get_id_no()
                    + " cl_class=" + blocking.clearance_class_no());
              }
            }
          }
        }
      }
      String verdict = legal_spots > 0 ? "ESCAPABLE_NOW"
          : legal_spots_pre > 0 ? "ORDERING_VICTIM" : "RULES_IMPOSSIBLE";
      scan_verdicts.merge(comp_name + ":" + verdict, 1, Integer::sum);
    }
    System.out.println("[pad-array] spot-scan verdicts=" + scan_verdicts);
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
