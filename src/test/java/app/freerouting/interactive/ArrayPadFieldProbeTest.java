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
    Freerouting.globalSettings.featureFlags.escapePlanner = Boolean.getBoolean("fr.escape");
    Freerouting.globalSettings.featureFlags.reflowablePours = Boolean.getBoolean("fr.reflow");
    Freerouting.globalSettings.featureFlags.negotiatedRouter = Boolean.getBoolean("fr.negotiate");
    scheduler = RoutingJobScheduler.getInstance();
    synchronized (scheduler.jobs) {
      scheduler.jobs.clear();
    }
  }

  @Test
  @Timeout(value = 1500, unit = TimeUnit.SECONDS)
  void detectArrayPadFields() {
    TestingSettings settings = new TestingSettings();
    // Default: load the board, route (nearly) nothing -- the structural reports. With
    // -Dfr.probe_items/-Dfr.probe_passes/-Dfr.timeout the probe routes first, which makes
    // the ROUTED-board reports below (return-path, length matching) meaningful.
    settings.setMaxPasses(Integer.getInteger("fr.probe_passes", 1));
    settings.setMaxItems(Integer.getInteger("fr.probe_items", 1));
    settings.setJobTimeoutString(System.getProperty("fr.timeout", "00:01:30"));
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
      // The v1 interior/boundary escape-feasibility heuristic that lived here is RETIRED
      // (it over-counted 48 of 57 rules-impossible verdicts); the spatial spot scan below
      // is the validated feasibility test, and app.freerouting.autoroute.EscapeFeasibility
      // is its production graduation (asserted against the inline scan below).
    }

    // Violations already present on the (nearly) unrouted board: constant late-pass
    // violation counts on the 8-layer fixture suggested they might be pre-existing.
    int raw_violations = 0;
    for (Item item : board.get_items()) {
      raw_violations += item.clearance_violation_count();
    }
    System.out.println("[pad-array] raw_board_violation_count=" + raw_violations);
    java.util.TreeSet<String> violation_pairs = new java.util.TreeSet<>();
    for (Item item : board.get_items()) {
      for (var v : item.clearance_violations()) {
        int a = item.get_id_no();
        int b = v.second_item.get_id_no();
        violation_pairs.add((Math.min(a, b)) + "&" + (Math.max(a, b))
            + ":" + item.getClass().getSimpleName() + "/" + v.second_item.getClass().getSimpleName());
      }
    }
    System.out.println("[pad-array] pair_count=" + violation_pairs.size());
    for (String pair : violation_pairs) {
      System.out.println("[pair] " + pair);
    }
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
    Map<String, Integer> production_verdicts = new java.util.TreeMap<>();
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
      // The graduated production classifier must agree with the inline scan PER PIN
      // (the ReturnPathReport graduation pattern: probe asserts production against its
      // own independent computation).
      var production_verdict = app.freerouting.autoroute.EscapeFeasibility.classify(board, pin);
      org.junit.jupiter.api.Assertions.assertEquals(verdict, production_verdict.name(),
          "production EscapeFeasibility verdict for pin " + pin.get_id_no()
              + " (" + comp_name + ")");
      production_verdicts.merge("TOTAL:" + production_verdict.name(), 1, Integer::sum);
    }
    System.out.println("[pad-array] spot-scan verdicts=" + scan_verdicts);
    System.out.println("[pad-array] production classifier agrees per pin; totals="
        + production_verdicts);
    // Phase 5 reporting: differential pairs by net-name convention, planes per layer.
    java.util.Map<String, String> diff_pairs = new java.util.TreeMap<>();
    for (int n = 1; n <= board.rules.nets.max_net_no(); n++) {
      var net = board.rules.nets.get(n);
      if (net == null) {
        continue;
      }
      String name = net.name;
      String partner = null;
      if (name.endsWith("_P")) {
        partner = name.substring(0, name.length() - 2) + "_N";
      } else if (name.endsWith("+")) {
        partner = name.substring(0, name.length() - 1) + "-";
      } else if (name.endsWith("P") && name.length() > 1 && Character.isDigit(name.charAt(name.length() - 2))) {
        partner = name.substring(0, name.length() - 1) + "N";
      }
      if (partner != null && board.rules.nets.get(partner) != null) {
        diff_pairs.put(name, partner);
      }
    }
    System.out.println("[phase5] diff_pairs=" + diff_pairs);
    java.util.Map<Integer, Integer> planes_per_layer = new java.util.TreeMap<>();
    int obstacle_planes = 0;
    for (Item item : board.get_items()) {
      if (item instanceof app.freerouting.board.ConductionArea pour) {
        planes_per_layer.merge(pour.first_layer(), 1, Integer::sum);
        if (pour.get_is_obstacle()) {
          ++obstacle_planes;
        }
      }
    }
    System.out.println("[phase5] planes_per_layer=" + planes_per_layer
        + " obstacle_planes=" + obstacle_planes);
    // Phase 5 item 4: return-path report. Every signal via is a layer-change point where
    // the return current must also change reference planes; a healthy design has a same-net
    // (series/stitching) via or a reference-plane via nearby. Report layer changes whose
    // nearest such via is farther than the threshold (default 30000 units = 3 mm at the
    // fixtures' 0.1 um unit).
    int return_threshold = Integer.getInteger("fr.returnpath_threshold", 30000);
    java.util.Set<Integer> plane_nets = new java.util.TreeSet<>();
    for (Item item : board.get_items()) {
      if (item instanceof app.freerouting.board.ConductionArea pour) {
        for (int n = 0; n < pour.net_count(); n++) {
          plane_nets.add(pour.get_net_no(n));
        }
      }
    }
    List<app.freerouting.board.Via> signal_vias = new ArrayList<>();
    List<app.freerouting.board.Via> return_vias = new ArrayList<>();
    for (Item item : board.get_items()) {
      if (item instanceof app.freerouting.board.Via via && via.net_count() > 0) {
        if (plane_nets.contains(via.get_net_no(0))) {
          return_vias.add(via);
        } else {
          signal_vias.add(via);
        }
      }
    }
    int return_offenders = 0;
    java.util.List<String> worst_offenders = new ArrayList<>();
    java.util.List<double[]> offender_distances = new ArrayList<>();
    for (app.freerouting.board.Via via : signal_vias) {
      var center = via.get_center().to_float();
      double nearest = Double.MAX_VALUE;
      for (app.freerouting.board.Via other : signal_vias) {
        if (other != via && other.get_net_no(0) == via.get_net_no(0)) {
          var oc = other.get_center().to_float();
          nearest = Math.min(nearest, Math.hypot(oc.x - center.x, oc.y - center.y));
        }
      }
      for (app.freerouting.board.Via other : return_vias) {
        var oc = other.get_center().to_float();
        nearest = Math.min(nearest, Math.hypot(oc.x - center.x, oc.y - center.y));
      }
      if (nearest > return_threshold) {
        ++return_offenders;
        offender_distances.add(new double[]{nearest, center.x, center.y, via.get_net_no(0)});
      }
    }
    offender_distances.sort((a, b) -> Double.compare(b[0], a[0]));
    for (int k = 0; k < Math.min(10, offender_distances.size()); k++) {
      double[] o = offender_distances.get(k);
      worst_offenders.add(board.rules.nets.get((int) o[3]).name + "@(" + (int) o[1] + ","
          + (int) o[2] + "):" + (o[0] == Double.MAX_VALUE ? "none" : String.valueOf((int) o[0])));
    }
    System.out.println("[phase5-return] signal_vias=" + signal_vias.size()
        + " reference_vias=" + return_vias.size() + " plane_nets=" + plane_nets.size()
        + " threshold=" + return_threshold + " offenders=" + return_offenders
        + " worst=" + worst_offenders);
    // The production report class must reproduce the probe-level numbers exactly.
    var production_report = app.freerouting.drc.ReturnPathReport.analyze(board, return_threshold);
    org.junit.jupiter.api.Assertions.assertEquals(signal_vias.size(), production_report.signal_via_count(),
        "production signal via count");
    org.junit.jupiter.api.Assertions.assertEquals(return_vias.size(), production_report.reference_via_count(),
        "production reference via count");
    org.junit.jupiter.api.Assertions.assertEquals(plane_nets.size(), production_report.plane_net_count(),
        "production plane net count");
    org.junit.jupiter.api.Assertions.assertEquals(return_offenders, production_report.findings().size(),
        "production offender count");
    if (!offender_distances.isEmpty()) {
      double inline_worst = offender_distances.get(0)[0] == Double.MAX_VALUE
          ? Double.POSITIVE_INFINITY : offender_distances.get(0)[0];
      org.junit.jupiter.api.Assertions.assertEquals(inline_worst,
          production_report.findings().get(0).nearest_return_distance(), 1e-6,
          "production worst offender distance");
    }
    System.out.println("[phase5-return-drc] production class agrees: findings="
        + production_report.findings().size());
    // Phase 5 item 5 (report half): routed length per net and per-diff-pair mismatch.
    java.util.Map<Integer, Double> net_lengths = new java.util.TreeMap<>();
    for (Item item : board.get_items()) {
      if (item instanceof app.freerouting.board.Trace trace && trace.net_count() > 0) {
        net_lengths.merge(trace.get_net_no(0), trace.get_length(), Double::sum);
      }
    }
    List<String> pair_reports = new ArrayList<>();
    for (var pair : diff_pairs.entrySet()) {
      double len_p = 0;
      double len_n = 0;
      for (var net : board.rules.nets.get(pair.getKey())) {
        len_p += net_lengths.getOrDefault(net.net_number, 0.0);
      }
      for (var net : board.rules.nets.get(pair.getValue())) {
        len_n += net_lengths.getOrDefault(net.net_number, 0.0);
      }
      pair_reports.add(pair.getKey() + "=" + (long) len_p + " " + pair.getValue() + "="
          + (long) len_n + " mismatch=" + (long) Math.abs(len_p - len_n));
    }
    long routed_nets = net_lengths.size();
    System.out.println("[phase5-length] routed_nets=" + routed_nets
        + " diff_pair_mismatch={" + String.join("; ", pair_reports) + "}");
    // Per-net unrouted report for A/B diagnosis of the routed state.
    var incompletes_drc = new app.freerouting.drc.DesignRulesChecker(board, null);
    incompletes_drc.calculateAllIncompletes();
    java.util.List<String> unrouted_nets = new ArrayList<>();
    for (int n = 1; n <= board.rules.nets.max_net_no(); n++) {
      int incomplete_count = incompletes_drc.getIncompleteCount(n);
      if (incomplete_count > 0) {
        var net = board.rules.nets.get(n);
        unrouted_nets.add((net == null ? "net#" + n : net.name) + "(#" + n + ")x" + incomplete_count);
      }
    }
    System.out.println("[probe-unrouted] count=" + incompletes_drc.getIncompleteCount()
        + " nets=" + unrouted_nets);
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
