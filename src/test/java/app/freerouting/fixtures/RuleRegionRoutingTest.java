package app.freerouting.fixtures;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.Freerouting;
import app.freerouting.board.RoutingBoard;
import app.freerouting.board.RuleRegion;
import app.freerouting.core.RoutingJob;
import app.freerouting.geometry.planar.IntPoint;
import app.freerouting.geometry.planar.Polyline;
import app.freerouting.io.specctra.DsnTestFixtures;
import app.freerouting.settings.RouterSettings;
import app.freerouting.settings.RuleRegionSettings;
import app.freerouting.settings.sources.TestingSettings;
import org.junit.jupiter.api.Test;

/**
 * Region-scoped rule overrides (docs/dense-bga-roadmap.md Phase 1 item 1) on the synthetic
 * {@code RuleRegionGap.dsn} fixture: a 2-layer, 20x10 mm board whose one net (two SMD pads at
 * y=-5000 um) is separated by a fixed vertical wall at x=10000 um with two identical 500 um
 * gaps -- one at y=-5000 um inside the configured rule region, one at y=-2000 um outside it.
 * The global rules (300 um width, 200 um clearance) need a 700 um gap, so the board is
 * unroutable without regions; the region rules (150 um width via trace_halfwidth_um=75,
 * 80 um clearance) need 310 um, so the in-region gap is legal there and ONLY there.
 */
public class RuleRegionRoutingTest extends RoutingFixtureTest {

  private static final String FIXTURE = "RuleRegionGap.dsn";
  /** Trace half width used by the probe polylines: the region's 75 um in board units (x10). */
  private static final int PROBE_HALF_WIDTH = 750;

  private static RuleRegionSettings[] regionSpec() {
    RuleRegionSettings region = new RuleRegionSettings();
    region.layers = "*";
    region.boxUm = new double[] {500, -7000, 19500, -3000};
    region.clearanceUm = 80.0;
    region.traceHalfwidthUm = 75.0;
    return new RuleRegionSettings[] {region};
  }

  /**
   * Verification (a) of the DRC semantics: a trace that is legal at the region clearance but
   * illegal at the global clearance passes {@code check_polyline_trace} inside the region and
   * fails outside it -- and fails everywhere when no regions are installed. Both gap
   * crossings pass the wall stubs at 175 um: legal at 80 um, illegal at 200 um.
   */
  @Test
  void drcCheckHonorsTheRegionOnlyInsideItsBox() throws Exception {
    RoutingBoard board = DsnTestFixtures.loadBoard(FIXTURE);
    int net_no = findNet(board, "N1");
    int clearance_class = board.rules.nets.get(net_no).get_class().get_trace_clearance_class();
    int[] net_arr = new int[] {net_no};
    // Horizontal crossings of the two wall gaps (board units = um * 10; layer 0 = Top).
    Polyline through_region_gap = new Polyline(new IntPoint(80000, -50000), new IntPoint(120000, -50000));
    Polyline through_outside_gap = new Polyline(new IntPoint(80000, -20000), new IntPoint(120000, -20000));

    // Without regions both crossings fail the global 200 um clearance.
    assertFalse(board.check_polyline_trace(through_region_gap, 0, PROBE_HALF_WIDTH, net_arr, clearance_class),
        "in-region gap crossing must FAIL the DRC check at global rules");
    assertFalse(board.check_polyline_trace(through_outside_gap, 0, PROBE_HALF_WIDTH, net_arr, clearance_class),
        "outside-region gap crossing must FAIL the DRC check at global rules");

    // Install the region and re-check: only the in-region crossing becomes legal.
    RouterSettings settings = new RouterSettings();
    settings.ruleRegions = regionSpec();
    RuleRegion.install(board, settings);
    assertNotNull(board.rule_regions, "region installation must populate board.rule_regions");
    assertEquals(1, board.rule_regions.size());
    RuleRegion region = board.rule_regions.get(0);
    assertEquals(800, region.clearance, "80 um must resolve to 800 board units");
    assertEquals(750, region.trace_half_width, "75 um must resolve to 750 board units");
    assertEquals(5000, region.box.ll.x);
    assertEquals(-70000, region.box.ll.y);
    assertEquals(195000, region.box.ur.x);
    assertEquals(-30000, region.box.ur.y);

    assertTrue(board.check_polyline_trace(through_region_gap, 0, PROBE_HALF_WIDTH, net_arr, clearance_class),
        "in-region gap crossing must PASS the DRC check once the region is installed");
    assertFalse(board.check_polyline_trace(through_outside_gap, 0, PROBE_HALF_WIDTH, net_arr, clearance_class),
        "outside-region gap crossing must still FAIL: the region does not cover it");
  }

  /**
   * Clearance-class sharing (the memory fix): the number of clearance classes appended by
   * region installation is the number of DISTINCT clearance VALUES, not the number of
   * regions. This is a memory-scaling invariant, not a cosmetic one -- the router allocates a
   * separate FULL-BOARD compensated search tree per distinct clearance class it routes at and
   * retains it for the life of the board (plus a precalculated tile-shape array per item per
   * tree), so one class per region multiplies the whole board by the region count. Measured on
   * an 8-layer fine-pitch board: 14 regions at one clearance OOM-ed a 6 GB heap in pass 1;
   * sharing the class fits the same run in well under that (docs/dense-bga-roadmap.md).
   */
  @Test
  void regionsShareOneClearanceClassPerDistinctClearanceValue() throws Exception {
    RoutingBoard board = DsnTestFixtures.loadBoard(FIXTURE);
    int classes_before = board.rules.clearance_matrix.get_class_count();

    // 14 regions (the measured repro's count), all carrying the same clearance.
    RuleRegionSettings[] same_clearance = new RuleRegionSettings[14];
    for (int i = 0; i < same_clearance.length; i++) {
      RuleRegionSettings curr = new RuleRegionSettings();
      curr.layers = "*";
      // Disjoint boxes, so this is not deduplication by geometry.
      curr.boxUm = new double[] {500 + i * 1000, -7000, 1400 + i * 1000, -3000};
      curr.clearanceUm = 80.0;
      same_clearance[i] = curr;
    }
    RouterSettings settings = new RouterSettings();
    settings.ruleRegions = same_clearance;
    RuleRegion.install(board, settings);

    assertNotNull(board.rule_regions);
    assertEquals(14, board.rule_regions.size(), "all 14 regions must be installed");
    assertEquals(classes_before + 1, board.rules.clearance_matrix.get_class_count(),
        "14 regions at one clearance value must append exactly ONE clearance class");
    int shared_class = board.rule_regions.get(0).clearance_class_no;
    for (RuleRegion region : board.rule_regions) {
      assertEquals(shared_class, region.clearance_class_no,
          "every region at the same clearance must share the same clearance class");
      assertEquals(800, region.clearance, "the shared class must still carry 80 um = 800 units");
    }

    // Distinct clearance values still get distinct classes: the class IS its clearance value,
    // so regions at different clearances must not be collapsed.
    RoutingBoard board2 = DsnTestFixtures.loadBoard(FIXTURE);
    int classes_before2 = board2.rules.clearance_matrix.get_class_count();
    double[] clearances = {80.0, 80.0, 120.0, 80.0, 120.0, 150.0};
    RuleRegionSettings[] mixed = new RuleRegionSettings[clearances.length];
    for (int i = 0; i < mixed.length; i++) {
      RuleRegionSettings curr = new RuleRegionSettings();
      curr.layers = "*";
      curr.boxUm = new double[] {500 + i * 1000, -7000, 1400 + i * 1000, -3000};
      curr.clearanceUm = clearances[i];
      mixed[i] = curr;
    }
    RouterSettings settings2 = new RouterSettings();
    settings2.ruleRegions = mixed;
    RuleRegion.install(board2, settings2);

    assertEquals(6, board2.rule_regions.size());
    assertEquals(classes_before2 + 3, board2.rules.clearance_matrix.get_class_count(),
        "6 regions over 3 distinct clearance values must append exactly 3 clearance classes");
    // Same value -> same class; different value -> different class.
    assertEquals(board2.rule_regions.get(0).clearance_class_no,
        board2.rule_regions.get(1).clearance_class_no);
    assertEquals(board2.rule_regions.get(0).clearance_class_no,
        board2.rule_regions.get(3).clearance_class_no);
    assertEquals(board2.rule_regions.get(2).clearance_class_no,
        board2.rule_regions.get(4).clearance_class_no);
    assertNotEquals(board2.rule_regions.get(0).clearance_class_no,
        board2.rule_regions.get(2).clearance_class_no);
    assertNotEquals(board2.rule_regions.get(0).clearance_class_no,
        board2.rule_regions.get(5).clearance_class_no);
    assertNotEquals(board2.rule_regions.get(2).clearance_class_no,
        board2.rule_regions.get(5).clearance_class_no);
    // And each shared class still requires exactly its own region clearance.
    for (int i = 0; i < clearances.length; i++) {
      RuleRegion region = board2.rule_regions.get(i);
      assertEquals((int) Math.round(clearances[i] * 10), region.clearance);
      assertEquals(region.clearance, board2.rules.clearance_matrix.get_value(
          region.clearance_class_no, 1, 0, false),
          "the shared class's matrix row must carry its own clearance value");
    }
  }

  /**
   * Verification (b), end-to-end: the fixture route completes with the region enabled and
   * does not complete without it, at zero clearance violations either way; and the flag-on
   * outcome is deterministic (two runs, identical final board hash).
   */
  @Test
  void regionEnablesRoutingThroughTheNeckedGap() {
    RoutingJob flag_off = runJob(false);
    assertRoutingResult(flag_off, FIXTURE)
        .exactIncompleteConnections(1) // the wall is impassable at global rules
        .exactClearanceViolations(0)
        .check();

    RoutingJob flag_on_1 = runJob(true);
    assertRoutingResult(flag_on_1, FIXTURE)
        .exactIncompleteConnections(0) // the region retry routes through the in-region gap
        .exactClearanceViolations(0)
        .check();

    RoutingJob flag_on_2 = runJob(true);
    assertRoutingResult(flag_on_2, FIXTURE)
        .exactIncompleteConnections(0)
        .exactClearanceViolations(0)
        .check();
    assertEquals(flag_on_1.board.get_hash(), flag_on_2.board.get_hash(),
        "flag-on routing must be deterministic (2/2 identical boards)");
  }

  private RoutingJob runJob(boolean with_regions) {
    Freerouting.globalSettings.featureFlags.ruleRegions = with_regions;
    TestingSettings testingSettings = new TestingSettings();
    testingSettings.setFanoutEnabled(false);
    if (with_regions) {
      testingSettings.setRuleRegions(regionSpec());
    }
    RoutingJob job = GetRoutingJob(FIXTURE, testingSettings);
    return RunRoutingJob(job);
  }

  private static int findNet(RoutingBoard board, String name) {
    for (int i = 1; i <= board.rules.nets.max_net_no(); i++) {
      var net = board.rules.nets.get(i);
      if (net != null && name.equals(net.name)) {
        return i;
      }
    }
    throw new IllegalStateException("net not found: " + name);
  }
}
