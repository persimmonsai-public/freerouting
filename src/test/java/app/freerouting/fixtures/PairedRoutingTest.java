package app.freerouting.fixtures;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.Freerouting;
import app.freerouting.autoroute.PairedRouter;
import app.freerouting.core.RoutingJob;
import app.freerouting.settings.sources.TestingSettings;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

/**
 * Paired differential routing with dual offset emission (docs/dense-bga-roadmap.md Phase 5 item 6)
 * on the synthetic {@code PairRouteGaps.dsn} fixture.
 *
 * <p>The fixture is a 2-layer 4x2.2 mm board carrying one differential pair (D+/D-, through-hole
 * pads so no fanout intervenes) and a vertical wall with two gaps. The upper gap is wide enough
 * for exactly ONE trace and the lower gap is wide enough for the pair, so the classic engine --
 * routing the two members independently and greedily -- sends one member through the near gap and
 * the other on a long detour through the far one, producing a large length mismatch. The paired
 * stage routes ONE centreline through the far gap and emits both members off it, which is the
 * mismatch the pair actually wants closed.
 */
public class PairedRoutingTest extends RoutingFixtureTest {

  private static final String FIXTURE = "PairRouteGaps.dsn";

  @AfterEach
  void clearCushionOverride() {
    System.clearProperty("fr.pairedroute.cushion");
  }

  /**
   * End to end: the pair's routed-length mismatch collapses with the flag on, both members are
   * emitted and DRC-clean, the board keeps zero clearance violations, and the flag-on run is
   * deterministic.
   */
  @Test
  void pairedRoutingCollapsesTheMembersLengthMismatch() {
    RoutingJob flag_off = runJob(false);
    assertRoutingResult(flag_off, FIXTURE)
        .exactIncompleteConnections(0)
        .exactClearanceViolations(0)
        .check();
    double unpaired_mismatch = mismatch(flag_off);
    assertTrue(unpaired_mismatch > 40000,
        "the fixture must present a real matching defect unpaired, measured 55162, got "
            + unpaired_mismatch);

    RoutingJob flag_on_1 = runJob(true);
    assertRoutingResult(flag_on_1, FIXTURE)
        .exactIncompleteConnections(0)
        .exactClearanceViolations(0)
        .check();
    double paired_mismatch = mismatch(flag_on_1);
    assertTrue(paired_mismatch < unpaired_mismatch / 10,
        "paired emission must collapse the mismatch (unpaired " + unpaired_mismatch
            + ", paired " + paired_mismatch + ")");
    // Both members really are on the board: neither length may be zero, and their sum grows
    // because both now take the pair corridor's detour.
    assertTrue(PairedRouter.net_length(flag_on_1.board, "D+") > 0);
    assertTrue(PairedRouter.net_length(flag_on_1.board, "D-") > 0);

    RoutingJob flag_on_2 = runJob(true);
    assertEquals(flag_on_1.board.get_hash(), flag_on_2.board.get_hash(),
        "paired routing must be deterministic (2/2 identical boards)");
    assertEquals(paired_mismatch, mismatch(flag_on_2));
  }

  /**
   * Commit coordination: with the pitch cushion forced to 1 board unit the second member fails
   * the DRC, and the whole pair is rolled back -- the run must land on EXACTLY the flag-off
   * board, not on a board carrying a half-committed pair.
   */
  @Test
  void aPairThatCannotBeFullyEmittedIsRolledBackWhole() {
    RoutingJob flag_off = runJob(false);
    double unpaired_mismatch = mismatch(flag_off);

    System.setProperty("fr.pairedroute.cushion", "1");
    RoutingJob half_pair = runJob(true);
    assertRoutingResult(half_pair, FIXTURE)
        .exactIncompleteConnections(0)
        .exactClearanceViolations(0)
        .check();
    assertEquals(unpaired_mismatch, mismatch(half_pair),
        "a rolled-back pair must leave the classic engine's board untouched");
    // Per-member equality is the atomicity assertion: neither member may carry a surviving
    // half of the pair. (Board HASHES are not compared here: the rolled-back insert consumes
    // item ids, so every later item is renumbered even though the geometry is identical.)
    assertEquals(PairedRouter.net_length(flag_off.board, "D+"),
        PairedRouter.net_length(half_pair.board, "D+"),
        "D+ must be exactly the classic engine's route after the pair rolled back");
    assertEquals(PairedRouter.net_length(flag_off.board, "D-"),
        PairedRouter.net_length(half_pair.board, "D-"),
        "D- must be exactly the classic engine's route after the pair rolled back");
  }

  private static double mismatch(RoutingJob p_job) {
    return Math.abs(PairedRouter.net_length(p_job.board, "D+")
        - PairedRouter.net_length(p_job.board, "D-"));
  }

  private RoutingJob runJob(boolean p_paired) {
    Freerouting.globalSettings.featureFlags.pairedRouting = p_paired;
    TestingSettings testingSettings = new TestingSettings();
    testingSettings.setFanoutEnabled(false);
    RoutingJob job = GetRoutingJob(FIXTURE, testingSettings);
    return RunRoutingJob(job);
  }
}
