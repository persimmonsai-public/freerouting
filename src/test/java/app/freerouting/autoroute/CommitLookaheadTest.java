package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * The trial-commit lookahead's decision rule ({@code featureFlags.commitLookahead}) over the
 * horizon pairs actually MEASURED on the gate fixtures (docs/dense-bga-roadmap.md, trial-commit
 * lookahead increment). Every number here is transcribed from a {@code [lookahead]} log line of
 * a flag-on run, so the test doubles as the regression guard for the measured behaviour: it
 * fails the day the shipped rule stops keeping bm11's paying commit, stops declining bm01's
 * toxic ones, or loses bm04's complementary chain.
 */
class CommitLookaheadTest {

  private static final int SHIPPED_MARGIN = 2;
  private static final String SHIPPED_TIE = "accept";

  private static boolean keeps(int p_accept, int p_decline) {
    return BatchAutorouter.lookahead_keeps(p_accept, p_decline, SHIPPED_MARGIN, SHIPPED_TIE);
  }

  @Test
  void keepsBm11sPayingCommit() {
    // bm11 negotiation + live channel: the ONE commit worth +1 net (987.49/2 vs 981.24/3).
    // Every commit-LOCAL predicate declined it -- own-net completion (completes=false),
    // corridor slack (mincap 0.42) and contention all get it wrong. The horizon gets it right
    // for the only reason that matters: the run measurably ends better with it.
    assertTrue(keeps(2, 3));
  }

  @Test
  void keepsBm04sComplementaryChain() {
    // bm04's three candidates, probed in commit order. The first looks like a 2-net LOSS on
    // its own (5 against a decline-all continuation of 3) and is kept only by the margin --
    // and once it is on the board the second probe flips to a 2-net GAIN. This is the whole
    // reason the margin exists: the chain pays, no member of it pays alone.
    assertTrue(keeps(5, 3));
    assertTrue(keeps(3, 5));
    assertTrue(keeps(3, 3));
  }

  @Test
  void declinesBm01sToxicCommits() {
    // bm01 negotiation + live channel, from the board that reaches 0 incompletes without
    // them: these are the commits that take the champion from 994.85/1 down to 979.47/4 when
    // the router is not allowed to look. Each is more than the margin worse.
    assertFalse(keeps(5, 0));
    assertFalse(keeps(4, 1));
    assertFalse(keeps(5, 1));
    assertFalse(keeps(8, 0));
  }

  @Test
  void keepsBm01sChampionCommits() {
    // The two bm01 commits the horizon shows PAYING: net 72 (10 incompletes avoided) and
    // net 34.
    assertTrue(keeps(0, 10));
    assertTrue(keeps(0, 3));
  }

  @Test
  void aZeroMarginKillsBm04sChainAtItsFirstMember() {
    // The refutation the margin exists to answer, pinned as an assertion: with the untuned
    // rule bm04's first two candidates are declined and the board falls to 972.02/4, BELOW
    // its own flag-off 979.01/3.
    assertFalse(BatchAutorouter.lookahead_keeps(5, 3, 0, SHIPPED_TIE));
    assertFalse(BatchAutorouter.lookahead_keeps(5, 3, 0, SHIPPED_TIE));
    // ... while the candidates the horizon shows paying outright survive any margin.
    assertTrue(BatchAutorouter.lookahead_keeps(2, 3, 0, SHIPPED_TIE));
  }

  @Test
  void tieRuleOnlyDecidesExactlyOnTheThreshold() {
    // On the threshold the tie rule decides, and nowhere else -- so "decline" cannot turn a
    // measured gain into a decline, and "accept" cannot rescue a measured loss.
    assertTrue(BatchAutorouter.lookahead_keeps(5, 3, 2, "accept"));
    assertFalse(BatchAutorouter.lookahead_keeps(5, 3, 2, "decline"));
    assertTrue(BatchAutorouter.lookahead_keeps(4, 3, 2, "decline"));
    assertFalse(BatchAutorouter.lookahead_keeps(6, 3, 2, "accept"));
  }

  @Test
  void unmeasuredHorizonsAreNeverKept() {
    // A probe that fails returns Integer.MAX_VALUE; it must not overflow into an accept.
    assertFalse(keeps(Integer.MAX_VALUE, 3));
    assertFalse(BatchAutorouter.lookahead_keeps(Integer.MAX_VALUE, Integer.MAX_VALUE - 1,
        SHIPPED_MARGIN, SHIPPED_TIE));
  }

  @Test
  void theRuleIsOrderIndependentInItsInputs() {
    // The comparison is between two counts of the same thing, so it cannot depend on which
    // trial ran first -- swapping the pair inverts the verdict at margin 0 unless they tie.
    for (int accept = 0; accept < 30; accept++) {
      for (int decline = 0; decline < 30; decline++) {
        boolean forward = BatchAutorouter.lookahead_keeps(accept, decline, 0, "decline");
        boolean backward = BatchAutorouter.lookahead_keeps(decline, accept, 0, "decline");
        assertEquals(accept != decline, forward != backward);
      }
    }
  }
}
