package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * The commit-acceptance predicate ({@code featureFlags.commitPolicy}) over the feature
 * vectors actually MEASURED on the gate fixtures (docs/dense-bga-roadmap.md, commit-policy
 * increment). The numbers in these tests are transcribed from the {@code [commit-feature]}
 * log lines of the flag-on runs, so the test doubles as the regression guard for the
 * measured classification: it fails the day the shipped predicate stops declining bm05's
 * diagnosed toxic commit or starts declining bm04's three paying ones.
 */
class CommitPolicyTest {

  private static BatchAutorouter.CommitFeatures features(boolean p_completes, double p_min_capacity,
      int p_contention) {
    BatchAutorouter.CommitFeatures f = new BatchAutorouter.CommitFeatures();
    f.completes_net = p_completes;
    f.min_capacity = p_min_capacity;
    f.contention = p_contention;
    return f;
  }

  @Test
  void shippedPredicateAcceptsTheThreeMeasuredBm04Commits() {
    // bm04 negotiation + live channel validation: the three commits worth +1 net.
    assertTrue(BatchAutorouter.commit_policy_accepts(features(true, 1.32, 0), "capacity"));
    assertTrue(BatchAutorouter.commit_policy_accepts(features(true, 1.57, 0), "capacity"));
    assertTrue(BatchAutorouter.commit_policy_accepts(features(true, 1.67, 0), "capacity"));
  }

  @Test
  void shippedPredicateDeclinesTheDiagnosedBm05ToxicCommit() {
    // /XADUIO_SCL, the commit the bm05 diagnosis attributed 4 lost nets to: mincap 0.38.
    assertFalse(BatchAutorouter.commit_policy_accepts(features(true, 0.38, -1), "capacity"));
  }

  @Test
  void ownNetCompletionDoesNotSeparateTheMeasuredClasses() {
    // The banked signature, refuted: bm05's diagnosed toxic commit completes its own net at
    // commit time (it is ripped up later), so the completion predicate accepts it ...
    assertTrue(BatchAutorouter.commit_policy_accepts(features(true, 0.38, -1), "completes"));
    // ... while it declines /3.3VDD, a commit in bm05's +2-net winning chain.
    assertFalse(BatchAutorouter.commit_policy_accepts(features(false, 1.27, -1), "completes"));
  }

  @Test
  void contentionPredicateDeclinesAMeasuredChampionCommit() {
    // bm01's champion commit PL7 shares an interior room with another dry route
    // (contention=1), so a contention gate removes a beneficial commit.
    assertFalse(BatchAutorouter.commit_policy_accepts(features(true, 2.71, 1), "contention"));
    assertTrue(BatchAutorouter.commit_policy_accepts(features(true, 2.71, 1), "capacity"));
  }

  @Test
  void slackModeRequiresBothSignals() {
    assertTrue(BatchAutorouter.commit_policy_accepts(features(true, 1.32, 0), "slack"));
    assertFalse(BatchAutorouter.commit_policy_accepts(features(false, 1.32, 0), "slack"));
    assertFalse(BatchAutorouter.commit_policy_accepts(features(true, 0.38, 0), "slack"));
  }
}
