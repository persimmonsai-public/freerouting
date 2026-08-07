package app.freerouting.interactive;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.Freerouting;
import app.freerouting.core.RoutingJob;
import app.freerouting.core.RoutingJobState;
import app.freerouting.drc.DesignRulesChecker;
import app.freerouting.management.RoutingJobScheduler;
import app.freerouting.management.SessionManager;
import app.freerouting.settings.GlobalSettings;
import app.freerouting.settings.RouterSettings;
import app.freerouting.settings.SettingsMerger;
import app.freerouting.settings.sources.DefaultSettings;
import app.freerouting.settings.sources.DsnFileSettings;
import app.freerouting.settings.sources.TestingSettings;
import app.freerouting.util.TextManager;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.UUID;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.RepetitionInfo;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Correctness and wall-clock benchmark for the parallel autorouting path added to
 * {@link app.freerouting.autoroute.BatchAutorouter#autoroute_pass_parallel}.
 *
 * <p>The parallel path is bulk-synchronous: within a batch every worker searches with the board
 * frozen, then a single thread commits the resulting plans, re-validating each against the live
 * board first. That structure is what makes it sound -- searches read live Item geometry well
 * beyond the search tree, so they cannot overlap with commits.
 *
 * <p><b>On workload size.</b> An earlier version of this class checked correctness with 30 items
 * over a single pass and passed consistently -- while the router was, in fact, intermittently
 * committing overlapping copper and throwing mid-search. Those races only surface once enough
 * connections contend: at 500 items over 8 passes they appeared in roughly half of runs. A
 * concurrency test that never loses the race proves nothing, so the correctness check below uses
 * the large workload and repeats, and it asserts an absolute result (zero violations) rather
 * than a comparison against a sequential run that would double its cost.
 *
 * <ol>
 *   <li><b>Correctness</b>: repeated large-workload parallel runs must produce no clearance
 *       violations and no errors.
 *   <li><b>Speedup</b>: the parallel run should beat the sequential run's wall clock. This is a
 *       benchmark, not a hard gate -- shared CI hardware is noisy -- so it logs the measured
 *       ratio rather than asserting a multiplier.
 * </ol>
 */
class ParallelAutorouteBenchmarkTest {

  private static final String FIXTURE = "Issue508-DAC2020_bm01.dsn";

  private RoutingJobScheduler scheduler;

  @BeforeEach
  void setUp() {
    Freerouting.globalSettings = new GlobalSettings();
    // The parallel auto-router is opt-in (off by default); this test exists to exercise it.
    Freerouting.globalSettings.featureFlags.parallelAutorouter = true;
    InteractiveSettings.resetForTesting();
    scheduler = RoutingJobScheduler.getInstance();
    synchronized (scheduler.jobs) {
      scheduler.jobs.clear();
    }
  }

  /**
   * Routes a contended workload in parallel and requires a clean board.
   *
   * <p>Repeated because the failures this guards against are races. Verified by mutation: with
   * the commit-time re-validation disabled, this test caught the resulting violation in 1 of 3
   * repetitions -- so any single-run version of it would have reported success two times out of
   * three. Five repetitions puts detection near 90%. Sized to the workload that actually reaches
   * the race: with 30 items over one pass, both bugs passed this suite for their entire lifetime.
   */
  @RepeatedTest(5)
  @Timeout(value = 120, unit = TimeUnit.SECONDS)
  void parallelAutorouting_producesNoViolations_underContention(RepetitionInfo repetition) {
    TestingSettings settings = new TestingSettings();
    settings.setMaxPasses(8);
    settings.setMaxItems(500);
    settings.setJobTimeoutString("00:01:30");

    RoutingJob job = assertDoesNotThrow(
        () -> {
          RoutingJob parallelJob = createRoutingJob(FIXTURE, settings);
          parallelJob.routerSettings.maxThreads = 8;
          runRoutingJob(parallelJob);
          return parallelJob;
        },
        "Parallel autorouting must not throw");

    int violations = countClearanceViolations(job);
    System.out.println("[parallel-drc-check] repetition=" + repetition.getCurrentRepetition()
        + " violations=" + violations + " routed=" + countRoutedTraces(job));

    assertTrue(
        job.state == RoutingJobState.COMPLETED || job.state == RoutingJobState.CANCELLED
            || job.state == RoutingJobState.TERMINATED,
        "Parallel routing job must reach a terminal state");
    assertEquals(0, violations,
        "Parallel autorouting left " + violations + " clearance violation(s) on the board. Two "
            + "workers committed overlapping geometry -- check that the search phase runs with "
            + "the board frozen and that commit-time re-validation is still in place.");
  }

  @Test
  @Timeout(value = 150, unit = TimeUnit.SECONDS)
  void parallelAutorouting_isFasterThanSequential_onALargerBudget() {
    TestingSettings sequentialSettings = new TestingSettings();
    sequentialSettings.setMaxPasses(1);
    sequentialSettings.setMaxItems(60);
    sequentialSettings.setJobTimeoutString("00:01:00");
    RoutingJob sequentialJob = createRoutingJob(FIXTURE, sequentialSettings);
    sequentialJob.routerSettings.maxThreads = 1;
    long sequentialStart = System.currentTimeMillis();
    runRoutingJob(sequentialJob);
    long sequentialMillis = System.currentTimeMillis() - sequentialStart;

    TestingSettings parallelSettings = new TestingSettings();
    parallelSettings.setMaxPasses(1);
    parallelSettings.setMaxItems(60);
    parallelSettings.setJobTimeoutString("00:01:00");
    RoutingJob parallelJob = createRoutingJob(FIXTURE, parallelSettings);
    parallelJob.routerSettings.maxThreads = 8;
    long parallelStart = System.currentTimeMillis();
    runRoutingJob(parallelJob);
    long parallelMillis = System.currentTimeMillis() - parallelStart;

    double speedup = parallelMillis == 0 ? 0 : sequentialMillis / (double) parallelMillis;
    System.out.println("[parallel-speedup] sequential=" + sequentialMillis + "ms, parallel="
        + parallelMillis + "ms, speedup=" + String.format("%.2fx", speedup)
        + ", sequential routed=" + countRoutedTraces(sequentialJob)
        + ", parallel routed=" + countRoutedTraces(parallelJob));

    assertNotNull(sequentialJob.board, "Sequential job must produce a board");
    assertNotNull(parallelJob.board, "Parallel job must produce a board");
  }

  private int countClearanceViolations(RoutingJob job) {
    assertNotNull(job.board, "Job must have produced a board to check for violations");
    DesignRulesChecker drc = new DesignRulesChecker(job.board, null);
    return drc.getAllClearanceViolations().size();
  }

  private int countRoutedTraces(RoutingJob job) {
    if (job.board == null) {
      return 0;
    }
    int count = 0;
    for (app.freerouting.board.Item item : job.board.get_items()) {
      if (item instanceof app.freerouting.board.Trace) {
        count++;
      }
    }
    return count;
  }

  // ── Helpers (mirrors HeadlessRoutingTest) ──────────────────────────────────

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

      if (testingSettings.getSettings().jobTimeoutString == null) {
        testingSettings.setJobTimeoutString("00:01:00");
      }
      if (testingSettings.getSettings().maxPasses == null) {
        testingSettings.setMaxPasses(100);
      }
      merger.addOrReplaceSources(testingSettings);

      Freerouting.globalSettings.settingsMergerProtype.addOrReplaceSources(testingSettings);
      job.routerSettings = merger.merge();
    } catch (IOException e) {
      throw new RuntimeException(testFile + " not found.", e);
    }

    return job;
  }

  private RoutingJob runRoutingJob(RoutingJob job) {
    if (job == null) {
      throw new IllegalArgumentException("The job cannot be null.");
    }

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
        // Best-effort: ask the router to stop before giving up on it, so a slow-converging
        // board doesn't leave a background thread pool running after the test has moved on.
        if (job.thread != null) {
          job.thread.requestStop();
        }
        float timeoutInMinutes = timeoutInMillis / 60000.0f;
        throw new RuntimeException("Routing job timed out after " + timeoutInMinutes + " minutes.");
      }
    }

    return job;
  }
}
