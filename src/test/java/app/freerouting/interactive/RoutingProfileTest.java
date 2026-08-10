package app.freerouting.interactive;

import app.freerouting.Freerouting;
import app.freerouting.core.RoutingJob;
import app.freerouting.core.RoutingJobState;
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
import java.util.UUID;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * A single, long, SEQUENTIAL routing run used purely as a profiling workload.
 *
 * <p>Deliberately single-threaded ({@code maxThreads = 1}) so a profiler attached to this run
 * measures the router's intrinsic algorithmic cost -- where the work actually is -- rather than
 * parallel scheduling artifacts. Run it with JFR attached to produce a hot-method profile.
 */
class RoutingProfileTest {

  private static final String FIXTURE = System.getProperty("fr.fixture", "Issue508-DAC2020_bm01.dsn");

  private RoutingJobScheduler scheduler;

  @BeforeEach
  void setUp() {
    Freerouting.globalSettings = new GlobalSettings();
    Freerouting.globalSettings.featureFlags.parallelAutorouter = true;
    Freerouting.globalSettings.featureFlags.partitionRouter = Boolean.getBoolean("fr.partition");
    Freerouting.globalSettings.featureFlags.escapePlanner = Boolean.getBoolean("fr.escape");
    Freerouting.globalSettings.featureFlags.reflowablePours = Boolean.getBoolean("fr.reflow");
    Freerouting.globalSettings.featureFlags.negotiatedRouter = Boolean.getBoolean("fr.negotiate");
    Freerouting.globalSettings.featureFlags.negotiatedRouterParallelDry = Boolean.getBoolean("fr.negpar");
    Freerouting.globalSettings.featureFlags.meanderMatching = Boolean.getBoolean("fr.meander");
    Freerouting.globalSettings.featureFlags.pairCorridorAffinity = Boolean.getBoolean("fr.pairaffinity");
    Freerouting.globalSettings.featureFlags.checkedRealizer = Boolean.getBoolean("fr.checked");
    InteractiveSettings.resetForTesting();
    scheduler = RoutingJobScheduler.getInstance();
    synchronized (scheduler.jobs) {
      scheduler.jobs.clear();
    }
  }

  @Test
  @Timeout(value = 1500, unit = TimeUnit.SECONDS)
  void profileSequentialRouting() {
    TestingSettings settings = new TestingSettings();
    settings.setMaxPasses(8);
    settings.setMaxItems(500);
    settings.setJobTimeoutString(System.getProperty("fr.timeout", "00:03:00"));

    RoutingJob job = createRoutingJob(FIXTURE, settings);
    job.routerSettings.maxThreads = 1;

    long start = System.currentTimeMillis();
    runRoutingJob(job);
    long elapsed = System.currentTimeMillis() - start;

    System.out.println("[profile-run] sequential routing completed in " + elapsed + "ms, state=" + job.state);
  }

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

  private RoutingJob runRoutingJob(RoutingJob job) {
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
    return job;
  }
}
