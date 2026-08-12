package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.geometry.planar.IntBox;
import java.util.Random;
import org.junit.jupiter.api.Test;

/**
 * Properties of the channel geometry used by live channel validation
 * ({@code featureFlags.liveChannelValidation}).
 *
 * <p>The validation's soundness rests on two invariants that hold for every channel, however
 * thin or however awkwardly the door crossings sit, and both are checked here over randomized
 * geometry rather than on hand-picked boxes:
 *
 * <ol>
 *   <li><b>Containment.</b> The validated channel is a sub-box of the plan's own channel, so
 *       validation can only ever remove space a plan was going to use -- it can never widen a
 *       plan into space the partition never proved free.
 *   <li><b>Connectivity.</b> It still contains both door crossing points, which are what
 *       Locate aims at; a channel that lost one of them would break the plan silently instead
 *       of rejecting it.
 * </ol>
 *
 * <p>The third property is the one the mechanism exists for: where the channel is wide enough,
 * the validated channel keeps a full erosion margin to the channel border, which is what makes
 * a compensated trace inside it stay inside the partition's free room.
 */
class LiveChannelValidationTest {

  @Test
  void compensatedChannel_isContained_andKeepsBothCrossings() {
    Random rnd = new Random(7);
    for (int round = 0; round < 2000; round++) {
      int ll_x = rnd.nextInt(20000) - 10000;
      int ll_y = rnd.nextInt(20000) - 10000;
      IntBox channel = new IntBox(ll_x, ll_y, ll_x + rnd.nextInt(9000), ll_y + rnd.nextInt(9000));
      IntBox entry = sub_box(channel, rnd);
      IntBox exit = sub_box(channel, rnd);
      int erosion = 1 + rnd.nextInt(3000);

      IntBox validated = PartitionRouter.compensated_channel(channel, entry, exit, erosion);

      assertFalse(validated.is_empty(), "validated channel is empty");
      assertTrue(contains(channel, validated),
          "validated channel left the plan's channel: " + PartitionRouter.box_key(validated)
              + " not in " + PartitionRouter.box_key(channel));
      assertTrue(contains(validated, clamp(PartitionRouter.joint_centre(entry), channel)),
          "validated channel lost the entry crossing point");
      assertTrue(contains(validated, clamp(PartitionRouter.joint_centre(exit), channel)),
          "validated channel lost the exit crossing point");
    }
  }

  @Test
  void compensatedChannel_keepsTheErosionMarginWhereTheChannelAllowsIt() {
    // A wide channel with both crossings in its middle: the validated channel is exactly the
    // eroded box, so a trace of half width `erosion` inside it stays inside the channel.
    IntBox channel = new IntBox(0, 0, 100000, 100000);
    IntBox entry = new IntBox(40000, 40000, 42000, 42000);
    IntBox exit = new IntBox(58000, 58000, 60000, 60000);
    int erosion = 2001;

    IntBox validated = PartitionRouter.compensated_channel(channel, entry, exit, erosion);

    assertEquals("2001:2001:97999:97999", PartitionRouter.box_key(validated));
    assertTrue(contains(channel, validated.offset(erosion)),
        "a compensated trace inside the validated channel must stay inside the channel");
  }

  @Test
  void compensatedChannel_onAChannelTooThinToErode_collapsesToTheCrossings() {
    // Erosion that would empty the channel must not empty it: the plan then stands or falls on
    // the live tree check over the crossing points themselves.
    IntBox channel = new IntBox(0, 0, 3000, 1000);
    IntBox entry = new IntBox(0, 400, 200, 600);
    IntBox exit = new IntBox(2800, 400, 3000, 600);

    IntBox validated = PartitionRouter.compensated_channel(channel, entry, exit, 2001);

    assertEquals("100:500:2900:500", PartitionRouter.box_key(validated));
    assertTrue(contains(channel, validated));
  }

  @Test
  void erodedPerAxis_collapsesOnlyTheAxisThatIsTooThin() {
    IntBox wide_and_thin = new IntBox(0, 0, 10000, 1000);

    IntBox eroded = PartitionRouter.eroded_per_axis(wide_and_thin, 2000);

    // x erodes normally; y is thinner than twice the erosion, so it collapses to its midpoint
    // instead of emptying the box (which would drop the door crossing altogether).
    assertEquals("2000:500:8000:500", PartitionRouter.box_key(eroded));
    assertTrue(contains(wide_and_thin, eroded));
  }

  @Test
  void erodedPerAxis_neverLeavesTheBox() {
    Random rnd = new Random(11);
    for (int round = 0; round < 2000; round++) {
      int ll_x = rnd.nextInt(2000);
      int ll_y = rnd.nextInt(2000);
      IntBox box = new IntBox(ll_x, ll_y, ll_x + rnd.nextInt(5000), ll_y + rnd.nextInt(5000));
      IntBox eroded = PartitionRouter.eroded_per_axis(box, rnd.nextInt(3000));
      assertFalse(eroded.is_empty());
      assertTrue(contains(box, eroded), "erosion left the box");
    }
  }

  private static IntBox sub_box(IntBox p_box, Random p_rnd) {
    int width = p_box.ur.x - p_box.ll.x;
    int height = p_box.ur.y - p_box.ll.y;
    int x0 = p_box.ll.x + (width == 0 ? 0 : p_rnd.nextInt(width + 1));
    int y0 = p_box.ll.y + (height == 0 ? 0 : p_rnd.nextInt(height + 1));
    int x1 = Math.min(p_box.ur.x, x0 + (width == 0 ? 0 : p_rnd.nextInt(width + 1)));
    int y1 = Math.min(p_box.ur.y, y0 + (height == 0 ? 0 : p_rnd.nextInt(height + 1)));
    return new IntBox(x0, y0, x1, y1);
  }

  private static IntBox clamp(IntBox p_box, IntBox p_bounds) {
    return p_box.intersection(p_bounds);
  }

  private static boolean contains(IntBox p_outer, IntBox p_inner) {
    return p_inner.ll.x >= p_outer.ll.x && p_inner.ll.y >= p_outer.ll.y
        && p_inner.ur.x <= p_outer.ur.x && p_inner.ur.y <= p_outer.ur.y;
  }
}
