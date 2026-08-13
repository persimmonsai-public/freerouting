package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.geometry.planar.IntPoint;
import org.junit.jupiter.api.Test;

/**
 * The dual-offset emission geometry of {@link PairedRouter} (docs/dense-bga-roadmap.md Phase 5
 * item 6): the mitred offset that turns one realized centreline into the two members' polylines,
 * and the 45-degree fan-in leg that attaches each member to its own terminal.
 *
 * <p>These are the two pieces the pair's length matching rests on -- both members follow the same
 * centreline at a constant perpendicular distance, so their lengths can only differ by the corner
 * mitres and the fan-in legs.
 */
class PairedRouterGeometryTest {

  /** Perpendicular distance of a point from the infinite line through p_a and p_b. */
  private static double distance_to_line(IntPoint p_a, IntPoint p_b, IntPoint p_point) {
    double ux = (double) p_b.x - p_a.x;
    double uy = (double) p_b.y - p_a.y;
    double length = Math.hypot(ux, uy);
    double wx = (double) p_point.x - p_a.x;
    double wy = (double) p_point.y - p_a.y;
    return (ux * wy - uy * wx) / length;
  }

  @Test
  void straightCentrelineOffsetsToTheLeftForPositiveDistance() {
    IntPoint[] centre = {new IntPoint(0, 0), new IntPoint(10000, 0)};
    IntPoint[] left = PairedRouter.offset_polyline(centre, 500);
    IntPoint[] right = PairedRouter.offset_polyline(centre, -500);
    assertNotNull(left);
    assertNotNull(right);
    assertEquals(new IntPoint(0, 500), left[0]);
    assertEquals(new IntPoint(10000, 500), left[1]);
    assertEquals(new IntPoint(0, -500), right[0]);
    assertEquals(new IntPoint(10000, -500), right[1]);
    // The two members' separation is exactly the pitch everywhere.
    assertEquals(1000, left[0].y - right[0].y);
  }

  /**
   * The load-bearing property: every offset segment stays at exactly the offset distance from
   * its own centreline segment, through 45- and 90-degree corners alike. That is what makes the
   * two members' lengths differ only by their mitres, and it is checked here on a centreline
   * with one axis run, one diagonal and one 90-degree turn.
   */
  @Test
  void mitredOffsetKeepsEverySegmentAtTheOffsetDistance() {
    IntPoint[] centre = {
        new IntPoint(0, 0), new IntPoint(10000, 0), new IntPoint(20000, -10000),
        new IntPoint(20000, -30000), new IntPoint(40000, -30000)};
    for (int offset : new int[]{2508, -2508}) {
      IntPoint[] emitted = PairedRouter.offset_polyline(centre, offset);
      assertNotNull(emitted, "offset must exist for a 45/90-degree centreline");
      assertEquals(centre.length, emitted.length);
      for (int i = 0; i + 1 < centre.length; i++) {
        assertEquals(offset, distance_to_line(centre[i], centre[i + 1], emitted[i]), 1.0,
            "offset corner " + i + " must sit at the offset distance from segment " + i);
        assertEquals(offset, distance_to_line(centre[i], centre[i + 1], emitted[i + 1]), 1.0,
            "offset corner " + (i + 1) + " must sit at the offset distance from segment " + i);
      }
    }
  }

  @Test
  void offsetPreservesEverySegmentDirection() {
    IntPoint[] centre = {
        new IntPoint(0, 0), new IntPoint(10000, -10000), new IntPoint(30000, -10000)};
    IntPoint[] emitted = PairedRouter.offset_polyline(centre, 1000);
    assertNotNull(emitted);
    for (int i = 0; i + 1 < centre.length; i++) {
      double cx = (double) centre[i + 1].x - centre[i].x;
      double cy = (double) centre[i + 1].y - centre[i].y;
      double ex = (double) emitted[i + 1].x - emitted[i].x;
      double ey = (double) emitted[i + 1].y - emitted[i].y;
      assertEquals(0.0, cx * ey - cy * ex, 1e-6,
          "emitted segment " + i + " must be parallel to the centreline segment");
      assertTrue(cx * ex + cy * ey > 0, "emitted segment " + i + " must keep its orientation");
    }
  }

  /** A centreline that doubles back has no sound miter; the pair is refused rather than deformed. */
  @Test
  void doublingBackCentrelineIsRefused() {
    IntPoint[] hairpin = {new IntPoint(0, 0), new IntPoint(10000, 0), new IntPoint(0, 1)};
    assertNull(PairedRouter.offset_polyline(hairpin, 1000));
    IntPoint[] degenerate = {new IntPoint(0, 0), new IntPoint(0, 0), new IntPoint(10000, 0)};
    assertNull(PairedRouter.offset_polyline(degenerate, 1000));
  }

  /**
   * The fan-in leg is one axis-parallel segment followed by one exact 45-degree diagonal, so a
   * member can leave its own pad and reach the offset body without breaking the board's
   * 45-degree angle restriction.
   */
  @Test
  void fanInCornerIsAxisThenExactDiagonal() {
    IntPoint pad = new IntPoint(1000, -2000);
    IntPoint body = new IntPoint(9000, -5000);
    IntPoint mid = PairedRouter.fan_in_corner(pad, body);
    assertNotNull(mid);
    assertTrue(mid.x == pad.x || mid.y == pad.y, "first leg must be axis-parallel");
    assertEquals(Math.abs(body.x - mid.x), Math.abs(body.y - mid.y),
        "second leg must be an exact 45-degree diagonal");
    // The leg stays inside the pad-to-body bounding box: no excursion away from the corridor.
    assertTrue(mid.x >= Math.min(pad.x, body.x) && mid.x <= Math.max(pad.x, body.x));
    assertTrue(mid.y >= Math.min(pad.y, body.y) && mid.y <= Math.max(pad.y, body.y));
  }

  @Test
  void fanInNeedsNoCornerWhenTheApproachIsAlreadyLegal() {
    assertNull(PairedRouter.fan_in_corner(new IntPoint(0, 0), new IntPoint(5000, -5000)),
        "an exact diagonal approach needs no intermediate corner");
    assertNull(PairedRouter.fan_in_corner(new IntPoint(0, 0), new IntPoint(5000, 0)),
        "an axis-parallel approach needs no intermediate corner");
    assertNull(PairedRouter.fan_in_corner(new IntPoint(0, 0), new IntPoint(0, -5000)),
        "an axis-parallel approach needs no intermediate corner");
  }
}
