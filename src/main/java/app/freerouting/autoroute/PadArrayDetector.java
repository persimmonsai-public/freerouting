package app.freerouting.autoroute;

import app.freerouting.board.Item;
import app.freerouting.board.Pin;
import app.freerouting.board.RoutingBoard;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntBox;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Detects array-arranged pad fields -- BGA grids, QFP-style perimeter rings, dual/single-row
 * peripheral components -- and derives the ESCAPE AXIS of each member pad: the direction a
 * trace can legally leave the pad at fine pitch.
 *
 * <p>Why this exists (measured on the reference fixture): its 100-pin perimeter ring has
 * 5000-unit pitch with 3500-unit pads, so the inter-pad channel is 1500 units against a
 * 2000-unit trace width -- NO trace fits between adjacent pads, and any exit not along the
 * pad's long axis clips a neighbour's clearance. That geometry was the dominant residual
 * failure of the partition router's generic aim-line channels.
 *
 * <p>Classification per component (8+ pins, coordinates clustered to a grid):
 *
 * <ul>
 *   <li><b>BGA_GRID</b>: both grid dimensions >= 4 and mostly filled. Escape axis of a pad
 *       points toward the nearest array boundary (the standard dogbone/escape direction).
 *   <li><b>PERIMETER_RING</b>: grid coordinates with a hollow interior (QFP). Escape axis is
 *       the pad's long axis (pads point away from the body on all four sides).
 *   <li><b>PERIPHERAL_LINES</b>: one or two rows/columns (SOIC, connectors). Escape axis is
 *       the pad's long axis, or the axis perpendicular to the row for square pads.
 * </ul>
 */
public final class PadArrayDetector {

  /** Escape information for one array pad. */
  public record PadEscape(
      /** Unit-ish axis (components in {-1,0,1}); the legal exit line through the pad. */
      int axis_x,
      int axis_y,
      /** Half-extent of the pad along the escape axis, for placing exit points. */
      int half_extent,
      /** True when the axis SIGN is fixed (BGA dogbone, square-pad row exit); false when
       * either end of the axis is a legal exit (ring/peripheral long-axis pads). */
      boolean signed) {
  }

  private final Map<Integer, PadEscape> escape_by_pin_id = new HashMap<>();

  public PadArrayDetector(RoutingBoard p_board) {
    Map<Integer, List<Pin>> pins_by_component = new HashMap<>();
    for (Item item : p_board.get_items()) {
      if (item instanceof Pin pin && pin.get_component_no() > 0) {
        pins_by_component.computeIfAbsent(pin.get_component_no(), k -> new ArrayList<>()).add(pin);
      }
    }
    for (List<Pin> pins : pins_by_component.values()) {
      if (pins.size() < 8) {
        continue;
      }
      classify_component(pins);
    }
  }

  /**
   * The escape geometry of p_pin, or null when the pin is not part of a detected array (free
   * routing applies).
   */
  public PadEscape escape_of(Pin p_pin) {
    return escape_by_pin_id.get(p_pin.get_id_no());
  }

  private void classify_component(List<Pin> p_pins) {
    List<Double> xs_sorted = new ArrayList<>(p_pins.size());
    List<Double> ys_sorted = new ArrayList<>(p_pins.size());
    for (Pin pin : p_pins) {
      FloatPoint center = pin.get_center().to_float();
      xs_sorted.add(center.x);
      ys_sorted.add(center.y);
    }
    xs_sorted.sort(Double::compare);
    ys_sorted.sort(Double::compare);
    List<Double> xs = cluster(xs_sorted);
    List<Double> ys = cluster(ys_sorted);
    double regularity = Math.min(delta_regularity(xs), delta_regularity(ys));
    if (regularity < 0.8) {
      return; // not a regular array; leave its pins to free routing
    }
    double fill = p_pins.size() / (double) Math.max(1, xs.size() * ys.size());
    boolean grid_like = xs.size() >= 4 && ys.size() >= 4;
    boolean bga = grid_like && fill >= 0.45;

    double centroid_x = 0;
    double centroid_y = 0;
    for (Pin pin : p_pins) {
      FloatPoint center = pin.get_center().to_float();
      centroid_x += center.x;
      centroid_y += center.y;
    }
    centroid_x /= p_pins.size();
    centroid_y /= p_pins.size();

    for (Pin pin : p_pins) {
      IntBox pad = pin.get_tile_shape_on_layer(pin.first_layer()).bounding_box();
      int width = pad.ur.x - pad.ll.x;
      int height = pad.ur.y - pad.ll.y;
      FloatPoint center = pin.get_center().to_float();
      int axis_x;
      int axis_y;
      if (bga) {
        // Dogbone direction: toward the nearest array boundary.
        double to_left = center.x - xs.get(0);
        double to_right = xs.get(xs.size() - 1) - center.x;
        double to_bottom = center.y - ys.get(0);
        double to_top = ys.get(ys.size() - 1) - center.y;
        double min_x = Math.min(to_left, to_right);
        double min_y = Math.min(to_bottom, to_top);
        if (min_x <= min_y) {
          axis_x = to_left <= to_right ? -1 : 1;
          axis_y = 0;
        } else {
          axis_x = 0;
          axis_y = to_bottom <= to_top ? -1 : 1;
        }
      } else if (width != height) {
        // Ring / peripheral: exit along the pad's long axis; the sign is decided per route
        // (both ends of the axis may be legal), so record the axis only.
        axis_x = width > height ? 1 : 0;
        axis_y = width > height ? 0 : 1;
      } else {
        // Square pad in a line component: exit perpendicular to the row direction, away from
        // the component centroid.
        boolean row_is_horizontal = xs.size() >= ys.size();
        if (row_is_horizontal) {
          axis_x = 0;
          axis_y = center.y >= centroid_y ? 1 : -1;
        } else {
          axis_x = center.x >= centroid_x ? 1 : -1;
          axis_y = 0;
        }
      }
      int half_extent = axis_x != 0 ? width / 2 : height / 2;
      boolean signed = bga || width == height;
      escape_by_pin_id.put(pin.get_id_no(), new PadEscape(axis_x, axis_y, half_extent, signed));
    }
  }

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

  private static double delta_regularity(List<Double> p_coords) {
    if (p_coords.size() < 3) {
      return 1.0;
    }
    double median = median_delta(p_coords);
    if (median <= 0) {
      return 0;
    }
    int regular = 0;
    for (int i = 1; i < p_coords.size(); i++) {
      if (Math.abs(p_coords.get(i) - p_coords.get(i - 1) - median) <= median * 0.1) {
        ++regular;
      }
    }
    return regular / (double) (p_coords.size() - 1);
  }
}
