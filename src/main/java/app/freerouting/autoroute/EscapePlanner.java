package app.freerouting.autoroute;

import app.freerouting.board.ConductionArea;
import app.freerouting.board.FixedState;
import app.freerouting.board.Item;
import app.freerouting.board.Pin;
import app.freerouting.board.RoutingBoard;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntBox;
import app.freerouting.geometry.planar.IntPoint;
import app.freerouting.geometry.planar.Polyline;
import app.freerouting.logger.FRLogger;
import app.freerouting.rules.NetClass;
import app.freerouting.rules.ViaInfo;
import java.util.ArrayList;
import java.util.List;

/**
 * Deterministic escape pre-routing for SMD pins the fanout stage left contactless (Phase 2
 * of docs/dense-bga-roadmap.md, v1): for each such pin, find the nearest via position that
 * is legal for the smallest via whose span covers the pin's layer, verify the dogbone stub
 * from the pad to that position is insertable, and place both as ordinary (rip-uppable)
 * items. Later routing then has all-layer access at every escaped pad.
 *
 * <p>Feasibility semantics follow the measured corrections behind
 * {@link PadArrayDetector#escape_spot_exists}: only the via's span layers are checked and
 * non-obstacle conduction areas (copper pours, which reflow) are passable. Candidates are
 * tried nearest-first, so stubs stay short. Pins are processed most-constrained-first
 * within a single pass would be the next refinement; v1 measures plain board order.
 */
public final class EscapePlanner {

  private EscapePlanner() {
  }

  /**
   * Plans and inserts escapes for every unescaped single-net SMD pin. Returns the number of
   * escapes inserted.
   */
  public static int plan_escapes(RoutingBoard p_board) {
    int inserted = 0;
    List<Pin> candidates = new ArrayList<>();
    for (Item item : p_board.get_items()) {
      if (item instanceof Pin pin && pin.first_layer() == pin.last_layer()
          && pin.net_count() == 1 && pin.get_normal_contacts().isEmpty()) {
        candidates.add(pin);
      }
    }
    // Escape-axis inputs (bm04 diagnosis): nearest-legal-spot placement put ~half the
    // dogbones ALONG the pad rows -- into the very ring channels the endgame needs
    // (972.02/4 vs the 979.01/3 baseline). The detector's per-pad axis plus the component
    // centroid (for the outward sign on unsigned ring axes) orders candidates
    // outward-along-axis first instead.
    PadArrayDetector detector = new PadArrayDetector(p_board);
    java.util.Map<Integer, double[]> component_centroids = new java.util.HashMap<>();
    for (Item item : p_board.get_items()) {
      if (item instanceof Pin pin) {
        double[] acc = component_centroids.computeIfAbsent(pin.get_component_no(),
            k -> new double[3]);
        FloatPoint pin_center = pin.get_center().to_float();
        acc[0] += pin_center.x;
        acc[1] += pin_center.y;
        acc[2] += 1;
      }
    }
    // Threshold calibration data (logged, not yet used): the absolute 150k gate was tuned
    // on one board, so record the board scale and the candidate airline distribution to
    // decide whether a board-relative rule reproduces it.
    var board_box = p_board.get_bounding_box();
    double board_diagonal = Math.hypot(board_box.ur.x - (double) board_box.ll.x,
        board_box.ur.y - (double) board_box.ll.y);
    List<Double> airlines = new ArrayList<>();
    for (Pin pin : candidates) {
      double airline = nearest_unconnected_airline(pin);
      if (airline != Double.MAX_VALUE) {
        airlines.add(airline);
      }
    }
    airlines.sort(Double::compare);
    double median_airline = airlines.isEmpty() ? 0
        : airlines.get(airlines.size() / 2);
    // Board-relative threshold (measured calibration): the absolute 150k gate corresponds to
    // wildly different board fractions (0.06 to 0.24 of the diagonal across the corpus), so
    // the diagonal is the wrong normalizer. Against the MEDIAN candidate airline the tuned
    // constant is nearly invariant on the two boards where escapes pay -- 2.09x on
    // Issue732, 2.32x on caniot-tiny-arm -- so that ratio is the scale-free rule.
    // Two board-relative terms, both required (each alone was measured to fail):
    //  - the median multiple sets the bar relative to what THIS board's nets look like, but
    //    collapses on boards whose median airline is near zero (Natural_Tone_Preamp: median
    //    2277 -> a 5009 gate, 23 escapes, 637.61/79 -> 633.02/80, a measured regression);
    //  - the diagonal fraction is a scale-free floor that keeps such boards sane, but alone
    //    it cannot reproduce the tuned behaviour (150k is 0.06 to 0.24 of the diagonal
    //    across the corpus).
    double threshold = Math.max(MEDIAN_AIRLINE_MULTIPLE * median_airline,
        DIAGONAL_FLOOR_FRACTION * board_diagonal);
    FRLogger.info("[escape-planner] candidates=" + candidates.size()
        + " board_diagonal=" + (long) board_diagonal
        + " median_airline=" + (long) median_airline
        + " max_airline=" + (airlines.isEmpty() ? 0 : (long) (double) airlines.get(airlines.size() - 1))
        + " threshold=" + (long) threshold
        + " legacy_threshold=" + NEEDED_AIRLINE
        + " diagonal_fraction=" + String.format("%.4f", threshold / Math.max(1, board_diagonal)));
    // Stagger lanes: adjacent pads of one component alternate between a near and a far
    // outward lane, so their dogbone vias cannot line up into the straight wall measured on
    // bm04 (four vias at one identical offset along the west channel). Lane assignment is
    // deterministic -- pins sorted by id within their component.
    java.util.Map<Integer, Integer> stagger_lane = new java.util.HashMap<>();
    java.util.Map<Integer, List<Pin>> by_component = new java.util.TreeMap<>();
    for (Pin pin : candidates) {
      by_component.computeIfAbsent(pin.get_component_no(), k -> new ArrayList<>()).add(pin);
    }
    for (var entry : by_component.entrySet()) {
      List<Pin> component_pins = entry.getValue();
      component_pins.sort((a, b) -> Integer.compare(a.get_id_no(), b.get_id_no()));
      for (int i = 0; i < component_pins.size(); i++) {
        stagger_lane.put(component_pins.get(i).get_id_no(), i % 2);
      }
    }
    List<Item> planned_items = new ArrayList<>();
    for (Pin pin : candidates) {
      if (!escape_needed(pin, threshold)) {
        continue;
      }
      if (plan_one(p_board, pin, planned_items, detector,
          component_centroids.get(pin.get_component_no()),
          stagger_lane.getOrDefault(pin.get_id_no(), 0))) {
        ++inserted;
      }
    }
    for (Item fresh : planned_items) {
      for (var violation : fresh.clearance_violations()) {
        FRLogger.info("[escape-planner] DIRTY " + fresh.getClass().getSimpleName()
            + "#" + fresh.get_id_no()
            + " net=" + (fresh.net_count() > 0 ? fresh.get_net_no(0) : -1) + " vs "
            + violation.second_item.getClass().getSimpleName()
            + "#" + violation.second_item.get_id_no()
            + " net=" + (violation.second_item.net_count() > 0 ? violation.second_item.get_net_no(0) : -1)
            + " layer=" + violation.layer);
      }
    }
    return inserted;
  }

  /**
   * The needs filter (bm04 diagnosis): every one of the planner's escapes there served a
   * net the classic engine routes fine on the surface, and each cost a full-span via --
   * the escaped board lost a net (972.02/4 vs 979.01/3) with ZERO of the escapes going to
   * the hard nets. A pin needs an escape only when its connection is long enough that
   * surface routing from the pad is at risk: the airline to the nearest unconnected
   * same-net item must reach the campaign's long-connection threshold.
   */
  private static final int NEEDED_AIRLINE = 150000;
  /**
   * Board-relative escape gate: a pin needs an escape when its airline is this multiple of
   * the board's median candidate airline. Calibrated (not guessed) from the two boards
   * where escapes pay -- the tuned 150k constant equals 2.09x median there and 2.32x on
   * caniot-tiny-arm -- and it makes the rule scale-free instead of fixture-tuned.
   */
  private static final double MEDIAN_AIRLINE_MULTIPLE = 2.2;
  /**
   * Scale-free floor under the median rule: a pin never counts as long-haul below this
   * fraction of the board diagonal. Calibrated so the two escape wins keep their placements
   * (Issue732 keeps the median term at 158064; caniot lands at 145666, both firing as
   * before) while degenerate-median boards stop over-firing.
   */
  private static final double DIAGONAL_FLOOR_FRACTION = 0.15;

  private static boolean escape_needed(Pin p_pin, double p_threshold) {
    double nearest = nearest_unconnected_airline(p_pin);
    return nearest != Double.MAX_VALUE && nearest >= p_threshold;
  }

  /** Distance to the nearest unconnected same-net item, or MAX_VALUE when there is none. */
  private static double nearest_unconnected_airline(Pin p_pin) {
    FloatPoint center = p_pin.get_center().to_float();
    double nearest = Double.MAX_VALUE;
    for (Item other : p_pin.get_unconnected_set(p_pin.get_net_no(0))) {
      if (other instanceof ConductionArea) {
        continue; // reflowable planes reach everywhere; not a routing target distance
      }
      FloatPoint other_center;
      if (other instanceof app.freerouting.board.DrillItem drill) {
        other_center = drill.get_center().to_float();
      } else {
        var box = other.bounding_box();
        other_center = new FloatPoint((box.ll.x + box.ur.x) / 2.0, (box.ll.y + box.ur.y) / 2.0);
      }
      nearest = Math.min(nearest, Math.hypot(other_center.x - center.x, other_center.y - center.y));
    }
    return nearest;
  }

  private static boolean plan_one(RoutingBoard p_board, Pin p_pin, List<Item> p_inserted,
      PadArrayDetector p_detector, double[] p_centroid, int p_stagger_lane) {
    int layer = p_pin.first_layer();
    int net = p_pin.get_net_no(0);
    // Smallest via whose span covers the pin's layer.
    ViaInfo best_via = null;
    int via_radius = Integer.MAX_VALUE;
    for (int v = 0; v < p_board.rules.via_infos.count(); v++) {
      ViaInfo info = p_board.rules.via_infos.get(v);
      var padstack = info.get_padstack();
      if (padstack.from_layer() > layer || padstack.to_layer() < layer) {
        continue;
      }
      IntBox via_box = padstack.get_shape(padstack.from_layer()).bounding_box();
      int radius = (via_box.ur.x - via_box.ll.x) / 2;
      if (radius < via_radius) {
        via_radius = radius;
        best_via = info;
      }
    }
    if (best_via == null) {
      return false;
    }
    var net_data = p_board.rules.nets.get(net);
    NetClass net_class = net_data == null ? null : net_data.get_class();
    if (net_class == null) {
      return false;
    }
    int half_width = net_class.get_trace_half_width(layer);
    int clearance_class = net_class.get_trace_clearance_class();
    var padstack = best_via.get_padstack();

    FloatPoint center = p_pin.get_center().to_float();
    IntBox pad = p_pin.get_tile_shape_on_layer(layer).bounding_box();
    int pitch_guess = Math.max(pad.ur.x - pad.ll.x, pad.ur.y - pad.ll.y);
    int reach = 3 * pitch_guess;
    int step = Math.max(500, pitch_guess / 4);

    List<int[]> spots = new ArrayList<>();
    for (int dx = -reach; dx <= reach; dx += step) {
      for (int dy = -reach; dy <= reach; dy += step) {
        if (dx != 0 || dy != 0) {
          spots.add(new int[]{dx, dy});
        }
      }
    }
    // Candidate order: outward along the detector's escape axis first (outward half-plane,
    // then smallest perpendicular offset from the axis, then nearest); plain nearest-first
    // when the pad has no detected axis.
    PadArrayDetector.PadEscape escape = p_detector == null ? null : p_detector.escape_of(p_pin);
    int axis_x = 0;
    int axis_y = 0;
    if (escape != null && (escape.axis_x() != 0 || escape.axis_y() != 0)) {
      axis_x = escape.axis_x();
      axis_y = escape.axis_y();
      if (!escape.signed() && p_centroid != null && p_centroid[2] > 0) {
        double out_x = center.x - p_centroid[0] / p_centroid[2];
        double out_y = center.y - p_centroid[1] / p_centroid[2];
        if (axis_x * out_x + axis_y * out_y < 0) {
          axis_x = -axis_x;
          axis_y = -axis_y;
        }
      }
    }
    final int ax = axis_x;
    final int ay = axis_y;
    if (ax == 0 && ay == 0) {
      spots.sort((a, b) -> Long.compare(
          (long) a[0] * a[0] + (long) a[1] * a[1],
          (long) b[0] * b[0] + (long) b[1] * b[1]));
    } else {
      final long lane_boundary = (long) step * 2;
      spots.sort((a, b) -> {
        long proj_a = (long) a[0] * ax + (long) a[1] * ay;
        long proj_b = (long) b[0] * ax + (long) b[1] * ay;
        int outward = Boolean.compare(proj_a <= 0, proj_b <= 0); // outward (proj > 0) first
        if (outward != 0) {
          return outward;
        }
        // Preferred lane first (near lane for even pins, far lane for odd), so neighbouring
        // pads of one component do not settle at the same outward distance.
        boolean far_a = proj_a > lane_boundary;
        boolean far_b = proj_b > lane_boundary;
        boolean want_far = p_stagger_lane == 1;
        if (far_a != far_b) {
          return far_a == want_far ? -1 : 1;
        }
        long perp_a = Math.abs((long) a[0] * -ay + (long) a[1] * ax);
        long perp_b = Math.abs((long) b[0] * -ay + (long) b[1] * ax);
        if (perp_a != perp_b) {
          return Long.compare(perp_a, perp_b);
        }
        // Nearest out (measured: farthest-out scored 965.03/5 vs 972.02/4 -- the longer
        // stubs consume more corridor than the via wall they avoid).
        return Long.compare((long) a[0] * a[0] + (long) a[1] * a[1],
            (long) b[0] * b[0] + (long) b[1] * b[1]);
      });
    }

    for (int[] spot : spots) {
      int cx = (int) center.x + spot[0];
      int cy = (int) center.y + spot[1];
      if (!via_spot_legal(p_board, p_pin, cx, cy, via_radius,
          padstack.from_layer(), padstack.to_layer(), best_via.get_clearance_class(),
          best_via.attach_smd_allowed())) {
        continue;
      }
      IntPoint via_location = new IntPoint(cx, cy);
      Polyline stub = new Polyline(new IntPoint[]{
          p_pin.get_center().to_float().round(), via_location});
      boolean stub_legal;
      try {
        stub_legal = p_board.check_polyline_trace(stub, layer, half_width,
            new int[]{net}, clearance_class);
      } catch (Exception e) {
        stub_legal = false;
      }
      if (!stub_legal) {
        continue;
      }
      FRLogger.info("[escape-planner] escape pin=" + p_pin.get_id_no() + " net=" + net
          + " pad=(" + (int) center.x + "," + (int) center.y + ")"
          + " via=(" + cx + "," + cy + ") offset=(" + spot[0] + "," + spot[1] + ")");
      var via_item = p_board.insert_via(padstack, via_location, new int[]{net},
          best_via.get_clearance_class(), FixedState.UNFIXED, best_via.attach_smd_allowed());
      var stub_item = p_board.insert_trace_without_cleaning(stub, layer, half_width, new int[]{net},
          clearance_class, FixedState.UNFIXED);
      if (p_inserted != null) {
        if (via_item != null) {
          p_inserted.add(via_item);
        }
        if (stub_item != null) {
          p_inserted.add(stub_item);
        }
      }
      return true;
    }
    return false;
  }

  private static boolean via_spot_legal(RoutingBoard p_board, Pin p_pin, int p_cx, int p_cy,
      int p_radius, int p_from_layer, int p_to_layer, int p_clearance_class,
      boolean p_attach_allowed) {
    var tree = p_board.search_tree_manager.get_default_tree();
    IntBox via_box = new IntBox(p_cx - p_radius, p_cy - p_radius, p_cx + p_radius, p_cy + p_radius);
    if (!via_box.is_contained_in(p_board.get_bounding_box())) {
      return false;
    }
    for (int layer = p_from_layer; layer <= p_to_layer; layer++) {
      for (var entry : tree.overlapping_tree_entries_with_clearance(via_box, layer, new int[0], p_clearance_class)) {
        if (!(entry.object instanceof Item blocking)) {
          continue;
        }
        if (blocking.shares_net(p_pin)) {
          // Same-net copper is passable EXCEPT pads when the via lacks attach permission:
          // Via.is_obstacle enforces the attach rule without a same-net exemption, so a
          // no-attach via too close to its OWN pad is a scored DRC pair (measured: all 24
          // added pairs were exactly this, planner via vs its own pin).
          if (!(blocking instanceof Pin) || p_attach_allowed) {
            continue;
          }
        }
        // Bidirectional obstacle semantics (measured): a via inside a foreign pour is clean
        // from the via's side but the POUR's directional DRC counts it, and the score counts
        // the pour's side (+12 scored pairs). Under pour-reflow modelling the pour no longer
        // counts such pairs, so placement inside reflowable pours becomes legal again.
        if (blocking instanceof ConductionArea pour && !pour.get_is_obstacle()
            && app.freerouting.Freerouting.globalSettings != null
            && app.freerouting.Freerouting.globalSettings.featureFlags.reflowablePours) {
          continue;
        }
        return false;
      }
    }
    return true;
  }
}
