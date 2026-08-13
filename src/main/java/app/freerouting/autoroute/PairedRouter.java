package app.freerouting.autoroute;

import app.freerouting.board.DrillItem;
import app.freerouting.board.FixedState;
import app.freerouting.board.Item;
import app.freerouting.board.PolylineTrace;
import app.freerouting.board.RoutingBoard;
import app.freerouting.board.ShapeSearchTree;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntPoint;
import app.freerouting.geometry.planar.Point;
import app.freerouting.geometry.planar.Polyline;
import app.freerouting.rules.Net;
import app.freerouting.settings.RouterSettings;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

/**
 * Paired differential routing with dual offset emission (Phase 5, item 6 of
 * docs/dense-bga-roadmap.md). Behind {@code featureFlags.pairedRouting}, default off.
 *
 * <p>The stage runs once, after fanout and before the first routing pass, so both members of a
 * pair are still unrouted and the pair gets first claim on its corridor. For every
 * name-convention differential pair (the same {@link MeanderMatcher#detect_pairs} detection the
 * meander matcher uses) it does three things, which are the roadmap's own scope statement for
 * this item:
 *
 * <ol>
 *   <li><b>Pair-aware connection selection and terminal fan-in.</b> Each member is decomposed
 *       into its connected components; a pair is routable here when both members have exactly
 *       two (one connection each). The two members' components are matched end-to-end by
 *       centroid distance, and the union of the matched components becomes the start/target set
 *       of ONE centreline search. After the centreline is realized, the terminal of each member
 *       at each end -- the drill item of that member's component nearest the centreline
 *       endpoint, which is the fanout via when there is one and the pad otherwise -- is
 *       connected to the offset body by a 45-degree-legal axis+diagonal fan-in leg.
 *   <li><b>Two-polyline offset emission, both sides validated.</b> The centreline is searched
 *       and materialized at the PAIR half width (pitch/2 plus a member's own half width), so the
 *       partition's windowed channels, the door windows and -- under
 *       {@code featureFlags.liveChannelValidation} -- the compensation erosion are all computed
 *       for a corridor that carries both traces. The realized centreline is then mitre-offset by
 *       +/- pitch/2 into two polylines, and each is validated with the board's own
 *       {@code check_polyline_trace}. The second member is validated AFTER the first is on the
 *       board, so the pair's own gap is checked against real copper by the real DRC rather than
 *       assumed from the pitch arithmetic.
 *   <li><b>Commit coordination.</b> The whole pair sits inside one {@code generate_snapshot} /
 *       {@code undo} bracket: if either member fails validation or insertion, the board is rolled
 *       back to the state before the pair and the classic engine routes both members as usual.
 *       A half-committed pair cannot survive.
 * </ol>
 *
 * <p><b>Pair gap</b> comes from the design rules where they have one: the clearance matrix entry
 * between the two members' trace clearance classes, which is exactly the separation the DRC will
 * enforce between them. Only when that resolves to nothing does the documented default (one
 * trace width) apply; the source is named in the {@code [paired-route]} log line.
 *
 * <p>Committed pair traces are inserted {@code SHOVE_FIXED}: pull-tight would otherwise
 * straighten the two members independently and destroy both the coupling and the length match
 * that paired emission produces. That also makes the commit a standing corridor claim, which is
 * the honest cost of the mechanism.
 *
 * <p>Determinism: pairs in net-name order, components in item-id order, terminals chosen by
 * distance with an id tie-break, no randomness and no iteration over identity-ordered
 * collections.
 *
 * <p><b>Measured v1 limits</b> (docs/dense-bga-roadmap.md, Phase 5 item 6), each reported with a
 * counted reason rather than silently skipped:
 *
 * <ul>
 *   <li><b>No crossover.</b> Both members run at a CONSTANT offset, so a pair whose terminals put
 *       one member on opposite sides of the centreline at its two ends would have to cross; that
 *       is refused. This is what Issue732's pair does under live channel validation.
 *   <li><b>Single layer.</b> The centreline may not need a via, and only layers where all four
 *       components have a terminal are searched. On bm01 the pair's SMD pads were never escaped
 *       by the fanout, which confines the pair to the top layer and leaves no pair-wide corridor.
 *   <li><b>One connection per member.</b> Multi-terminal pairs need a per-connection
 *       correspondence this increment does not build.
 * </ul>
 */
public final class PairedRouter {

  /**
   * Miter cap: at a corner whose turn is sharper than this, the offset corner runs away from the
   * centreline (the miter length is {@code d / sin(angle/2)}). 45-degree geometry needs at most
   * 2.62 (a 135-degree turn); beyond 4 the plan is refused rather than emitted deformed.
   */
  private static final double MAX_MITER_FACTOR = 4.0;

  /**
   * Cushion added to the rule-derived pair pitch, in board units, overridable for measurement
   * with {@code -Dfr.pairedroute.cushion}.
   *
   * <p>Measured necessary, and measured SMALL: at the bare rule pitch (which already carries the
   * clearance matrix's own safety margin) the two members sit exactly at the DRC minimum, and
   * the INTEGER ROUNDING of the mitred offset corners then loses the last unit -- the
   * demonstration fixture rejects at cushion 0 and 1 and commits from 4 upward, with the pair's
   * length mismatch degrading monotonically as the cushion grows (910 at 4, 925 at 16, 1141 at
   * 200, 1843 at 800). The error being rounded is one unit per emitted coordinate and does not
   * scale with board units, so the default is a fixed constant: the board's own clearance
   * safety margin, four times the measured threshold and 15 units of mismatch above the best
   * achievable.
   */
  private static int gap_cushion() {
    return Math.max(0, Integer.getInteger("fr.pairedroute.cushion",
        app.freerouting.rules.ClearanceMatrix.clearance_safety_margin));
  }

  private final RoutingBoard board;
  private final RouterSettings settings;
  private final AutorouteControl.ExpansionCostFactor[] trace_costs;

  /** attempted pairs, committed pairs, search failures, validation rejects, insert rollbacks. */
  private long attempted;
  private long committed;
  private long search_failures;
  private long validation_rejects;
  private long insert_rollbacks;
  private long skipped;

  public PairedRouter(RoutingBoard p_board, RouterSettings p_settings,
      AutorouteControl.ExpansionCostFactor[] p_trace_costs) {
    this.board = p_board;
    this.settings = p_settings;
    this.trace_costs = p_trace_costs;
  }

  /**
   * Counters of the last {@link #route_pairs()} call: attempted, committed, search failures,
   * validation rejects, insert rollbacks, skipped (pairs out of scope).
   */
  public long[] counters() {
    return new long[]{attempted, committed, search_failures, validation_rejects, insert_rollbacks,
        skipped};
  }

  /** Runs the stage over every detected pair; returns one report line per pair. */
  public List<String> route_pairs() {
    List<String> report = new ArrayList<>();
    for (var pair : MeanderMatcher.detect_pairs(board).entrySet()) {
      String prefix = pair.getKey() + "/" + pair.getValue() + ": ";
      try {
        report.add(prefix + route_pair(pair.getKey(), pair.getValue()));
      } catch (Exception e) {
        ++search_failures;
        report.add(prefix + "failed with " + e);
      }
    }
    return report;
  }

  /** One connected component of a member: its drill items and the full item set to lift. */
  private record Component(List<DrillItem> drills, Set<Item> items, FloatPoint centroid) {
  }

  private String route_pair(String p_name, String p_name_partner) {
    Net net_p = single_net(p_name);
    Net net_n = single_net(p_name_partner);
    if (net_p == null || net_n == null) {
      ++skipped;
      return "skipped (not a single-subnet net pair)";
    }
    List<Component> comps_p = components(net_p.net_number);
    List<Component> comps_n = components(net_n.net_number);
    if (comps_p.size() != 2 || comps_n.size() != 2) {
      // v1 scope: one unrouted connection per member. A member already routed has one
      // component; a multi-terminal member has more, and pairing those needs a per-connection
      // correspondence this increment does not build.
      ++skipped;
      return "skipped (components " + comps_p.size() + "/" + comps_n.size() + ", need 2/2)";
    }
    // End-to-end correspondence: match the members' components by centroid distance.
    double straight = comps_p.get(0).centroid.distance(comps_n.get(0).centroid)
        + comps_p.get(1).centroid.distance(comps_n.get(1).centroid);
    double crossed = comps_p.get(0).centroid.distance(comps_n.get(1).centroid)
        + comps_p.get(1).centroid.distance(comps_n.get(0).centroid);
    Component start_p = comps_p.get(0);
    Component target_p = comps_p.get(1);
    Component start_n = crossed < straight ? comps_n.get(1) : comps_n.get(0);
    Component target_n = crossed < straight ? comps_n.get(0) : comps_n.get(1);

    ++attempted;
    AutorouteControl ctrl_p = control_of(net_p.net_number);
    AutorouteControl ctrl_n = control_of(net_n.net_number);
    AutorouteControl pair_ctrl = control_of(net_p.net_number);
    int layer_count = board.get_layer_count();
    int[] pitch = new int[layer_count];
    String gap_source = "rules";
    for (int layer = 0; layer < layer_count; layer++) {
      int hw_p = ctrl_p.trace_half_width[layer];
      int hw_n = ctrl_n.trace_half_width[layer];
      int gap = board.rules.clearance_matrix.get_value(ctrl_p.trace_clearance_class_no,
          ctrl_n.trace_clearance_class_no, layer, true);
      if (gap <= 0) {
        // Documented default: one trace width of separation when the rules carry no clearance
        // between the two members' classes.
        gap = hw_p + hw_n;
        gap_source = "default";
      }
      pitch[layer] = hw_p + gap + hw_n + gap_cushion();
      // The centreline is searched and materialized at the PAIR half width, so every channel,
      // door window and (under live channel validation) compensation erosion is computed for a
      // corridor that has to carry both members.
      pair_ctrl.trace_half_width[layer] = pitch[layer] / 2 + Math.max(hw_p, hw_n);
      // The COMPENSATED pair half width additionally carries the outer clearance. The single-net
      // path can leave that to the pre-insert DRC (a plan that ignores it is simply rejected),
      // but a pair plan is expensive to lose: with the standard UNCOMPENSATED default tree the
      // members' own compensated half width equals their pen half width, so a channel eroded by
      // it puts the outer member's CENTRE against the obstacle -- measured on the demonstration
      // fixture as a reject 14 units short of the wall clearance. The widest clearance this
      // class demands on the layer is what the corridor actually has to keep.
      int outer_clearance = Math.max(
          board.rules.clearance_matrix.max_value(ctrl_p.trace_clearance_class_no, layer),
          board.rules.clearance_matrix.max_value(ctrl_n.trace_clearance_class_no, layer));
      pair_ctrl.compensated_trace_half_width[layer] = pitch[layer] / 2
          + Math.max(ctrl_p.compensated_trace_half_width[layer],
              ctrl_n.compensated_trace_half_width[layer])
          + outer_clearance;
    }

    Set<Item> start_set = new LinkedHashSet<>();
    start_set.addAll(start_p.items);
    start_set.addAll(start_n.items);
    Set<Item> dest_set = new LinkedHashSet<>();
    dest_set.addAll(target_p.items);
    dest_set.addAll(target_n.items);

    ShapeSearchTree tree =
        board.search_tree_manager.get_autoroute_tree(pair_ctrl.trace_clearance_class_no);
    PartitionRouter router = new PartitionRouter(board, tree);
    // Both members' copper is own copper for the pair's corridor: without this the live channel
    // validation would reject every terminal channel on the partner's own pads.
    router.set_pair_nets(new int[]{net_p.net_number, net_n.net_number});
    // A single-layer pair emission can only attach on a layer where ALL FOUR components have a
    // terminal: the pair's own pads on that layer, or the fanout via that reached it. Layers
    // where one member has no terminal are excluded from the search rather than discovered
    // after realization.
    boolean[] pair_layers = new boolean[layer_count];
    List<String> layer_names = new ArrayList<>();
    for (int l = 0; l < layer_count; l++) {
      pair_layers[l] = has_terminal(start_p, l) && has_terminal(target_p, l)
          && has_terminal(start_n, l) && has_terminal(target_n, l);
      if (pair_layers[l]) {
        layer_names.add(String.valueOf(l));
      }
    }
    if (layer_names.isEmpty()) {
      ++search_failures;
      return "no layer carries a terminal of all four pair components";
    }
    PartitionRouter.CellRoute route = router.try_route(start_set, dest_set,
        pair_ctrl.compensated_trace_half_width, true, null, pair_layers);
    if (route == null) {
      ++search_failures;
      return "no pair corridor found (pitch=" + pitch[0] + ", layers=" + layer_names + ")";
    }

    boolean live_validation = app.freerouting.Freerouting.globalSettings != null
        && app.freerouting.Freerouting.globalSettings.featureFlags.liveChannelValidation;
    LocateFoundConnectionAlgo located = null;
    for (int attempt = 0; attempt < (live_validation ? 2 : 1) && located == null; attempt++) {
      MazeSearchAlgo.Result seed =
          router.materialize(route, pair_ctrl, live_validation && attempt == 0);
      if (seed == null) {
        continue;
      }
      located = LocateFoundConnectionAlgo.get_instance(seed, pair_ctrl, tree,
          board.rules.get_trace_angle_restriction(), new TreeSet<>(), null);
      if (located != null && (located.connection_items == null
          || located.connection_items.size() != 1)) {
        // A pair centreline that needs more than one trace item would need a via pair; out of
        // scope for this increment.
        located = null;
      }
    }
    if (located == null) {
      ++search_failures;
      return "centreline not realizable";
    }
    LocateFoundConnectionAlgo.ResultItem centre_item =
        new ArrayList<>(located.connection_items).get(0);
    IntPoint[] centre = centre_item.corners;
    int layer = centre_item.layer;
    if (centre == null || centre.length < 2) {
      ++search_failures;
      return "degenerate centreline";
    }
    // Locate backtracks from the destination, so the realized corner list can run target-first.
    // Orient it start-first before anything reads centre[0] as "the start end" -- with it
    // reversed the fan-in legs attach each member's start pad to the TARGET end of the body,
    // which was measured as a polyline running right across the board and back.
    if (centre[0].to_float().distance(start_p.centroid)
        > centre[centre.length - 1].to_float().distance(start_p.centroid)) {
      IntPoint[] reversed = new IntPoint[centre.length];
      for (int i = 0; i < centre.length; i++) {
        reversed[i] = centre[centre.length - 1 - i];
      }
      centre = reversed;
    }

    int half_pitch = pitch[layer] / 2;
    int last = centre.length - 1;
    IntPoint start_pad_p = terminal_of(start_p, centre[0], layer);
    IntPoint target_pad_p = terminal_of(target_p, centre[last], layer);
    IntPoint start_pad_n = terminal_of(start_n, centre[0], layer);
    IntPoint target_pad_n = terminal_of(target_n, centre[last], layer);
    int side_p = member_side(centre, start_pad_p, target_pad_p);
    int side_n = member_side(centre, start_pad_n, target_pad_n);
    // The centreline is anchored at whichever terminal item the partition attached to, so ONE
    // member can legitimately sit exactly on it at both ends; the partner then fixes the sides.
    if (side_p == 0) {
      side_p = -side_n;
    } else if (side_n == 0) {
      side_n = -side_p;
    }
    if (side_p == 0 || side_p != -side_n) {
      ++search_failures;
      return "members do not straddle the centreline (sides " + side_p + "/" + side_n
          + ", per end +[" + cross_sign(centre[0], centre[1], start_pad_p) + ","
          + cross_sign(centre[last - 1], centre[last], target_pad_p) + "] -["
          + cross_sign(centre[0], centre[1], start_pad_n) + ","
          + cross_sign(centre[last - 1], centre[last], target_pad_n) + "]"
          + ", centreline=" + describe(centre) + ")";
    }

    Polyline poly_p = member_polyline(centre, side_p * half_pitch, start_p, target_p, layer);
    Polyline poly_n = member_polyline(centre, side_n * half_pitch, start_n, target_n, layer);
    if (poly_p == null || poly_n == null) {
      ++validation_rejects;
      return "offset emission degenerate (" + member_failure + ", half_pitch=" + half_pitch
          + ", centreline=" + describe(centre) + ")";
    }

    // Commit coordination: both members inside one snapshot bracket. The second member is
    // validated with the first already on the board, so the DRC checks the real pair gap.
    board.generate_snapshot();
    PolylineTrace trace_p = insert_member(poly_p, layer, ctrl_p, net_p.net_number);
    if (trace_p == null) {
      board.undo(null);
      ++validation_rejects;
      return "member " + p_name + " rejected (layer " + layer + ", pitch " + pitch[layer] + "): "
          + explain(poly_p, layer, ctrl_p, net_p.net_number);
    }
    PolylineTrace trace_n = insert_member(poly_n, layer, ctrl_n, net_n.net_number);
    if (trace_n == null) {
      String reason = explain(poly_n, layer, ctrl_n, net_n.net_number);
      board.undo(null);
      ++insert_rollbacks;
      return "member " + p_name_partner + " rejected; PAIR ROLLED BACK (layer " + layer
          + ", pitch " + pitch[layer] + "): " + reason
          + " partner_corners=" + describe_polyline(poly_p);
    }
    double len_p = trace_p.get_length();
    double len_n = trace_n.get_length();
    board.pop_snapshot();
    ++committed;
    return "committed layer=" + layer + " pitch=" + pitch[layer] + " (" + gap_source + ")"
        + " corners=" + centre.length
        + " len+=" + Math.round(len_p) + " len-=" + Math.round(len_n)
        + " mismatch=" + Math.round(Math.abs(len_p - len_n));
  }

  /**
   * Validates one member's polyline against the live board and inserts it SHOVE_FIXED when
   * legal. Returns null when the DRC refuses it or the insert fails; the caller rolls the pair
   * back.
   */
  private PolylineTrace insert_member(Polyline p_polyline, int p_layer, AutorouteControl p_ctrl,
      int p_net_no) {
    int half_width = p_ctrl.trace_half_width[p_layer];
    int[] net_arr = new int[]{p_net_no};
    boolean legal;
    try {
      legal = board.check_polyline_trace(p_polyline, p_layer, half_width, net_arr,
          p_ctrl.trace_clearance_class_no);
    } catch (Exception e) {
      legal = false;
    }
    if (!legal) {
      return null;
    }
    return board.insert_trace_without_cleaning(p_polyline, p_layer, half_width, net_arr,
        p_ctrl.trace_clearance_class_no, FixedState.SHOVE_FIXED);
  }

  /**
   * The board's own reject explanation for a member the DRC refused -- the same read-only
   * diagnostic the materialization rejects use, so a paired reject can be named rather than
   * counted.
   */
  private String explain(Polyline p_polyline, int p_layer, AutorouteControl p_ctrl, int p_net_no) {
    try {
      return board.explain_polyline_trace_reject(p_polyline, p_layer,
          p_ctrl.trace_half_width[p_layer], new int[]{p_net_no}, p_ctrl.trace_clearance_class_no)
          + " corners=" + describe_polyline(p_polyline);
    } catch (Exception e) {
      return "insert failed (" + e + ")";
    }
  }

  private static String describe_polyline(Polyline p_polyline) {
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < p_polyline.corner_count(); i++) {
      FloatPoint corner = p_polyline.corner(i).to_float();
      sb.append(sb.isEmpty() ? "" : " ").append(Math.round(corner.x)).append(',')
          .append(Math.round(corner.y));
    }
    return sb.toString();
  }

  /**
   * One member's emitted polyline: the centreline offset by p_offset (signed, left-positive),
   * with the offset body's first and last corners replaced by 45-degree-legal fan-in legs to
   * that member's own terminals.
   */
  private String member_failure = "";

  /** Compact centreline dump for the reject diagnostics. */
  private static String describe(IntPoint[] p_corners) {
    StringBuilder sb = new StringBuilder();
    for (IntPoint corner : p_corners) {
      sb.append(sb.isEmpty() ? "" : " ").append(corner.x).append(',').append(corner.y);
    }
    return sb.toString();
  }

  private Polyline member_polyline(IntPoint[] p_centre, int p_offset, Component p_start,
      Component p_target, int p_layer) {
    IntPoint[] body = offset_polyline(p_centre, p_offset);
    if (body == null) {
      member_failure = "miter";
      return null;
    }
    IntPoint start_pad = terminal_of(p_start, p_centre[0], p_layer);
    IntPoint target_pad = terminal_of(p_target, p_centre[p_centre.length - 1], p_layer);
    if (start_pad == null || target_pad == null) {
      member_failure = "no terminal on layer " + p_layer;
      return null;
    }
    List<Point> corners = new ArrayList<>();
    // Interior of the offset body: its own ends are the offset images of the centreline's
    // terminal corners, which sit beside the pads rather than on them; the fan-in legs replace
    // them.
    int first_interior = body.length > 2 ? 1 : 0;
    int last_interior = body.length > 2 ? body.length - 2 : body.length - 1;
    corners.add(start_pad);
    IntPoint mid_start = fan_in_corner(start_pad, body[first_interior]);
    if (mid_start != null) {
      corners.add(mid_start);
    }
    for (int i = first_interior; i <= last_interior; i++) {
      corners.add(body[i]);
    }
    IntPoint mid_target = fan_in_corner(target_pad, body[last_interior]);
    if (mid_target != null) {
      corners.add(mid_target);
    }
    corners.add(target_pad);
    // Drop duplicated consecutive corners; Polyline collapses them anyway, but an exact
    // duplicate at a fan-in join makes the collapse check below ambiguous.
    List<Point> cleaned = new ArrayList<>();
    for (Point corner : corners) {
      if (cleaned.isEmpty() || !cleaned.get(cleaned.size() - 1).equals(corner)) {
        cleaned.add(corner);
      }
    }
    if (cleaned.size() < 2) {
      member_failure = "fewer than two corners";
      return null;
    }
    try {
      Polyline result = new Polyline(cleaned.toArray(new Point[0]));
      if (result.corner_count() < 2) {
        member_failure = "polyline collapsed to " + result.corner_count() + " corners";
        return null;
      }
      return result;
    } catch (Exception e) {
      member_failure = "polyline construction: " + e;
      return null;
    }
  }

  /**
   * The intermediate corner of a 45-degree-legal two-segment fan-in from a pad to the offset
   * body: one axis-parallel leg followed by one exact diagonal. Null when the pad already lies
   * on a diagonal or axis through the body corner, so a single segment suffices.
   */
  static IntPoint fan_in_corner(IntPoint p_pad, IntPoint p_body) {
    long dx = (long) p_body.x - p_pad.x;
    long dy = (long) p_body.y - p_pad.y;
    long adx = Math.abs(dx);
    long ady = Math.abs(dy);
    if (adx == ady || adx == 0 || ady == 0) {
      return null;
    }
    if (adx > ady) {
      return new IntPoint((int) (p_pad.x + Long.signum(dx) * (adx - ady)), p_pad.y);
    }
    return new IntPoint(p_pad.x, (int) (p_pad.y + Long.signum(dy) * (ady - adx)));
  }

  /**
   * The centreline offset perpendicular by p_offset (left-positive), mitred at the corners.
   * Returns null when a corner's miter runs away (a turn sharper than {@link #MAX_MITER_FACTOR}
   * allows) or the centreline has a degenerate segment.
   */
  static IntPoint[] offset_polyline(IntPoint[] p_centre, int p_offset) {
    int count = p_centre.length;
    double[] nx = new double[count - 1];
    double[] ny = new double[count - 1];
    for (int i = 0; i + 1 < count; i++) {
      double dx = (double) p_centre[i + 1].x - p_centre[i].x;
      double dy = (double) p_centre[i + 1].y - p_centre[i].y;
      double length = Math.hypot(dx, dy);
      if (length == 0) {
        return null;
      }
      nx[i] = -dy / length;
      ny[i] = dx / length;
    }
    IntPoint[] result = new IntPoint[count];
    result[0] = round(p_centre[0].x + p_offset * nx[0], p_centre[0].y + p_offset * ny[0]);
    result[count - 1] = round(p_centre[count - 1].x + p_offset * nx[count - 2],
        p_centre[count - 1].y + p_offset * ny[count - 2]);
    for (int i = 1; i + 1 < count; i++) {
      double mx = nx[i - 1] + nx[i];
      double my = ny[i - 1] + ny[i];
      double denominator = 1 + (nx[i - 1] * nx[i] + ny[i - 1] * ny[i]);
      if (denominator <= 1e-9) {
        return null; // the centreline doubles back; no sound miter exists
      }
      mx /= denominator;
      my /= denominator;
      if (Math.hypot(mx, my) > MAX_MITER_FACTOR) {
        return null;
      }
      result[i] = round(p_centre[i].x + p_offset * mx, p_centre[i].y + p_offset * my);
    }
    return result;
  }

  private static IntPoint round(double p_x, double p_y) {
    return new IntPoint((int) Math.round(p_x), (int) Math.round(p_y));
  }

  /**
   * The side of the centreline a member runs on: +1 left, -1 right, 0 undetermined. Read at the
   * start terminal against the first segment (the one its fan-in leaves along) and at the
   * target terminal against the last segment; a member whose two ends disagree would have to
   * CROSS the centreline, which this emission cannot do, so it reports 0 and the pair is
   * refused.
   */
  private static int member_side(IntPoint[] p_centre, IntPoint p_start, IntPoint p_target) {
    int last = p_centre.length - 1;
    int at_start = cross_sign(p_centre[0], p_centre[1], p_start);
    int at_target = cross_sign(p_centre[last - 1], p_centre[last], p_target);
    if (at_start != 0 && at_target != 0 && at_start != at_target) {
      return 0;
    }
    return at_start != 0 ? at_start : at_target;
  }

  /** Sign of the cross product of (p_b - p_a) with (p_point - p_a); 0 when collinear. */
  private static int cross_sign(IntPoint p_a, IntPoint p_b, IntPoint p_point) {
    if (p_point == null) {
      return 0;
    }
    double ux = (double) p_b.x - p_a.x;
    double uy = (double) p_b.y - p_a.y;
    double wx = (double) p_point.x - p_a.x;
    double wy = (double) p_point.y - p_a.y;
    double cross = ux * wy - uy * wx;
    if (cross > 0) {
      return 1;
    }
    return cross < 0 ? -1 : 0;
  }

  /**
   * The drill item of a component that the member should attach to at one end: the one on the
   * route's layer nearest the centreline endpoint (the fanout via when there is one, the pad
   * otherwise). Deterministic: distance with an item-id tie-break.
   */
  private IntPoint terminal_of(Component p_component, IntPoint p_near, int p_layer) {
    DrillItem best = null;
    double best_distance = Double.MAX_VALUE;
    for (DrillItem drill : p_component.drills) {
      if (!drill.is_on_layer(p_layer)) {
        continue;
      }
      double distance = drill.get_center().to_float().distance(p_near.to_float());
      if (best == null || distance < best_distance
          || (distance == best_distance && drill.get_id_no() < best.get_id_no())) {
        best = drill;
        best_distance = distance;
      }
    }
    return best == null ? null : best.get_center().to_float().round();
  }

  /** Whether the component has a drill item the emitted trace can attach to on this layer. */
  private static boolean has_terminal(Component p_component, int p_layer) {
    for (DrillItem drill : p_component.drills) {
      if (drill.is_on_layer(p_layer)) {
        return true;
      }
    }
    return false;
  }

  /** The single net of this name, or null when the name maps to zero or several subnets. */
  private Net single_net(String p_name) {
    Collection<Net> nets = board.rules.nets.get(p_name);
    if (nets == null || nets.size() != 1) {
      return null;
    }
    return nets.iterator().next();
  }

  private AutorouteControl control_of(int p_net_no) {
    return new AutorouteControl(board, p_net_no, settings, settings.get_via_costs(), trace_costs);
  }

  /**
   * The net's connected components, as drill items plus the full item set of each component
   * (the set the partition search lifts). Item-id order throughout.
   */
  private List<Component> components(int p_net_no) {
    List<DrillItem> drills = new ArrayList<>();
    for (Item item : board.get_connectable_items(p_net_no)) {
      if (item instanceof DrillItem drill) {
        drills.add(drill);
      }
    }
    drills.sort((a, b) -> Integer.compare(a.get_id_no(), b.get_id_no()));
    List<Component> result = new ArrayList<>();
    Set<Integer> assigned = new TreeSet<>();
    for (DrillItem drill : drills) {
      if (assigned.contains(drill.get_id_no())) {
        continue;
      }
      Set<Item> connected = drill.get_connected_set(p_net_no);
      List<DrillItem> component_drills = new ArrayList<>();
      Set<Item> items = new LinkedHashSet<>();
      List<Item> ordered = new ArrayList<>(connected);
      ordered.sort((a, b) -> Integer.compare(a.get_id_no(), b.get_id_no()));
      double sum_x = 0;
      double sum_y = 0;
      for (Item item : ordered) {
        items.add(item);
        if (item instanceof DrillItem component_drill) {
          component_drills.add(component_drill);
          assigned.add(component_drill.get_id_no());
          FloatPoint centre = component_drill.get_center().to_float();
          sum_x += centre.x;
          sum_y += centre.y;
        }
      }
      if (component_drills.isEmpty()) {
        continue;
      }
      result.add(new Component(component_drills, items,
          new FloatPoint(sum_x / component_drills.size(), sum_y / component_drills.size())));
    }
    return result;
  }

  /**
   * Total routed length of a net by name, summed over its traces -- the same measure the
   * meander matcher matches on, exposed so the paired stage can report the pair's mismatch at
   * the end of a run.
   */
  public static double net_length(RoutingBoard p_board, String p_net_name) {
    return MeanderMatcher.net_length(p_board, p_net_name);
  }
}
