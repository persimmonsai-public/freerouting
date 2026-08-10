package app.freerouting.autoroute;

import app.freerouting.board.FixedState;
import app.freerouting.board.Item;
import app.freerouting.board.PolylineTrace;
import app.freerouting.board.RoutingBoard;
import app.freerouting.geometry.planar.IntPoint;
import app.freerouting.geometry.planar.Point;
import app.freerouting.geometry.planar.Polyline;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * Post-route differential-pair length matching by meander insertion (Phase 5, item 5 of
 * docs/dense-bga-roadmap.md). Behind {@code featureFlags.meanderMatching}, default off.
 *
 * <p>Pairs are detected by net-name convention (the same rules as the probe's Phase-5
 * detection: {@code _P/_N}, {@code +/-}, {@code <digit>P/<digit>N}). For each pair with a
 * routed-length mismatch above {@link #MIN_DEFICIT}, 45-degree triangle-wave meanders are
 * inserted into straight axis-parallel segments of the SHORTER member's traces. Every
 * candidate polyline is validated with the board's own insertability predicate
 * ({@code check_polyline_trace}, the DRC the router itself uses) before the original trace
 * is replaced; the replacement is inserted SHOVE_FIXED so the pull-tight optimizer does not
 * straighten the meander away.
 *
 * <p>Determinism: pairs are processed in net-name order, traces in id order, bump side
 * +lateral before -lateral, amplitudes descending. No randomness, no iteration over
 * identity-ordered collections.
 */
public final class MeanderMatcher {

  /** Mismatch below this many board units is considered matched (2000 = 0.2 mm here). */
  private static final int MIN_DEFICIT = 2000;

  private MeanderMatcher() {
  }

  /**
   * Detects diff pairs by net-name convention. Returns positive-name -> negative-name.
   */
  public static Map<String, String> detect_pairs(RoutingBoard p_board) {
    Map<String, String> result = new TreeMap<>();
    for (int n = 1; n <= p_board.rules.nets.max_net_no(); n++) {
      var net = p_board.rules.nets.get(n);
      if (net == null) {
        continue;
      }
      String name = net.name;
      String partner = null;
      if (name.endsWith("_P")) {
        partner = name.substring(0, name.length() - 2) + "_N";
      } else if (name.endsWith("+")) {
        partner = name.substring(0, name.length() - 1) + "-";
      } else if (name.endsWith("P") && name.length() > 1 && Character.isDigit(name.charAt(name.length() - 2))) {
        partner = name.substring(0, name.length() - 1) + "N";
      }
      if (partner != null) {
        var partners = p_board.rules.nets.get(partner);
        if (partners != null && !partners.isEmpty()) {
          result.put(name, partner);
        }
      }
    }
    return result;
  }

  /**
   * Runs the matcher over every detected pair. Returns a per-pair report of the form
   * {@code name+ / name-: mismatch before -> after (bumps inserted)}.
   */
  public static List<String> match(RoutingBoard p_board) {
    List<String> report = new ArrayList<>();
    Map<String, String> pairs = detect_pairs(p_board);
    for (var pair : pairs.entrySet()) {
      double len_p = net_length(p_board, pair.getKey());
      double len_n = net_length(p_board, pair.getValue());
      double before = Math.abs(len_p - len_n);
      if (before < MIN_DEFICIT || len_p == 0 || len_n == 0) {
        continue; // matched already, or a member is unrouted -- nothing sound to do
      }
      int bumps = 0;
      double deficit = before;
      // Traces this run inserted (SHOVE_FIXED): still legitimate meander candidates for
      // their remaining straight segments, unlike traces fixed by anyone else.
      java.util.Set<Integer> matcher_owned = new java.util.TreeSet<>();
      // Insert until matched or no candidate segment accepts a meander any more. The
      // SHORTER member is recomputed every iteration and per-call gains are floored to the
      // remaining deficit: with a fixed member and ceil-rounded gains, a single overshoot
      // flipped which member was shorter and the loop then LENGTHENED the longer member,
      // diverging (measured on 8-layer: 37112 -> 43435 across 62 bumps).
      for (int attempt = 0; attempt < 16 && deficit >= MIN_DEFICIT; attempt++) {
        double len_p_now = net_length(p_board, pair.getKey());
        double len_n_now = net_length(p_board, pair.getValue());
        String shorter = len_p_now < len_n_now ? pair.getKey() : pair.getValue();
        int inserted = insert_meander(p_board, shorter, deficit, matcher_owned);
        if (inserted == 0) {
          break;
        }
        bumps += inserted;
        deficit = Math.abs(net_length(p_board, pair.getKey()) - net_length(p_board, pair.getValue()));
      }
      report.add(pair.getKey() + "/" + pair.getValue() + ": mismatch " + (long) before
          + " -> " + (long) deficit + " (" + bumps + " bumps)");
    }
    return report;
  }

  private static double net_length(RoutingBoard p_board, String p_net_name) {
    double result = 0;
    for (var net : p_board.rules.nets.get(p_net_name)) {
      for (Item item : p_board.get_items()) {
        if (item instanceof app.freerouting.board.Trace trace && trace.contains_net(net.net_number)) {
          result += trace.get_length();
        }
      }
    }
    return result;
  }

  /**
   * Tries to insert one meander run (a series of 45-degree triangle bumps) into the longest
   * suitable straight segment of the net's traces. Returns the number of bumps inserted
   * (0 when no candidate validates).
   */
  private static int insert_meander(RoutingBoard p_board, String p_net_name, double p_deficit,
      java.util.Set<Integer> p_matcher_owned) {
    List<PolylineTrace> traces = new ArrayList<>();
    for (var net : p_board.rules.nets.get(p_net_name)) {
      for (Item item : p_board.get_items()) {
        if (item instanceof PolylineTrace trace && trace.contains_net(net.net_number)
            && (trace.get_fixed_state() == FixedState.UNFIXED
                || p_matcher_owned.contains(trace.get_id_no()))) {
          traces.add(trace);
        }
      }
    }
    traces.sort((a, b) -> Integer.compare(a.get_id_no(), b.get_id_no()));
    for (PolylineTrace trace : traces) {
      Polyline polyline = trace.polyline();
      // Segments of this trace: axis-parallel (triangle-wave meanders) and perfect 45-degree
      // diagonals (staircase conversion -- a diagonal run replaced by an x/y staircase adds
      // (2 - sqrt(2)) of its axial length while deviating at most one step laterally, which
      // is what a 45-degree-routed board's long runs offer). Deterministic ordering.
      List<long[]> segments = new ArrayList<>(); // {length_or_axial, index, type 0=axis 1=diag}
      for (int i = 0; i < polyline.corner_count() - 1; i++) {
        Point a = polyline.corner(i);
        Point b = polyline.corner(i + 1);
        IntPoint ai = a.to_float().round();
        IntPoint bi = b.to_float().round();
        long adx = Math.abs((long) (bi.x - ai.x));
        long ady = Math.abs((long) (bi.y - ai.y));
        if (adx == 0 || ady == 0) {
          segments.add(new long[]{adx + ady, i, 0});
        } else if (adx == ady) {
          segments.add(new long[]{adx, i, 1});
        }
      }
      int half_width = trace.get_half_width();
      int[] amplitudes = {12 * half_width, 8 * half_width, 5 * half_width, 3 * half_width, 2 * half_width};
      // Rank every candidate by the length it can actually add (limited by both the deficit
      // and the segment's span), best first; ties by segment index then amplitude.
      List<long[]> candidates = new ArrayList<>(); // {achievable_gain, segment_index, amplitude/step, type}
      for (long[] segment : segments) {
        if (segment[2] == 0) {
          for (int amplitude : amplitudes) {
            long usable = segment[0] - 2L * amplitude;
            long fits = usable / (3L * amplitude);
            if (fits <= 0) {
              continue;
            }
            double extra_per_bump = 2 * amplitude * (Math.sqrt(2) - 1);
            long wanted = (long) Math.floor(p_deficit / extra_per_bump);
            long gain = (long) (Math.min(fits, wanted) * extra_per_bump);
            if (gain <= 0) {
              continue;
            }
            candidates.add(new long[]{gain, segment[1], amplitude, 0});
          }
        } else {
          for (int step : new int[]{8 * half_width, 4 * half_width, 2 * half_width}) {
            long usable = segment[0] - 2L * step;
            long fits = usable / step;
            if (fits <= 0) {
              continue;
            }
            double extra_per_step = step * (2 - Math.sqrt(2));
            long wanted = (long) Math.floor(p_deficit / extra_per_step);
            long gain = (long) (Math.min(fits, wanted) * extra_per_step);
            if (gain <= 0) {
              continue;
            }
            candidates.add(new long[]{gain, segment[1], step, 1});
          }
        }
      }
      candidates.sort((x, y) -> {
        if (x[0] != y[0]) {
          return Long.compare(y[0], x[0]);
        }
        if (x[1] != y[1]) {
          return Long.compare(x[1], y[1]);
        }
        return Long.compare(y[2], x[2]);
      });
      for (long[] candidate : candidates) {
        for (int side = 0; side < 2; side++) {
          int inserted = candidate[3] == 0
              ? try_meander_on_segment(p_board, trace, (int) candidate[1],
                  (int) candidate[2], side == 0 ? 1 : -1, p_deficit)
              : try_staircase_on_segment(p_board, trace, (int) candidate[1],
                  (int) candidate[2], side == 0, p_deficit);
          if (inserted > 0) {
            if (last_inserted_id >= 0) {
              p_matcher_owned.add(last_inserted_id);
            }
            return inserted;
          }
        }
      }
    }
    return 0;
  }

  /**
   * Staircase conversion of a perfect 45-degree segment: n diagonal steps of p_step become
   * axis-aligned L-moves (x-first or y-first), each adding p_step * (2 - sqrt(2)) of length
   * while deviating at most one step from the original centreline. Validated and replaced
   * exactly like the triangle meander.
   */
  private static int try_staircase_on_segment(RoutingBoard p_board, PolylineTrace p_trace,
      int p_segment_index, int p_step, boolean p_x_first, double p_deficit) {
    Polyline polyline = p_trace.polyline();
    IntPoint a = polyline.corner(p_segment_index).to_float().round();
    IntPoint b = polyline.corner(p_segment_index + 1).to_float().round();
    int sx = Integer.signum(b.x - a.x);
    int sy = Integer.signum(b.y - a.y);
    long axial = Math.abs((long) (b.x - a.x));
    long margin = p_step;
    long usable = axial - 2 * margin;
    long fits = usable / p_step;
    if (fits <= 0) {
      return 0;
    }
    double extra_per_step = p_step * (2 - Math.sqrt(2));
    long wanted = (long) Math.floor(p_deficit / extra_per_step);
    int steps = (int) Math.min(fits, wanted);
    if (steps <= 0) {
      return 0;
    }
    List<Point> corners = new ArrayList<>();
    for (int i = 0; i <= p_segment_index; i++) {
      corners.add(polyline.corner(i));
    }
    long cx = a.x + (long) sx * margin;
    long cy = a.y + (long) sy * margin;
    corners.add(new IntPoint((int) cx, (int) cy));
    for (int k = 0; k < steps; k++) {
      if (p_x_first) {
        corners.add(new IntPoint((int) (cx + (long) sx * p_step), (int) cy));
      } else {
        corners.add(new IntPoint((int) cx, (int) (cy + (long) sy * p_step)));
      }
      cx += (long) sx * p_step;
      cy += (long) sy * p_step;
      corners.add(new IntPoint((int) cx, (int) cy));
    }
    for (int i = p_segment_index + 1; i < polyline.corner_count(); i++) {
      corners.add(polyline.corner(i));
    }
    return validate_and_replace(p_board, p_trace, polyline, corners, steps);
  }

  /** Id of the trace created by the most recent successful try_meander_on_segment. */
  private static int last_inserted_id = -1;

  /**
   * Builds the meandered polyline for one segment, validates it against the board DRC and
   * replaces the trace when legal. One triangle bump of amplitude a consumes 2a of axial
   * span and adds 2a(sqrt(2)-1) of length.
   */
  private static int try_meander_on_segment(RoutingBoard p_board, PolylineTrace p_trace,
      int p_segment_index, int p_amplitude, int p_side, double p_deficit) {
    Polyline polyline = p_trace.polyline();
    IntPoint a = polyline.corner(p_segment_index).to_float().round();
    IntPoint b = polyline.corner(p_segment_index + 1).to_float().round();
    int ux = Integer.signum(b.x - a.x);
    int uy = Integer.signum(b.y - a.y);
    long segment_length = Math.abs((long) (b.x - a.x)) + Math.abs((long) (b.y - a.y));
    // Keep a margin of one amplitude to each original corner, and one amplitude of gap
    // between bumps so consecutive diagonals cannot interact.
    long margin = p_amplitude;
    long usable = segment_length - 2 * margin;
    long per_bump_span = 3L * p_amplitude; // 2a of zigzag + a of gap
    if (usable < per_bump_span) {
      return 0;
    }
    double extra_per_bump = 2 * p_amplitude * (Math.sqrt(2) - 1);
    int bumps_wanted = (int) Math.floor(p_deficit / extra_per_bump);
    int bumps = (int) Math.min(bumps_wanted, usable / per_bump_span);
    if (bumps <= 0) {
      return 0;
    }
    // Lateral unit: perpendicular to (ux, uy), signed by p_side.
    int vx = -uy * p_side;
    int vy = ux * p_side;
    List<Point> corners = new ArrayList<>();
    for (int i = 0; i <= p_segment_index; i++) {
      corners.add(polyline.corner(i));
    }
    long cursor_x = a.x + (long) ux * margin;
    long cursor_y = a.y + (long) uy * margin;
    for (int bump = 0; bump < bumps; bump++) {
      corners.add(new IntPoint((int) cursor_x, (int) cursor_y));
      corners.add(new IntPoint((int) (cursor_x + (long) ux * p_amplitude + (long) vx * p_amplitude),
          (int) (cursor_y + (long) uy * p_amplitude + (long) vy * p_amplitude)));
      corners.add(new IntPoint((int) (cursor_x + 2L * ux * p_amplitude),
          (int) (cursor_y + 2L * uy * p_amplitude)));
      cursor_x += (long) ux * per_bump_span;
      cursor_y += (long) uy * per_bump_span;
    }
    for (int i = p_segment_index + 1; i < polyline.corner_count(); i++) {
      corners.add(polyline.corner(i));
    }
    return validate_and_replace(p_board, p_trace, polyline, corners, bumps);
  }

  /**
   * Shared tail of both meander shapes: build the polyline, run the board's own DRC, and
   * replace the trace (SHOVE_FIXED) only when legal; restore the original on insert failure.
   */
  private static int validate_and_replace(RoutingBoard p_board, PolylineTrace p_trace,
      Polyline p_original, List<Point> p_corners, int p_bumps) {
    Polyline meandered;
    try {
      meandered = new Polyline(p_corners.toArray(new Point[0]));
    } catch (Exception e) {
      return 0; // degenerate corner list
    }
    if (meandered.corner_count() < p_corners.size() - 2) {
      return 0; // Polyline collapsed corners; geometry not as planned
    }
    int[] net_no_arr = new int[p_trace.net_count()];
    for (int i = 0; i < net_no_arr.length; i++) {
      net_no_arr[i] = p_trace.get_net_no(i);
    }
    boolean legal;
    try {
      legal = p_board.check_polyline_trace(meandered, p_trace.get_layer(), p_trace.get_half_width(),
          net_no_arr, p_trace.clearance_class_no());
    } catch (Exception e) {
      legal = false;
    }
    if (!legal) {
      return 0;
    }
    int layer = p_trace.get_layer();
    int half_width = p_trace.get_half_width();
    int clearance_class = p_trace.clearance_class_no();
    p_board.remove_item(p_trace);
    PolylineTrace inserted = p_board.insert_trace_without_cleaning(meandered, layer, half_width,
        net_no_arr, clearance_class, FixedState.SHOVE_FIXED);
    if (inserted == null) {
      // Restore the original; the removal cannot be left dangling.
      p_board.insert_trace_without_cleaning(p_original, layer, half_width, net_no_arr,
          clearance_class, FixedState.UNFIXED);
      last_inserted_id = -1;
      return 0;
    }
    last_inserted_id = inserted.get_id_no();
    return p_bumps;
  }
}
