package app.freerouting.autoroute;

import app.freerouting.board.ConductionArea;
import app.freerouting.board.Item;
import app.freerouting.board.Pin;
import app.freerouting.board.RoutingBoard;
import app.freerouting.board.Trace;
import app.freerouting.board.Via;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntBox;

/**
 * Escape-feasibility classifier for unescaped SMD pins -- the production graduation of the
 * probe-level spatial spot scan (docs/dense-bga-roadmap.md: "U1 investigation complete" and
 * "RETRACTION + corrected accounting"). It replaces the retired v1 interior/boundary
 * heuristic, which over-counted 48 of 57 rules-impossible verdicts on the Issue732 gate
 * board; the spatial scan against the search tree is the validated feasibility test.
 *
 * <p>Scan semantics (each term measured, none guessed):
 * <ul>
 * <li>IS-OBSTACLE semantics: conduction areas with {@code get_is_obstacle() == false}
 *     (copper pours the exporting CAD reflows) are passable -- treating them as obstacles
 *     was the exact bug behind the retracted "all 57 rules-impossible" verdict.</li>
 * <li>Only the candidate via's SPAN layers are checked, not the whole stack.</li>
 * <li>Same-net copper is passable (this is a feasibility scan, not the planner's placement
 *     check, which additionally enforces the no-attach own-pad rule before inserting).</li>
 * </ul>
 *
 * <p>Verdicts: {@link Verdict#ESCAPABLE_NOW} -- a legal spot exists on the current board;
 * {@link Verdict#ORDERING_VICTIM} -- no legal spot now, but one exists when routed copper
 * (traces and vias) is ignored, i.e. the spot was consumed by neighbours' fanout/routing --
 * the Phase-2 planning case. This consumed-spot analysis is the probe's own pre-fanout
 * approximation and is cheap on the live board, so it graduates to production with the
 * stated caveat that boards carrying FIXED pre-routed copper have that copper treated as
 * consumable too (on the measured fixtures pre-existing copper is pours and pads, so the
 * approximation matched the true pre-fanout board exactly);
 * {@link Verdict#RULES_IMPOSSIBLE} -- no legal spot even on the bare board: the design's
 * own rules forbid the escape, and search work spent on it is pure waste.
 *
 * <p>Two entry points with DIFFERENT calibrations, both sharing the scan engine:
 * <ul>
 * <li>{@link #classify(RoutingBoard, Pin)} -- the accounting/reporting form: the probe's
 *     exact calibration (globally smallest via, default clearance class,
 *     {@link #SCAN_REACH}/{@link #SCAN_STEP} grid). This is what every roadmap escape
 *     accounting was measured with, and the probe asserts production agreement per pin.</li>
 * <li>{@link #classify_for_planner(RoutingBoard, Pin)} -- the pre-filter form
 *     {@link EscapePlanner} keys skips on: the PLANNER'S OWN placement geometry (smallest
 *     via covering the pin's layer, that via's clearance class and span, reach/step derived
 *     from the pad exactly as the planner derives them) with the scan's strictly more
 *     permissive passability. A RULES_IMPOSSIBLE verdict here is therefore provably
 *     conservative: every candidate the planner would try is blocked even ignoring routed
 *     copper, so skipping the pin cannot change what the planner inserts -- only save the
 *     wasted search. (Measured necessity: skipping on the probe calibration instead changed
 *     caniot-tiny-arm from 26 inserted escapes to 11 -- fixed probe constants do not
 *     transfer across board unit scales and via sets; the planner-geometry form does by
 *     construction.)</li>
 * </ul>
 *
 * <p>Deterministic (fixed scan order, no hashing) and read-only: the board is never
 * mutated, so classification can never change routing outcomes by itself.
 */
public final class EscapeFeasibility {

  /** Feasibility verdict for one unescaped SMD pin. */
  public enum Verdict {
    /** A legal via spot exists on the current board state. */
    ESCAPABLE_NOW,
    /** Legal spots existed before routed copper consumed them (planning/ordering case). */
    ORDERING_VICTIM,
    /** No legal via spot exists even ignoring routed copper: impossible under the rules. */
    RULES_IMPOSSIBLE
  }

  /**
   * How far from the pad centre the accounting scan reaches, in board units -- the probe
   * calibration (1.5x the Issue732 U1 pitch) that every roadmap accounting was measured
   * with.
   */
  public static final int SCAN_REACH = 13500;

  /** Accounting scan grid step in board units (the probe calibration; 1/6 of the U1 pitch). */
  public static final int SCAN_STEP = 1500;

  private EscapeFeasibility() {
  }

  /** Classifies with the probe's measured accounting calibration. */
  public static Verdict classify(RoutingBoard p_board, Pin p_pin) {
    return classify(p_board, p_pin, SCAN_REACH, SCAN_STEP);
  }

  /**
   * Accounting classification: the globally SMALLEST via governs (its span, default
   * clearance class 1), candidates on the given grid around the pad centre. Read-only.
   *
   * @param p_reach half-extent of the candidate scan around the pad centre, board units
   * @param p_step scan grid step, board units
   */
  public static Verdict classify(RoutingBoard p_board, Pin p_pin, int p_reach, int p_step) {
    int via_radius = Integer.MAX_VALUE;
    int from_layer = 0;
    int to_layer = p_board.get_layer_count() - 1;
    for (int v = 0; v < p_board.rules.via_infos.count(); v++) {
      var padstack = p_board.rules.via_infos.get(v).get_padstack();
      var shape = padstack.get_shape(padstack.from_layer());
      if (shape == null) {
        continue;
      }
      IntBox via_box = shape.bounding_box();
      int radius = (via_box.ur.x - via_box.ll.x) / 2;
      if (radius < via_radius) {
        via_radius = radius;
        from_layer = padstack.from_layer();
        to_layer = padstack.to_layer();
      }
    }
    if (via_radius == Integer.MAX_VALUE) {
      // No via rule at all: nothing can ever escape by via under this design.
      return Verdict.RULES_IMPOSSIBLE;
    }
    return scan(p_board, p_pin, via_radius, from_layer, to_layer, 1, p_reach, p_step);
  }

  /**
   * Pre-filter classification in the planner's own placement geometry: the smallest via
   * whose span covers the pin's layer (the via {@link EscapePlanner} would place), that
   * via's clearance class and span layers, and the planner's own candidate grid (reach =
   * 3x the pad's larger extent, step = max(500, extent/4)). Because the scan's passability
   * is a strict superset of the planner's placement test, RULES_IMPOSSIBLE here proves the
   * planner would fail every candidate anyway -- skipping such a pin is outcome-neutral by
   * construction. Read-only.
   */
  public static Verdict classify_for_planner(RoutingBoard p_board, Pin p_pin) {
    int layer = p_pin.first_layer();
    int via_radius = Integer.MAX_VALUE;
    int from_layer = 0;
    int to_layer = 0;
    int clearance_class = 1;
    for (int v = 0; v < p_board.rules.via_infos.count(); v++) {
      var info = p_board.rules.via_infos.get(v);
      var padstack = info.get_padstack();
      if (padstack.from_layer() > layer || padstack.to_layer() < layer) {
        continue;
      }
      var shape = padstack.get_shape(padstack.from_layer());
      if (shape == null) {
        continue;
      }
      IntBox via_box = shape.bounding_box();
      int radius = (via_box.ur.x - via_box.ll.x) / 2;
      if (radius < via_radius) {
        via_radius = radius;
        from_layer = padstack.from_layer();
        to_layer = padstack.to_layer();
        clearance_class = info.get_clearance_class();
      }
    }
    if (via_radius == Integer.MAX_VALUE) {
      // No via spans this pin's layer: the planner has nothing to place.
      return Verdict.RULES_IMPOSSIBLE;
    }
    IntBox pad = p_pin.get_tile_shape_on_layer(layer).bounding_box();
    int pitch_guess = Math.max(pad.ur.x - pad.ll.x, pad.ur.y - pad.ll.y);
    int reach = 3 * pitch_guess;
    int step = Math.max(500, pitch_guess / 4);
    return scan(p_board, p_pin, via_radius, from_layer, to_layer, clearance_class, reach, step);
  }

  /**
   * The shared scan engine: grid candidates around the pad centre, each checked on the via
   * span against the default search tree. Passability: same-net copper, non-obstacle
   * conduction areas; the "pre" pass additionally ignores traces and vias (routed copper).
   */
  private static Verdict scan(RoutingBoard p_board, Pin p_pin, int p_via_radius,
      int p_from_layer, int p_to_layer, int p_clearance_class, int p_reach, int p_step) {
    var tree = p_board.search_tree_manager.get_default_tree();
    FloatPoint center = p_pin.get_center().to_float();
    boolean pre_spot_exists = false;
    for (int dx = -p_reach; dx <= p_reach; dx += p_step) {
      for (int dy = -p_reach; dy <= p_reach; dy += p_step) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        int cx = (int) center.x + dx;
        int cy = (int) center.y + dy;
        IntBox via_box = new IntBox(cx - p_via_radius, cy - p_via_radius,
            cx + p_via_radius, cy + p_via_radius);
        boolean legal_now = true;
        // "Pre" ignores routed copper (traces and vias): the consumed-spot approximation of
        // the pre-fanout board that separates ORDERING_VICTIM from RULES_IMPOSSIBLE.
        boolean legal_pre = true;
        for (int layer = p_from_layer; layer <= p_to_layer && legal_pre; layer++) {
          for (var entry : tree.overlapping_tree_entries_with_clearance(
              via_box, layer, new int[0], p_clearance_class)) {
            if (!(entry.object instanceof Item blocking) || blocking.shares_net(p_pin)) {
              continue;
            }
            if (blocking instanceof ConductionArea pour && !pour.get_is_obstacle()) {
              continue; // pours reflow around new copper; the router treats them as passable
            }
            legal_now = false;
            if (!(blocking instanceof Trace) && !(blocking instanceof Via)) {
              legal_pre = false;
              break;
            }
          }
        }
        if (legal_now) {
          return Verdict.ESCAPABLE_NOW;
        }
        if (legal_pre) {
          pre_spot_exists = true;
        }
      }
    }
    return pre_spot_exists ? Verdict.ORDERING_VICTIM : Verdict.RULES_IMPOSSIBLE;
  }
}
