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
    List<Item> planned_items = new ArrayList<>();
    for (Pin pin : candidates) {
      if (plan_one(p_board, pin, planned_items)) {
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

  private static boolean plan_one(RoutingBoard p_board, Pin p_pin, List<Item> p_inserted) {
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

    // Candidates nearest-first.
    List<int[]> spots = new ArrayList<>();
    for (int dx = -reach; dx <= reach; dx += step) {
      for (int dy = -reach; dy <= reach; dy += step) {
        if (dx != 0 || dy != 0) {
          spots.add(new int[]{dx, dy});
        }
      }
    }
    spots.sort((a, b) -> Long.compare(
        (long) a[0] * a[0] + (long) a[1] * a[1],
        (long) b[0] * b[0] + (long) b[1] * b[1]));

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
