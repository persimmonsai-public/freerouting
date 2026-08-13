package app.freerouting.board;

import app.freerouting.geometry.planar.IntBox;
import app.freerouting.geometry.planar.TileShape;
import app.freerouting.logger.FRLogger;
import app.freerouting.rules.ClearanceMatrix;
import app.freerouting.settings.RouterSettings;
import app.freerouting.settings.RuleRegionSettings;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * A resolved region-scoped rule override (docs/dense-bga-roadmap.md Phase 1 item 1): an
 * axis-aligned box in board coordinates plus a layer set, carrying a clearance override and
 * optionally a trace half-width override. Resolved from {@link RuleRegionSettings}
 * (micrometers, DSN coordinate frame) against the board's own coordinate transform at
 * install time, so all values here are in internal board units.
 *
 * <p><b>v1 semantics</b> (each documented where it is implemented):
 * <ul>
 *   <li>DRC pair rule: an item pair may come as close as min(global, region) clearance when
 *       AT LEAST ONE shape of the pair lies fully inside the region box on the checked layer
 *       ({@link BasicBoard#rule_region_pair_clearance}) -- the roadmap's checked-shape rule,
 *       made symmetric in the pair so the insertability check and the scored violation count
 *       agree. A pair with neither shape inside keeps the global rule.</li>
 *   <li>Engine reach: the batch autorouter retries failed connections whose terminals touch
 *       a region at the region's rules (clearance via {@link #clearance_class_no}, clamped
 *       half-width), then enforces the global rule on every new trace shape outside the
 *       region and rolls the board back if any fails.</li>
 *   <li>Vias keep their global clearance class in v1 (regions neck traces, not vias).</li>
 * </ul>
 */
public class RuleRegion implements Serializable {

  /** Region box in board coordinates. */
  public final IntBox box;
  /** Per-layer applicability, indexed by board layer number. */
  public final boolean[] layers;
  /** Clearance override in board units (always positive). */
  public final int clearance;
  /** Trace half-width override in board units, or -1 when the region only overrides clearance. */
  public final int trace_half_width;
  /**
   * Number of the clearance class appended to the board's clearance matrix for this region:
   * its value against every other class (and itself) is {@link #clearance}, so traces
   * inserted by the region retry are legal at the region clearance and scored accordingly.
   *
   * <p><b>SHARED between all regions with the same {@link #clearance}</b>, by clearance value
   * (see {@link #install}). A clearance class is not free: the router materializes one
   * FULL-BOARD compensated {@code ShapeSearchTree} per distinct clearance class it routes at,
   * retained for the life of the board, plus a precalculated {@code TileShape[]} per item per
   * tree -- so a class-per-region scheme multiplies the whole board by the region count. The
   * class is fully determined by its clearance value (its matrix row is that value against
   * every other class), so regions that carry the same clearance are indistinguishable to the
   * engine and sharing changes no routing or DRC semantics.
   */
  public final int clearance_class_no;
  /** Name of the region, for diagnostics. */
  public final String name;

  private RuleRegion(IntBox p_box, boolean[] p_layers, int p_clearance, int p_trace_half_width,
      int p_clearance_class_no, String p_name) {
    this.box = p_box;
    this.layers = p_layers;
    this.clearance = p_clearance;
    this.trace_half_width = p_trace_half_width;
    this.clearance_class_no = p_clearance_class_no;
    this.name = p_name;
  }

  /** True if the region applies on p_layer. */
  public boolean covers_layer(int p_layer) {
    return p_layer >= 0 && p_layer < layers.length && layers[p_layer];
  }

  /** True if the region applies on p_layer and p_shape lies fully inside the region box. */
  public boolean contains(TileShape p_shape, int p_layer) {
    return covers_layer(p_layer) && p_shape.is_contained_in(this.box);
  }

  /**
   * Resolves the configured {@code router.rule_regions} against p_board's coordinate system
   * and installs them as {@code p_board.rule_regions}, appending one clearance class per
   * DISTINCT clearance VALUE to the board's clearance matrix. Idempotent: does nothing when
   * regions are already installed or none are configured. Must only be called when
   * {@code featureFlags.ruleRegions} is enabled -- flag-off boards never carry regions.
   *
   * <p>The class is keyed by the resolved clearance in board units, so the common real
   * configuration -- N exception zones all at one design-rule clearance -- costs exactly ONE
   * appended class no matter how large N is. See {@link #clearance_class_no} for why that
   * matters: distinct clearance classes, not clearance-matrix entries, are what the router's
   * memory scales on.
   */
  public static void install(BasicBoard p_board, RouterSettings p_settings) {
    if (p_board.rule_regions != null || p_settings == null) {
      return;
    }
    RuleRegionSettings[] configured = p_settings.getRuleRegions();
    if (configured.length == 0) {
      return;
    }
    List<RuleRegion> result = new ArrayList<>(configured.length);
    // Resolved clearance (board units) -> the clearance class shared by every region with
    // that clearance. Keeps the class count at the number of DISTINCT clearance values.
    java.util.Map<Integer, Integer> class_by_clearance = new java.util.LinkedHashMap<>();
    int layer_count = p_board.get_layer_count();
    for (int k = 0; k < configured.length; k++) {
      RuleRegionSettings curr = configured[k];
      if (curr == null || curr.boxUm == null || curr.boxUm.length != 4
          || curr.clearanceUm == null || curr.clearanceUm <= 0) {
        FRLogger.warn("[rule-region] region " + (k + 1)
            + " ignored: box_um must have 4 values and clearance_um must be positive");
        continue;
      }
      int x1 = to_board_coor(p_board, curr.boxUm[0]);
      int y1 = to_board_coor(p_board, curr.boxUm[1]);
      int x2 = to_board_coor(p_board, curr.boxUm[2]);
      int y2 = to_board_coor(p_board, curr.boxUm[3]);
      IntBox box = new IntBox(Math.min(x1, x2), Math.min(y1, y2), Math.max(x1, x2), Math.max(y1, y2));
      int clearance = to_board_coor(p_board, curr.clearanceUm);
      int half_width = (curr.traceHalfwidthUm != null && curr.traceHalfwidthUm > 0)
          ? to_board_coor(p_board, curr.traceHalfwidthUm)
          : -1;
      boolean[] layers = parse_layers(p_board, curr.layers, layer_count);
      String name = "fr_region_" + (k + 1);
      ClearanceMatrix matrix = p_board.rules.clearance_matrix;
      Integer shared_class = class_by_clearance.get(clearance);
      int class_no;
      if (shared_class != null) {
        class_no = shared_class;
      } else {
        String class_name = "fr_region_clearance_" + clearance;
        matrix.append_class(class_name);
        class_no = matrix.get_no(class_name);
        if (class_no < 0) {
          FRLogger.warn("[rule-region] region " + (k + 1) + " ignored: could not append clearance class");
          continue;
        }
        // The region class requires exactly the region clearance against everything,
        // including itself. Class 0 stays 0 (no clearance requirement by convention).
        for (int j = 1; j < matrix.get_class_count(); j++) {
          matrix.set_value(class_no, j, clearance);
          matrix.set_value(j, class_no, clearance);
        }
        class_by_clearance.put(clearance, class_no);
      }
      // A region whose box misses the board entirely is almost always a UNIT mistake, and it
      // fails silently otherwise: the region installs, logs, costs a clearance class, and
      // never triggers, which looks exactly like "the feature did nothing". box_um is the raw
      // number as written in the DSN file (see RuleRegionSettings), NOT that number divided
      // by the file's resolution -- getting that wrong puts every box off-board by the
      // resolution factor. Measured the hard way: 14 regions built from resolution-divided
      // coordinates installed cleanly and never fired once.
      IntBox board_box = p_board.get_bounding_box();
      if (board_box != null && board_box.intersection(box).is_empty()) {
        FRLogger.warn("[rule-region] " + name + " box=(" + box.ll.x + "," + box.ll.y + ")-("
            + box.ur.x + "," + box.ur.y + ") does NOT intersect the board bounding box ("
            + board_box.ll.x + "," + board_box.ll.y + ")-(" + board_box.ur.x + ","
            + board_box.ur.y + "): the region can never trigger. box_um must be the raw"
            + " coordinate numbers as written in the DSN file.");
      }
      result.add(new RuleRegion(box, layers, clearance, half_width, class_no, name));
      FRLogger.info("[rule-region] installed " + name
          + " box=(" + box.ll.x + "," + box.ll.y + ")-(" + box.ur.x + "," + box.ur.y + ")"
          + " clearance=" + clearance + " trace_half_width=" + half_width
          + " clearance_class=" + class_no + (shared_class != null ? " (shared)" : " (new)"));
    }
    if (!result.isEmpty()) {
      p_board.rule_regions = result;
      FRLogger.info("[rule-region] installed " + result.size() + " region(s) using "
          + class_by_clearance.size() + " clearance class(es)"
          + " (one per distinct clearance value; each distinct class costs a full-board"
          + " compensated search tree at route time)");
    }
  }

  /**
   * Converts a micrometer value in the DSN coordinate frame to internal board units, using
   * the same transform the DSN parser used for the board geometry (scale only; the
   * transform's base offset is 0 for DSN imports).
   */
  private static int to_board_coor(BasicBoard p_board, double p_um_value) {
    double dsn_value = Unit.scale(p_um_value, Unit.UM, p_board.communication.unit);
    return (int) Math.round(p_board.communication.coordinate_transform.dsn_to_board(dsn_value));
  }

  private static boolean[] parse_layers(BasicBoard p_board, String p_spec, int p_layer_count) {
    boolean[] result = new boolean[p_layer_count];
    if (p_spec == null || p_spec.isBlank() || "*".equals(p_spec.trim())) {
      java.util.Arrays.fill(result, true);
      return result;
    }
    for (String token : p_spec.split(",")) {
      String trimmed = token.trim();
      if (trimmed.isEmpty()) {
        continue;
      }
      int layer_no;
      try {
        layer_no = Integer.parseInt(trimmed);
      } catch (NumberFormatException e) {
        layer_no = p_board.layer_structure.get_no(trimmed);
      }
      if (layer_no >= 0 && layer_no < p_layer_count) {
        result[layer_no] = true;
      } else {
        FRLogger.warn("[rule-region] unknown layer '" + trimmed + "' in rule region layer set");
      }
    }
    return result;
  }
}
