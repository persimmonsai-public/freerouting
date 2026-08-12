package app.freerouting.settings;

import com.google.gson.annotations.SerializedName;
import java.io.Serializable;

/**
 * One region-scoped rule override (Phase 1 of docs/dense-bga-roadmap.md): an axis-aligned
 * box plus a layer set, carrying a clearance override and optionally a trace half-width
 * override. Real fine-pitch boards define exception zones (BGA fields, module tiles) whose
 * legal clearance is smaller than the global rule; without region rules the engine is
 * permanently locked out of those zones even though DRC-legal routing exists there.
 *
 * <p><b>Units</b>: all values are in MICROMETERS in the DSN file's coordinate frame -- the
 * numbers are the same ones you read in the DSN file when the DSN unit is um (the common
 * KiCad export: {@code (resolution um 10) (unit um)}). They are converted to internal board
 * units at load time with the board's own coordinate transform, so boards with mil/inch DSN
 * units are handled too. Y follows the DSN sign convention (KiCad exports negative-down).
 *
 * <p>Only honored when {@code featureFlags.ruleRegions} is enabled; see
 * {@link app.freerouting.board.RuleRegion} for the exact semantics each engine layer
 * implements.
 */
public class RuleRegionSettings implements Serializable, Cloneable {

  /**
   * Layer set the region applies to: {@code "*"} (or null) for all layers, otherwise a
   * comma-separated list of 0-based layer indices or layer names (e.g. {@code "0,1"} or
   * {@code "F.Cu,B.Cu"}).
   */
  @SerializedName("layers")
  public String layers;

  /**
   * The region box as {@code [x1, y1, x2, y2]} in micrometers, DSN coordinate frame
   * (corner order does not matter; the box is normalized at load time).
   */
  @SerializedName("box_um")
  public double[] boxUm;

  /** Clearance override inside the region, in micrometers. Required, must be positive. */
  @SerializedName("clearance_um")
  public Double clearanceUm;

  /**
   * Optional trace half-width override inside the region, in micrometers. When set, the
   * region retry clamps every layer's trace half-width to this value (never widens).
   */
  @SerializedName("trace_halfwidth_um")
  public Double traceHalfwidthUm;

  @Override
  public RuleRegionSettings clone() {
    RuleRegionSettings result = new RuleRegionSettings();
    result.layers = this.layers;
    result.boxUm = (this.boxUm != null) ? this.boxUm.clone() : null;
    result.clearanceUm = this.clearanceUm;
    result.traceHalfwidthUm = this.traceHalfwidthUm;
    return result;
  }
}
