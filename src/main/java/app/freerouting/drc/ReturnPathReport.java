package app.freerouting.drc;

import app.freerouting.board.BasicBoard;
import app.freerouting.board.ConductionArea;
import app.freerouting.board.Item;
import app.freerouting.board.Via;
import java.util.ArrayList;
import java.util.List;
import java.util.TreeSet;

/**
 * Return-path discontinuity report (docs/dense-bga-roadmap.md, Phase 5): every signal via is
 * a layer-change point where the return current must also change reference planes, so a
 * healthy design has a same-net (series/stitching) via or a reference-plane via nearby.
 * This report lists the signal vias whose nearest such via is farther than a threshold.
 *
 * <p>Reference vias are the vias of nets owning {@link ConductionArea}s (planes/pours).
 * Distances are between via centres in board units. Purely analytical: no board mutation,
 * no participation in routing or scoring. Findings are ordered worst-first, deterministic
 * (ties resolve by via id).
 */
public final class ReturnPathReport {

  /**
   * One offending layer-change point.
   *
   * @param via_id id of the signal via
   * @param net_no the via's (first) net number
   * @param net_name that net's name
   * @param x via centre x in board units
   * @param y via centre y in board units
   * @param first_layer first layer of the via span
   * @param last_layer last layer of the via span
   * @param nearest_return_distance centre distance to the nearest same-net or
   *     reference-plane via; {@link Double#POSITIVE_INFINITY} when none exists at all
   */
  public record Finding(int via_id, int net_no, String net_name, double x, double y,
      int first_layer, int last_layer, double nearest_return_distance) {
  }

  /**
   * The full analysis result.
   *
   * @param signal_via_count vias on non-plane nets (the layer-change points examined)
   * @param reference_via_count vias on plane-owning nets
   * @param plane_net_count distinct nets owning conduction areas
   * @param threshold the distance threshold the findings exceed
   * @param findings offenders only, worst (largest distance) first
   */
  public record Result(int signal_via_count, int reference_via_count, int plane_net_count,
      int threshold, List<Finding> findings) {
  }

  private ReturnPathReport() {
  }

  /**
   * Analyzes the board's routed state. Read-only.
   */
  public static Result analyze(BasicBoard p_board, int p_threshold) {
    TreeSet<Integer> plane_nets = new TreeSet<>();
    for (Item item : p_board.get_items()) {
      if (item instanceof ConductionArea pour) {
        for (int n = 0; n < pour.net_count(); n++) {
          plane_nets.add(pour.get_net_no(n));
        }
      }
    }
    List<Via> signal_vias = new ArrayList<>();
    List<Via> reference_vias = new ArrayList<>();
    for (Item item : p_board.get_items()) {
      if (item instanceof Via via && via.net_count() > 0) {
        if (plane_nets.contains(via.get_net_no(0))) {
          reference_vias.add(via);
        } else {
          signal_vias.add(via);
        }
      }
    }
    List<Finding> findings = new ArrayList<>();
    for (Via via : signal_vias) {
      var center = via.get_center().to_float();
      double nearest = Double.POSITIVE_INFINITY;
      for (Via other : signal_vias) {
        if (other != via && other.get_net_no(0) == via.get_net_no(0)) {
          var oc = other.get_center().to_float();
          nearest = Math.min(nearest, Math.hypot(oc.x - center.x, oc.y - center.y));
        }
      }
      for (Via other : reference_vias) {
        var oc = other.get_center().to_float();
        nearest = Math.min(nearest, Math.hypot(oc.x - center.x, oc.y - center.y));
      }
      if (nearest > p_threshold) {
        int net_no = via.get_net_no(0);
        var net = p_board.rules.nets.get(net_no);
        findings.add(new Finding(via.get_id_no(), net_no, net == null ? ("net#" + net_no) : net.name,
            center.x, center.y, via.first_layer(), via.last_layer(), nearest));
      }
    }
    findings.sort((a, b) -> {
      int by_distance = Double.compare(b.nearest_return_distance(), a.nearest_return_distance());
      return by_distance != 0 ? by_distance : Integer.compare(a.via_id(), b.via_id());
    });
    return new Result(signal_vias.size(), reference_vias.size(), plane_nets.size(), p_threshold,
        findings);
  }
}
