package app.freerouting.autoroute;

import app.freerouting.geometry.planar.IntBox;
import app.freerouting.geometry.planar.TileShape;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;

/**
 * A maintained partition of one layer's free space into axis-aligned rectangular cells.
 *
 * <p>This is the data structure behind the planned replacement of the router's lazy,
 * search-context-dependent expansion-room construction. Today roughly half of routing time is
 * spent inside the maze search building maximal free-space rooms with tree queries
 * ({@code complete_shape} and friends) and tearing them down again after every connection --
 * measured at 184,190 room constructions for 578 connections on the reference fixture, up to
 * 1,911 rooms for a single hard connection. Retaining those rooms naively was measured at 2.6x
 * SLOWER, because they are carved relative to a specific search (ignore shapes, from-doors) and
 * fragment monotonically.
 *
 * <p>The partition takes the opposite approach: a canonical, search-independent decomposition,
 * built once and maintained locally as items are inserted and removed. Feasibility was measured
 * before this class was written: on the reference board in its fully routed (densest) state the
 * partition has <b>2,148 cells across both signal layers</b>, a full rebuild costs <b>21 ms</b>,
 * and re-partitioning a swath spanning 10% of the board costs <b>~0.8 ms</b> -- against the
 * 184k per-search room constructions it is designed to replace.
 *
 * <h2>Construction</h2>
 *
 * <p>Vertical-slab decomposition with maximal horizontal merging. Slab edges are the distinct
 * x-extents of obstacle shapes; within each slab, the free space is the complement of the
 * obstacles' y-intervals; slab-adjacent free intervals with identical bounds merge into one
 * cell. The merge criterion (identical y-interval) means <b>every cell is a rectangle</b>,
 * measured to coarsen the raw slab cells by ~23x.
 *
 * <p>Obstacle y-intervals are computed per slab from the <em>clipped</em> shape
 * ({@code shape.intersection(slabBox)}), not from the shape's global bounding box -- a
 * 45-degree trace's global box would block free space along its whole diagonal. The residual
 * over-blocking is limited to sub-slab triangles at diagonal edges.
 *
 * <h2>Maintenance</h2>
 *
 * <p>{@link #insert} and {@link #remove} rebuild only the slab range the changed shapes span,
 * then repair the horizontal merge across the two boundary edges. There is no incremental
 * cleverness inside the range -- the measured cost of wholesale range recomputation is already
 * far below every alternative, and wholesale recomputation cannot drift from the
 * built-from-scratch result, which the unit tests verify on randomized histories.
 */
public final class FreeSpacePartition {

  /**
   * One rectangular free-space cell, with adjacency to the cells it shares a vertical border
   * with. Neighbor lists are rebuilt lazily after mutations; see {@link #neighbors}.
   */
  public static final class Cell {

    public final IntBox box;
    /**
     * Position of this cell in {@link #cells()}, stable until the next mutation. Search code
     * uses it to index parallel arrays; it must not be persisted across partition updates.
     */
    public int index;

    Cell(IntBox p_box) {
      this.box = p_box;
    }
  }

  private final IntBox bounds;
  /**
   * Obstacles by caller-assigned id. A single id may carry several shapes (an item's tree
   * shapes on this layer are inserted under the item's id).
   */
  private final Map<Integer, List<TileShape>> obstacles = new HashMap<>();
  /**
   * Distinct slab-edge x coordinates, always including both bounds edges.
   */
  private final TreeSet<Integer> slab_edges = new TreeSet<>();
  /**
   * How many live obstacle shapes contribute each slab edge. Without this, edges from removed
   * obstacles accumulated forever, and repeated remove/insert cycles (the per-search net lift)
   * made every subsequent rebuild progressively slower.
   */
  private final Map<Integer, Integer> edge_refcount = new HashMap<>();
  /**
   * Free y-intervals per slab, keyed by the slab's left edge. Each interval is {low, high}.
   * Rebuilt (for affected slabs only) on every mutation.
   */
  private final Map<Integer, List<int[]>> free_intervals = new HashMap<>();

  private List<Cell> cells;
  private Map<Cell, List<Cell>> adjacency;
  /**
   * While a bulk mutation is open, {@link #insert}/{@link #remove} only accumulate the dirty
   * x-range; {@link #end_bulk} performs one slab rebuild over the union. Lifting a routed
   * net's items one at a time triggered a near-full rebuild per long trace, twice per search.
   */
  private boolean bulk_open;
  private int bulk_from_x;
  private int bulk_to_x;

  /**
   * Starts accumulating mutations without rebuilding; must be paired with {@link #end_bulk}.
   */
  public void begin_bulk() {
    bulk_open = true;
    bulk_from_x = bounds.ur.x;
    bulk_to_x = bounds.ll.x;
  }

  /**
   * Applies the accumulated dirty range in a single rebuild.
   */
  public void end_bulk() {
    bulk_open = false;
    if (bulk_from_x < bulk_to_x) {
      rebuild_slab_range(clamp_x(bulk_from_x), clamp_x(bulk_to_x));
    }
  }

  private void mark_or_rebuild(int p_from_x, int p_to_x) {
    if (bulk_open) {
      bulk_from_x = Math.min(bulk_from_x, p_from_x);
      bulk_to_x = Math.max(bulk_to_x, p_to_x);
    } else {
      rebuild_slab_range(clamp_x(p_from_x), clamp_x(p_to_x));
    }
  }

  public FreeSpacePartition(IntBox p_bounds) {
    this.bounds = p_bounds;
    this.slab_edges.add(p_bounds.ll.x);
    this.slab_edges.add(p_bounds.ur.x);
    rebuild_slab_range(p_bounds.ll.x, p_bounds.ur.x);
  }

  /**
   * Inserts the shapes of one obstacle. Replaces any previous shapes under the same id.
   */
  public void insert(int p_obstacle_id, Collection<TileShape> p_shapes) {
    int from_x = bounds.ur.x;
    int to_x = bounds.ll.x;
    List<TileShape> previous = obstacles.remove(p_obstacle_id);
    if (previous != null) {
      for (TileShape shape : previous) {
        IntBox b = shape.bounding_box();
        from_x = Math.min(from_x, b.ll.x);
        to_x = Math.max(to_x, b.ur.x);
        release_edge(clamp_x(b.ll.x));
        release_edge(clamp_x(b.ur.x));
      }
    }
    List<TileShape> kept = new ArrayList<>();
    for (TileShape shape : p_shapes) {
      if (shape == null || shape.is_empty()) {
        continue;
      }
      IntBox b = shape.bounding_box();
      if (b.ur.x <= bounds.ll.x || b.ll.x >= bounds.ur.x
          || b.ur.y <= bounds.ll.y || b.ll.y >= bounds.ur.y) {
        continue;
      }
      kept.add(shape);
      add_edge(clamp_x(b.ll.x));
      add_edge(clamp_x(b.ur.x));
      from_x = Math.min(from_x, b.ll.x);
      to_x = Math.max(to_x, b.ur.x);
    }
    if (!kept.isEmpty()) {
      obstacles.put(p_obstacle_id, kept);
    }
    if (from_x < to_x) {
      mark_or_rebuild(from_x, to_x);
    }
  }

  /**
   * Removes all shapes previously inserted under p_obstacle_id.
   */
  public void remove(int p_obstacle_id) {
    List<TileShape> previous = obstacles.remove(p_obstacle_id);
    if (previous == null) {
      return;
    }
    int from_x = bounds.ur.x;
    int to_x = bounds.ll.x;
    for (TileShape shape : previous) {
      IntBox b = shape.bounding_box();
      from_x = Math.min(from_x, b.ll.x);
      to_x = Math.max(to_x, b.ur.x);
      release_edge(clamp_x(b.ll.x));
      release_edge(clamp_x(b.ur.x));
    }
    if (from_x < to_x) {
      mark_or_rebuild(from_x, to_x);
    }
  }

  private void add_edge(int p_x) {
    edge_refcount.merge(p_x, 1, Integer::sum);
    slab_edges.add(p_x);
  }

  private void release_edge(int p_x) {
    Integer count = edge_refcount.get(p_x);
    if (count == null) {
      return;
    }
    if (count <= 1) {
      edge_refcount.remove(p_x);
      if (p_x != bounds.ll.x && p_x != bounds.ur.x) {
        slab_edges.remove(p_x);
        free_intervals.remove(p_x);
      }
    } else {
      edge_refcount.put(p_x, count - 1);
    }
  }

  /**
   * The current cells. The returned list is rebuilt on demand after mutations and must not be
   * mutated by the caller.
   */
  public List<Cell> cells() {
    if (cells == null) {
      build_cells();
    }
    return cells;
  }

  /**
   * The cells whose rectangles intersect p_region (touching edges do not count).
   */
  public List<Cell> cells_intersecting(IntBox p_region) {
    List<Cell> result = new ArrayList<>();
    for (Cell c : cells()) {
      if (c.box.ll.x < p_region.ur.x && p_region.ll.x < c.box.ur.x
          && c.box.ll.y < p_region.ur.y && p_region.ll.y < c.box.ur.y) {
        result.add(c);
      }
    }
    return result;
  }

  /**
   * The cells sharing a vertical border segment with p_cell.
   */
  public List<Cell> neighbors(Cell p_cell) {
    if (adjacency == null) {
      build_cells();
    }
    List<Cell> result = adjacency.get(p_cell);
    return result == null ? List.of() : result;
  }

  private int clamp_x(int p_x) {
    return Math.max(bounds.ll.x, Math.min(bounds.ur.x, p_x));
  }

  /**
   * Recomputes the free intervals of every slab whose left edge lies in [p_from_x, p_to_x).
   */
  private void rebuild_slab_range(int p_from_x, int p_to_x) {
    cells = null;
    adjacency = null;
    // One slab wider on both sides than [p_from_x, p_to_x): a newly added edge can SPLIT a
    // pre-existing slab, leaving the portion before p_from_x with a stale entry keyed by the
    // old left edge, and the portion at/after p_to_x with no entry at all. (Found by the
    // randomized update-equivalence property test.)
    Integer slab_left = slab_edges.lower(p_from_x);
    if (slab_left == null) {
      slab_left = slab_edges.first();
    }
    for (Integer left = slab_left; left != null; left = slab_edges.higher(left)) {
      Integer right = slab_edges.higher(left);
      if (right == null) {
        break;
      }
      free_intervals.put(left, compute_free_intervals(left, right));
      if (left >= p_to_x) {
        break;
      }
    }
    // Drop stale entries for x values that are no longer slab lefts (edge set may have grown,
    // splitting a slab whose old entry would otherwise linger).
    free_intervals.keySet().removeIf(x -> !slab_edges.contains(x) || x.equals(slab_edges.last()));
  }

  /**
   * Free y-intervals of the slab [p_left, p_right): the complement, within bounds, of every
   * obstacle shape clipped to the slab.
   */
  private List<int[]> compute_free_intervals(int p_left, int p_right) {
    IntBox slab_box = new IntBox(p_left, bounds.ll.y, p_right, bounds.ur.y);
    List<int[]> blocked = new ArrayList<>();
    for (List<TileShape> shapes : obstacles.values()) {
      for (TileShape shape : shapes) {
        IntBox b = shape.bounding_box();
        if (b.ur.x <= p_left || b.ll.x >= p_right) {
          continue;
        }
        IntBox clipped;
        if (b.ll.x >= p_left && b.ur.x <= p_right) {
          // Shape lies entirely inside the slab; its own box is exact enough.
          clipped = b;
        } else {
          TileShape piece = shape.intersection(slab_box);
          if (piece == null || piece.is_empty()) {
            continue;
          }
          clipped = piece.bounding_box();
        }
        int low = Math.max(clipped.ll.y, bounds.ll.y);
        int high = Math.min(clipped.ur.y, bounds.ur.y);
        if (high > low) {
          blocked.add(new int[]{low, high});
        }
      }
    }
    blocked.sort((a, b) -> Integer.compare(a[0], b[0]));
    List<int[]> free = new ArrayList<>();
    int cursor = bounds.ll.y;
    for (int[] iv : blocked) {
      if (iv[0] > cursor) {
        free.add(new int[]{cursor, iv[0]});
      }
      cursor = Math.max(cursor, iv[1]);
    }
    if (cursor < bounds.ur.y) {
      free.add(new int[]{cursor, bounds.ur.y});
    }
    return free;
  }

  /**
   * Builds cells (maximal horizontal merges of identical slab intervals) and their adjacency.
   */
  private void build_cells() {
    cells = new ArrayList<>();
    adjacency = new HashMap<>();
    // Open strips from the previous slab: y-interval -> cell start x. Iterated in slab order.
    Map<Long, int[]> open = new HashMap<>(); // key = packed interval, value = {startX, low, high}
    Map<Long, int[]> next_open;
    // Cells finalized at each slab edge, for adjacency: right edge x -> cells ending there.
    Map<Integer, List<Cell>> ended_at = new HashMap<>();
    Map<Integer, List<Cell>> started_at = new HashMap<>();

    Integer left = slab_edges.first();
    while (left != null && !left.equals(slab_edges.last())) {
      Integer right = slab_edges.higher(left);
      if (right == null) {
        break;
      }
      List<int[]> intervals = free_intervals.getOrDefault(left, List.of());
      next_open = new HashMap<>();
      for (int[] iv : intervals) {
        long key = pack(iv[0], iv[1]);
        int[] existing = open.remove(key);
        if (existing != null) {
          next_open.put(key, existing); // strip continues through this slab
        } else {
          next_open.put(key, new int[]{left, iv[0], iv[1]}); // strip starts here
        }
      }
      // Whatever remains in `open` did not continue: finalize those cells at x = left.
      finalize_open(open, left, ended_at, started_at);
      open = next_open;
      left = right;
    }
    finalize_open(open, slab_edges.last(), ended_at, started_at);

    // Adjacency: cells ending at edge x border cells starting at edge x where y-ranges overlap.
    for (Map.Entry<Integer, List<Cell>> e : ended_at.entrySet()) {
      List<Cell> starters = started_at.get(e.getKey());
      if (starters == null) {
        continue;
      }
      for (Cell a : e.getValue()) {
        for (Cell b : starters) {
          if (a.box.ll.y < b.box.ur.y && b.box.ll.y < a.box.ur.y) {
            adjacency.computeIfAbsent(a, k -> new ArrayList<>()).add(b);
            adjacency.computeIfAbsent(b, k -> new ArrayList<>()).add(a);
          }
        }
      }
    }
  }

  private void finalize_open(Map<Long, int[]> p_open, int p_end_x,
      Map<Integer, List<Cell>> p_ended_at, Map<Integer, List<Cell>> p_started_at) {
    for (int[] strip : p_open.values()) {
      Cell cell = new Cell(new IntBox(strip[0], strip[1], p_end_x, strip[2]));
      cell.index = cells.size();
      cells.add(cell);
      p_ended_at.computeIfAbsent(p_end_x, k -> new ArrayList<>()).add(cell);
      p_started_at.computeIfAbsent(strip[0], k -> new ArrayList<>()).add(cell);
    }
    p_open.clear();
  }

  private static long pack(int p_low, int p_high) {
    return ((long) p_low << 32) ^ (p_high & 0xffffffffL);
  }
}
