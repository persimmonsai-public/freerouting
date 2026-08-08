package app.freerouting.autoroute;

import app.freerouting.geometry.planar.IntBox;
import app.freerouting.geometry.planar.TileShape;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
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
   * While a bulk mutation is open, {@link #insert}/{@link #remove} only accumulate dirty
   * x-ranges; {@link #end_bulk} rebuilds each disjoint range. The ranges are kept disjoint
   * (from_x -> to_x, merged on overlap) rather than collapsed into one bounding range:
   * lifting a routed net whose pads sit at opposite board edges must cost the slabs the net
   * actually touches, not a whole-board rebuild.
   */
  private boolean bulk_open;
  private final TreeMap<Integer, Integer> bulk_ranges = new TreeMap<>();
  /**
   * Slab spans whose intervals changed since cells/rooms were last brought up to date.
   * {@link #cells()} and {@link #rooms()} apply these by LOCAL splicing instead of whole-board
   * rebuilds -- the design's "updates are local" property extended from the slab intervals to
   * the derived structures (whole-board cell+room rebuilds per attempt were measured as the
   * dominant flag-on cost: 19-26 s of a 38 s pass).
   */
  private final TreeMap<Integer, Integer> stale_cell_ranges = new TreeMap<>();
  private final TreeMap<Integer, Integer> stale_room_ranges = new TreeMap<>();

  /**
   * Merges [p_from, p_to] into a disjoint range map, absorbing every overlapping entry.
   */
  private static void merge_range(TreeMap<Integer, Integer> p_ranges, int p_from, int p_to) {
    int from = p_from;
    int to = p_to;
    Map.Entry<Integer, Integer> prev = p_ranges.floorEntry(to);
    while (prev != null && prev.getValue() >= from) {
      from = Math.min(from, prev.getKey());
      to = Math.max(to, prev.getValue());
      p_ranges.remove(prev.getKey());
      prev = p_ranges.floorEntry(to);
    }
    p_ranges.put(from, to);
  }

  /**
   * The live obstacle map (id -> stored shapes), for diff-based synchronization against an
   * external source of truth. Callers must treat it as read-only.
   */
  Map<Integer, List<TileShape>> obstacle_map() {
    return obstacles;
  }

  /**
   * Starts accumulating mutations without rebuilding; must be paired with {@link #end_bulk}.
   */
  public void begin_bulk() {
    bulk_open = true;
    bulk_ranges.clear();
  }

  /**
   * Applies the accumulated dirty ranges, one rebuild per disjoint range.
   */
  public void end_bulk() {
    bulk_open = false;
    for (Map.Entry<Integer, Integer> range : bulk_ranges.entrySet()) {
      rebuild_slab_range(clamp_x(range.getKey()), clamp_x(range.getValue()));
    }
    bulk_ranges.clear();
  }

  private void mark_or_rebuild(int p_from_x, int p_to_x) {
    if (bulk_open) {
      merge_range(bulk_ranges, p_from_x, p_to_x);
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
   * The current cells. Brought up to date on demand -- built once, then locally spliced over
   * the accumulated stale slab spans. Must not be mutated by the caller.
   */
  public List<Cell> cells() {
    if (cells == null) {
      build_cells();
      stale_cell_ranges.clear();
      rooms = null;
      room_adjacency = null;
      room_keys = null;
      stale_room_ranges.clear();
    } else if (!stale_cell_ranges.isEmpty()) {
      List<Map.Entry<Integer, Integer>> pending = new ArrayList<>(stale_cell_ranges.entrySet());
      stale_cell_ranges.clear();
      for (Map.Entry<Integer, Integer> range : pending) {
        update_cells_range(range.getKey(), range.getValue());
      }
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
    cells();
    if (adjacency == null) {
      build_cell_adjacency();
    }
    List<Cell> result = adjacency.get(p_cell);
    return result == null ? List.of() : result;
  }

  /**
   * Cell adjacency from scratch over the current cell list: cells ending at an edge border
   * cells starting at that edge where y-ranges overlap. Lazy because only diagnostics and
   * tests read it; the router searches the room cover.
   */
  private void build_cell_adjacency() {
    adjacency = new HashMap<>();
    Map<Integer, List<Cell>> ended_at = new HashMap<>();
    Map<Integer, List<Cell>> started_at = new HashMap<>();
    for (Cell c : cells) {
      ended_at.computeIfAbsent(c.box.ur.x, k -> new ArrayList<>()).add(c);
      started_at.computeIfAbsent(c.box.ll.x, k -> new ArrayList<>()).add(c);
    }
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

  private int clamp_x(int p_x) {
    return Math.max(bounds.ll.x, Math.min(bounds.ur.x, p_x));
  }

  /**
   * Recomputes the free intervals of every slab whose left edge lies in [p_from_x, p_to_x).
   */
  private void rebuild_slab_range(int p_from_x, int p_to_x) {
    // One slab wider on both sides than [p_from_x, p_to_x): a newly added edge can SPLIT a
    // pre-existing slab, leaving the portion before p_from_x with a stale entry keyed by the
    // old left edge, and the portion at/after p_to_x with no entry at all. (Found by the
    // randomized update-equivalence property test.)
    Integer slab_left = slab_edges.lower(p_from_x);
    if (slab_left == null) {
      slab_left = slab_edges.first();
    }
    int span_to = p_to_x;
    for (Integer left = slab_left; left != null; left = slab_edges.higher(left)) {
      Integer right = slab_edges.higher(left);
      if (right == null) {
        break;
      }
      free_intervals.put(left, compute_free_intervals(left, right));
      span_to = Math.max(span_to, right);
      if (left >= p_to_x) {
        break;
      }
    }
    // Drop stale entries for x values that are no longer slab lefts (edge set may have grown,
    // splitting a slab whose old entry would otherwise linger).
    free_intervals.keySet().removeIf(x -> !slab_edges.contains(x) || x.equals(slab_edges.last()));
    if (cells != null) {
      merge_range(stale_cell_ranges, slab_left, span_to);
    }
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
   * Builds cells (maximal horizontal merges of identical slab intervals) from scratch.
   */
  private void build_cells() {
    cells = new ArrayList<>();
    adjacency = null;
    // Open strips from the previous slab: y-interval -> cell start x. Iterated in slab order.
    Map<Long, int[]> open = new HashMap<>(); // key = packed interval, value = {startX, low, high}
    Map<Long, int[]> next_open;

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
      finalize_open(open, left);
      open = next_open;
      left = right;
    }
    finalize_open(open, slab_edges.last());
  }

  /**
   * Splices the cell list over one stale slab span [p_a, p_b]: removes every cell overlapping
   * the span's hull, re-sweeps that hull from the current intervals, and merges the boundary
   * strips with the untouched cells they now continue into (absorption). The result must equal
   * a from-scratch build -- verified by the randomized update-equivalence property tests.
   */
  private void update_cells_range(int p_a, int p_b) {
    // Snap to live slab edges: endpoints recorded earlier may since have been deleted.
    Integer a_edge = slab_edges.floor(p_a);
    int a = a_edge == null ? slab_edges.first() : a_edge;
    Integer b_edge = slab_edges.ceiling(p_b);
    int b = b_edge == null ? slab_edges.last() : b_edge;
    // Hull of cells touching [a, b] (closed: a cell ending exactly at `a` may now merge on),
    // grown to a FIXPOINT: any cell overlapping the hull's open interior will be removed and
    // re-swept, so its full span must lie inside the hull -- otherwise its tail beyond the
    // hull is silently lost (caught by the update-equivalence property test).
    int x0 = a;
    int x1 = b;
    boolean grew = true;
    while (grew) {
      grew = false;
      for (Cell c : cells) {
        boolean touches_range = c.box.ur.x >= a && c.box.ll.x <= b;
        boolean overlaps_hull = c.box.ur.x > x0 && c.box.ll.x < x1;
        if (touches_range || overlaps_hull) {
          if (c.box.ll.x < x0) {
            x0 = c.box.ll.x;
            grew = true;
          }
          if (c.box.ur.x > x1) {
            x1 = c.box.ur.x;
            grew = true;
          }
        }
      }
    }
    if (x0 >= x1) {
      return;
    }
    // Keep cells whose span lies outside the OPEN hull; collect absorption candidates at the
    // two hull boundaries (at most one per y-interval: two identically-intervalled cells
    // meeting at an edge cannot both exist, they would have been merged).
    List<Cell> kept = new ArrayList<>(cells.size());
    Map<Long, Cell> absorb_left = new HashMap<>();
    Map<Long, Cell> absorb_right = new HashMap<>();
    for (Cell c : cells) {
      if (c.box.ur.x > x0 && c.box.ll.x < x1) {
        continue; // inside the hull: superseded by the re-sweep
      }
      kept.add(c);
      if (c.box.ur.x == x0) {
        absorb_left.put(pack(c.box.ll.y, c.box.ur.y), c);
      }
      if (c.box.ll.x == x1) {
        absorb_right.put(pack(c.box.ll.y, c.box.ur.y), c);
      }
    }
    java.util.Set<Cell> absorbed = new java.util.HashSet<>();
    List<Cell> fresh = new ArrayList<>();
    Map<Long, int[]> open = new HashMap<>();
    int room_from = x0;
    int room_to = x1;
    Integer left = x0;
    while (left != null && left < x1) {
      Integer right = slab_edges.higher(left);
      if (right == null) {
        break;
      }
      List<int[]> intervals = free_intervals.getOrDefault(left, List.of());
      Map<Long, int[]> next_open = new HashMap<>();
      for (int[] iv : intervals) {
        long key = pack(iv[0], iv[1]);
        int[] existing = open.remove(key);
        if (existing != null) {
          next_open.put(key, existing);
        } else {
          int start = left;
          if (left == x0) {
            Cell continued = absorb_left.get(key);
            if (continued != null && absorbed.add(continued)) {
              start = continued.box.ll.x;
              room_from = Math.min(room_from, start);
            }
          }
          next_open.put(key, new int[]{start, iv[0], iv[1]});
        }
      }
      for (int[] strip : open.values()) {
        fresh.add(new Cell(new IntBox(strip[0], strip[1], left, strip[2])));
      }
      open = next_open;
      left = right;
    }
    for (int[] strip : open.values()) {
      long key = pack(strip[1], strip[2]);
      int end = x1;
      Cell continued = absorb_right.get(key);
      if (continued != null && absorbed.add(continued)) {
        end = continued.box.ur.x;
        room_to = Math.max(room_to, end);
      }
      fresh.add(new Cell(new IntBox(strip[0], strip[1], end, strip[2])));
    }
    List<Cell> spliced = new ArrayList<>(kept.size() + fresh.size());
    for (Cell c : kept) {
      if (!absorbed.contains(c)) {
        spliced.add(c);
      }
    }
    spliced.addAll(fresh);
    for (int i = 0; i < spliced.size(); i++) {
      spliced.get(i).index = i;
    }
    cells = spliced;
    adjacency = null;
    merge_range(stale_room_ranges, room_from, room_to);
  }

  /**
   * A horizontally-maximal free rectangle: a merged cell extended left and right through every
   * slab whose free interval CONTAINS the cell's y-interval (the cell merge requires equality).
   * Rooms overlap each other and form a COVER of the free space, not a partition.
   *
   * <p>Why they exist: the disjoint slab cells are about one trace width wide in dense
   * regions, and {@code LocateFoundConnectionAlgo} erodes each room by the compensated trace
   * half-width before placing interior corners -- on a thin cell the eroded interior is empty
   * and corner placement degenerates (measured: realized polylines left the cell chain
   * entirely). Maximal rectangles give the erosion real interior to work with.
   */
  public static final class Room {
    public final IntBox box;
    /** Position in {@link #rooms()}, stable until the next mutation. */
    public int index;

    Room(IntBox p_box) {
      this.box = p_box;
    }
  }

  private List<Room> rooms;
  private Map<Room, List<Room>> room_adjacency;
  private java.util.Set<RoomKey> room_keys;

  /**
   * The current room cover. Brought up to date on demand -- built once, then locally updated
   * over the accumulated stale spans (invalidate touched rooms, re-extend the cells that can
   * regenerate them, dedup against the kept rooms). Must not be mutated by the caller.
   */
  public List<Room> rooms() {
    cells();
    if (rooms == null) {
      build_rooms();
      stale_room_ranges.clear();
    } else if (!stale_room_ranges.isEmpty()) {
      List<Map.Entry<Integer, Integer>> pending = new ArrayList<>(stale_room_ranges.entrySet());
      stale_room_ranges.clear();
      update_rooms(pending);
    }
    return rooms;
  }

  /**
   * The rooms whose rectangles intersect p_region (touching edges do not count).
   */
  public List<Room> rooms_intersecting(IntBox p_region) {
    List<Room> result = new ArrayList<>();
    for (Room r : rooms()) {
      if (r.box.ll.x < p_region.ur.x && p_region.ll.x < r.box.ur.x
          && r.box.ll.y < p_region.ur.y && p_region.ll.y < r.box.ur.y) {
        result.add(r);
      }
    }
    return result;
  }

  /**
   * The rooms whose rectangles intersect p_room's with positive extent in at least one
   * dimension (2-dimensional overlap, or a shared border segment; corner-point touches are
   * excluded).
   */
  public List<Room> room_neighbors(Room p_room) {
    if (room_adjacency == null) {
      build_rooms();
    }
    List<Room> result = room_adjacency.get(p_room);
    return result == null ? List.of() : result;
  }

  private record RoomKey(int left, int right, int lo, int hi) {
  }

  /**
   * The slab structure flattened into arrays: extension walks over TreeSet navigation with
   * linear interval scans measured at ~66 ms per build, ~26 s per routing pass. Per slab i
   * (left edge edges[i]): free interval bounds sorted by low; intervals are disjoint, so the
   * only candidate to contain [lo, hi] is the last one with low <= lo.
   */
  private record FlatSlabs(int[] edges, int edge_count, int[][] lows, int[][] highs) {
  }

  private FlatSlabs flatten_slabs() {
    int[] edges = new int[slab_edges.size()];
    int edge_count = 0;
    for (int edge : slab_edges) {
      edges[edge_count++] = edge;
    }
    int[][] slab_lows = new int[edge_count][];
    int[][] slab_highs = new int[edge_count][];
    for (int i = 0; i + 1 < edge_count; i++) {
      List<int[]> intervals = free_intervals.get(edges[i]);
      if (intervals == null) {
        continue;
      }
      int[] lows = new int[intervals.size()];
      int[] highs = new int[intervals.size()];
      for (int k = 0; k < intervals.size(); k++) {
        lows[k] = intervals.get(k)[0];
        highs[k] = intervals.get(k)[1];
      }
      slab_lows[i] = lows;
      slab_highs[i] = highs;
    }
    return new FlatSlabs(edges, edge_count, slab_lows, slab_highs);
  }

  /**
   * The horizontally-maximal extension of one cell, or null when the cell's edges are not live
   * slab edges (cannot happen for a fresh cell list; defensive).
   */
  private static RoomKey extend_cell(Cell p_cell, FlatSlabs p_slabs) {
    int lo = p_cell.box.ll.y;
    int hi = p_cell.box.ur.y;
    int left_index = java.util.Arrays.binarySearch(p_slabs.edges, 0, p_slabs.edge_count, p_cell.box.ll.x);
    int right_index = java.util.Arrays.binarySearch(p_slabs.edges, 0, p_slabs.edge_count, p_cell.box.ur.x);
    if (left_index < 0 || right_index < 0) {
      return null;
    }
    while (left_index > 0 && slab_contains(p_slabs.lows[left_index - 1], p_slabs.highs[left_index - 1], lo, hi)) {
      --left_index;
    }
    while (right_index + 1 < p_slabs.edge_count
        && slab_contains(p_slabs.lows[right_index], p_slabs.highs[right_index], lo, hi)) {
      ++right_index;
    }
    return new RoomKey(p_slabs.edges[left_index], p_slabs.edges[right_index], lo, hi);
  }

  private void build_rooms() {
    FlatSlabs slabs = flatten_slabs();
    rooms = new ArrayList<>();
    room_adjacency = new HashMap<>();
    room_keys = new java.util.HashSet<>();
    for (Cell c : cells) {
      RoomKey key = extend_cell(c, slabs);
      if (key != null && room_keys.add(key)) {
        Room room = new Room(new IntBox(key.left(), key.lo(), key.right(), key.hi()));
        room.index = rooms.size();
        rooms.add(room);
      }
    }
    for (int i = 0; i < rooms.size(); i++) {
      for (int j = i + 1; j < rooms.size(); j++) {
        if (rooms_share_region(rooms.get(i).box, rooms.get(j).box)) {
          room_adjacency.computeIfAbsent(rooms.get(i), k -> new ArrayList<>()).add(rooms.get(j));
          room_adjacency.computeIfAbsent(rooms.get(j), k -> new ArrayList<>()).add(rooms.get(i));
        }
      }
    }
  }

  private static boolean rooms_share_region(IntBox p_a, IntBox p_b) {
    int dx = Math.min(p_a.ur.x, p_b.ur.x) - Math.max(p_a.ll.x, p_b.ll.x);
    int dy = Math.min(p_a.ur.y, p_b.ur.y) - Math.max(p_a.ll.y, p_b.ll.y);
    return dx >= 0 && dy >= 0 && (dx > 0 || dy > 0);
  }

  /**
   * Locally updates the room cover over the accumulated stale spans, applied in ONE batch (a
   * per-span version re-flattened the slabs and re-scanned per span, which was measured
   * slower than the wholesale rebuild it replaced). Every room touching a span is invalid
   * (its extension read intervals that changed, or it should now extend further). The seeds
   * able to regenerate every replacement are exactly the cells TOUCHING the spans: extension
   * is maximal, so any single cell of a fragment regenerates the whole fragment, and a
   * fragment of a split room always owns a cell ending at or inside the changed span.
   * Deduplication against the kept rooms' keys makes regeneration idempotent. Must equal a
   * from-scratch build -- property-tested.
   */
  private void update_rooms(List<Map.Entry<Integer, Integer>> p_ranges) {
    List<Room> invalid = new ArrayList<>();
    List<Room> survivors = new ArrayList<>(rooms.size());
    for (Room r : rooms) {
      if (touches_any(r.box.ll.x, r.box.ur.x, p_ranges)) {
        invalid.add(r);
      } else {
        survivors.add(r);
      }
    }
    java.util.Set<Room> invalid_set = new java.util.HashSet<>(invalid);
    for (Room r : invalid) {
      room_keys.remove(new RoomKey(r.box.ll.x, r.box.ur.x, r.box.ll.y, r.box.ur.y));
      List<Room> neighbor_list = room_adjacency.remove(r);
      if (neighbor_list != null) {
        for (Room neighbor : neighbor_list) {
          if (!invalid_set.contains(neighbor)) {
            List<Room> back = room_adjacency.get(neighbor);
            if (back != null) {
              back.remove(r);
            }
          }
        }
      }
    }
    rooms = survivors;
    FlatSlabs slabs = flatten_slabs();
    // Seeds: cells touching a span (new/changed cells, fragment boundary cells) PLUS the
    // DEFINING cells of every removed room -- cells whose exact y-interval equals the room's,
    // inside its span. A still-valid removed room, or a fragment pinched far from the span,
    // is regenerated only by such a cell; span-touching cells at wider slabs extend to WIDER
    // rooms, not to it (a gap the property tests catch without this).
    java.util.Set<Cell> seeds = new java.util.LinkedHashSet<>();
    Map<Long, List<Cell>> cells_by_interval = new HashMap<>();
    for (Cell c : cells) {
      if (touches_any(c.box.ll.x, c.box.ur.x, p_ranges)) {
        seeds.add(c);
      }
      cells_by_interval.computeIfAbsent(pack(c.box.ll.y, c.box.ur.y), k -> new ArrayList<>()).add(c);
    }
    for (Room r : invalid) {
      List<Cell> defining = cells_by_interval.get(pack(r.box.ll.y, r.box.ur.y));
      if (defining == null) {
        continue;
      }
      for (Cell c : defining) {
        if (c.box.ll.x >= r.box.ll.x && c.box.ur.x <= r.box.ur.x) {
          seeds.add(c);
        }
      }
    }
    List<Room> added = new ArrayList<>();
    for (Cell c : seeds) {
      RoomKey key = extend_cell(c, slabs);
      if (key != null && room_keys.add(key)) {
        Room room = new Room(new IntBox(key.left(), key.lo(), key.right(), key.hi()));
        added.add(room);
      }
    }
    // Link fresh rooms: against survivors first, then fresh-fresh pairs -- indexed loops so
    // no membership scans are needed.
    for (Room fresh : added) {
      for (Room survivor : rooms) {
        if (rooms_share_region(fresh.box, survivor.box)) {
          room_adjacency.computeIfAbsent(fresh, k -> new ArrayList<>()).add(survivor);
          room_adjacency.computeIfAbsent(survivor, k -> new ArrayList<>()).add(fresh);
        }
      }
    }
    for (int i = 0; i < added.size(); i++) {
      for (int j = i + 1; j < added.size(); j++) {
        if (rooms_share_region(added.get(i).box, added.get(j).box)) {
          room_adjacency.computeIfAbsent(added.get(i), k -> new ArrayList<>()).add(added.get(j));
          room_adjacency.computeIfAbsent(added.get(j), k -> new ArrayList<>()).add(added.get(i));
        }
      }
    }
    rooms.addAll(added);
    for (int i = 0; i < rooms.size(); i++) {
      rooms.get(i).index = i;
    }
  }

  /**
   * Whether [p_from, p_to] touches (closed) any of the disjoint ascending ranges.
   */
  private static boolean touches_any(int p_from, int p_to, List<Map.Entry<Integer, Integer>> p_ranges) {
    for (Map.Entry<Integer, Integer> range : p_ranges) {
      if (range.getKey() > p_to) {
        return false;
      }
      if (range.getValue() >= p_from) {
        return true;
      }
    }
    return false;
  }

  /**
   * Whether one of the disjoint sorted free intervals (p_lows/p_highs) contains [p_lo, p_hi].
   */
  private static boolean slab_contains(int[] p_lows, int[] p_highs, int p_lo, int p_hi) {
    if (p_lows == null || p_lows.length == 0) {
      return false;
    }
    int idx = java.util.Arrays.binarySearch(p_lows, p_lo);
    if (idx < 0) {
      idx = -idx - 2; // last interval with low <= p_lo
    }
    return idx >= 0 && p_highs[idx] >= p_hi;
  }

  private void finalize_open(Map<Long, int[]> p_open, int p_end_x) {
    for (int[] strip : p_open.values()) {
      Cell cell = new Cell(new IntBox(strip[0], strip[1], p_end_x, strip[2]));
      cell.index = cells.size();
      cells.add(cell);
    }
    p_open.clear();
  }

  private static long pack(int p_low, int p_high) {
    return ((long) p_low << 32) ^ (p_high & 0xffffffffL);
  }
}
