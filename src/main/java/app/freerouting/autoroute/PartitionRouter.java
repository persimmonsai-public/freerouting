package app.freerouting.autoroute;

import app.freerouting.board.Item;
import app.freerouting.board.Pin;
import app.freerouting.board.RoutingBoard;
import app.freerouting.board.ShapeSearchTree;
import app.freerouting.geometry.planar.IntBox;
import app.freerouting.geometry.planar.TileShape;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

/**
 * Connection search over the maintained {@link FreeSpacePartition} instead of the lazily built
 * expansion-room decomposition. Stage 2 of the free-space-partition rework
 * (docs/free-space-partition.md); used only behind a flag, with fallback to the classic engine.
 *
 * <p>Scope of this first stage, chosen so the increment is small and gateable:
 *
 * <ul>
 *   <li><b>Single-layer routes only.</b> A route that needs a layer change requires
 *       {@code ExpansionDrill} objects in the backtrack chain handed to the existing
 *       realization pipeline; connections needing vias fall back to the classic engine.
 *   <li><b>No ripup.</b> The partition is net-agnostic (every item is a wall), so a blocked
 *       route simply fails here and falls back to the classic engine, which rips.
 *   <li><b>Staleness is safe, not prevented.</b> The partition may lag the board (pull-tight
 *       moves traces after every commit); the found path is re-validated against the live
 *       search tree before insertion, and a conflict triggers refresh-and-fallback.
 * </ul>
 */
public final class PartitionRouter {

  /**
   * A found single-layer route: the cell sequence to realize, plus the item shapes it attaches
   * to at both ends.
   */
  public static final class CellRoute {
    public final int layer;
    public final List<FreeSpacePartition.Cell> cells;
    public final Item start_item;
    public final int start_tree_entry_no;
    public final Item target_item;
    public final int target_tree_entry_no;

    CellRoute(int p_layer, List<FreeSpacePartition.Cell> p_cells, Item p_start_item,
        int p_start_entry, Item p_target_item, int p_target_entry) {
      this.layer = p_layer;
      this.cells = p_cells;
      this.start_item = p_start_item;
      this.start_tree_entry_no = p_start_entry;
      this.target_item = p_target_item;
      this.target_tree_entry_no = p_target_entry;
    }
  }

  private final RoutingBoard board;
  private final ShapeSearchTree tree;
  /**
   * One partition per layer index; null for non-signal layers.
   */
  private final FreeSpacePartition[] partitions;
  private boolean dirty = true;

  public PartitionRouter(RoutingBoard p_board, ShapeSearchTree p_tree) {
    this.board = p_board;
    this.tree = p_tree;
    this.partitions = new FreeSpacePartition[p_board.get_layer_count()];
  }

  /**
   * Marks the partitions stale; they are rebuilt wholesale on the next search. A full rebuild
   * was measured at 21 ms for the reference board, so per-commit incremental maintenance is
   * deliberately deferred until the wholesale cost shows up in a profile.
   */
  public void invalidate() {
    this.dirty = true;
  }

  private void ensure_fresh() {
    if (!dirty) {
      return;
    }
    IntBox bounds = board.get_bounding_box();
    for (int layer = 0; layer < partitions.length; layer++) {
      if (!board.layer_structure.arr[layer].is_signal) {
        partitions[layer] = null;
        continue;
      }
      FreeSpacePartition partition = new FreeSpacePartition(bounds);
      for (Item item : board.get_items()) {
        int shape_count = item.tree_shape_count(tree);
        List<TileShape> shapes = null;
        for (int i = 0; i < shape_count; i++) {
          if (item.shape_layer(i) != layer) {
            continue;
          }
          TileShape shape = item.get_tree_shape(tree, i);
          if (shape == null || shape.is_empty()) {
            continue;
          }
          if (shapes == null) {
            shapes = new ArrayList<>(2);
          }
          shapes.add(shape);
        }
        if (shapes != null) {
          partition.insert(item.get_id_no(), shapes);
        }
      }
      partitions[layer] = partition;
    }
    dirty = false;
  }

  /**
   * Attempts a single-layer route between the two item sets. Returns null when no such route
   * exists in the current partition state (including every case this stage does not support);
   * the caller falls back to the classic engine.
   *
   * @param p_half_width per-layer compensated trace half width; a cell border must admit the
   *     full trace width to be traversable
   */
  public CellRoute try_route(Set<Item> p_start_set, Set<Item> p_dest_set, int[] p_half_width) {
    ensure_fresh();
    // The routing net's own items must not be walls: Locate seeds the destination from the
    // item's CONNECTION shape -- the pad center point for drill items, the centerline for
    // traces -- so the destination room must overlap the item's interior, and the classic
    // engine gets that by carving rooms net-dependently. Here the net's items are lifted out
    // of the shared partition for the duration of the search and re-inserted afterwards;
    // each lift/re-insert is a cheap local slab rebuild.
    Set<Item> net_items = new java.util.HashSet<>();
    net_items.addAll(p_start_set);
    net_items.addAll(p_dest_set);
    if (net_items.size() > 48) {
      // Lifting a huge net (power/ground class) out of the partition costs hundreds of slab
      // rebuilds per attempt; those connections go to the classic engine.
      return null;
    }
    for (FreeSpacePartition partition : partitions) {
      if (partition == null) {
        continue;
      }
      partition.begin_bulk();
      for (Item item : net_items) {
        partition.remove(item.get_id_no());
      }
      partition.end_bulk();
    }
    try {
      CellRoute best = null;
      double best_cost = Double.MAX_VALUE;
      for (int layer = 0; layer < partitions.length; layer++) {
        if (partitions[layer] == null) {
          continue;
        }
        CellRoute route = try_route_on_layer(layer, p_start_set, p_dest_set,
            Math.max(1, p_half_width[layer]));
        if (route != null) {
          double cost = route_length(route);
          if (cost < best_cost) {
            best_cost = cost;
            best = route;
          }
        }
      }
      return best;
    } finally {
      for (int layer = 0; layer < partitions.length; layer++) {
        FreeSpacePartition partition = partitions[layer];
        if (partition == null) {
          continue;
        }
        partition.begin_bulk();
        for (Item item : net_items) {
          List<TileShape> shapes = null;
          int shape_count = item.tree_shape_count(tree);
          for (int i = 0; i < shape_count; i++) {
            if (item.shape_layer(i) != layer) {
              continue;
            }
            TileShape shape = item.get_tree_shape(tree, i);
            if (shape != null && !shape.is_empty()) {
              if (shapes == null) {
                shapes = new ArrayList<>(2);
              }
              shapes.add(shape);
            }
          }
          if (shapes != null) {
            partition.insert(item.get_id_no(), shapes);
          }
        }
        partition.end_bulk();
      }
    }
  }

  private static double route_length(CellRoute p_route) {
    double result = 0;
    for (int i = 0; i + 1 < p_route.cells.size(); i++) {
      result += center_distance(p_route.cells.get(i).box, p_route.cells.get(i + 1).box);
    }
    return result;
  }

  private CellRoute try_route_on_layer(int p_layer, Set<Item> p_start_set, Set<Item> p_dest_set,
      int p_half_width) {
    FreeSpacePartition partition = partitions[p_layer];
    // Cells abutting an item shape, with the shape's tree entry number for door construction.
    Map<FreeSpacePartition.Cell, ItemContact> starts = contact_cells(partition, p_start_set, p_layer, p_half_width, false);
    if (starts.isEmpty()) {
      return null;
    }
    Map<FreeSpacePartition.Cell, ItemContact> targets = contact_cells(partition, p_dest_set, p_layer, p_half_width, true);
    if (targets.isEmpty()) {
      return null;
    }

    // A* over cells: g = accumulated center distance, h = distance to the nearest target box.
    List<FreeSpacePartition.Cell> cells = partition.cells();
    double[] g = new double[cells.size()];
    int[] came_from = new int[cells.size()];
    java.util.Arrays.fill(g, Double.MAX_VALUE);
    java.util.Arrays.fill(came_from, -1);
    PriorityQueue<double[]> open = new PriorityQueue<>((a, b) -> Double.compare(a[0], b[0]));
    for (FreeSpacePartition.Cell start : starts.keySet()) {
      g[start.index] = 0;
      open.add(new double[]{heuristic(start, targets.keySet()), start.index});
    }
    FreeSpacePartition.Cell reached = null;
    boolean[] closed = new boolean[cells.size()];
    while (!open.isEmpty()) {
      int current = (int) open.poll()[1];
      if (closed[current]) {
        continue;
      }
      closed[current] = true;
      FreeSpacePartition.Cell cell = cells.get(current);
      if (targets.containsKey(cell)) {
        reached = cell;
        break;
      }
      for (FreeSpacePartition.Cell neighbor : partition.neighbors(cell)) {
        if (closed[neighbor.index]) {
          continue;
        }
        int min_pass = 2 * (p_half_width + AutorouteEngine.TRACE_WIDTH_TOLERANCE) + 2;
        if (border_length(cell.box, neighbor.box) < min_pass) {
          continue; // the border cannot host a non-degenerate door section at this width
        }
        // Doors are vertical, so door length measures the trace's Y clearance -- but a cell
        // NARROWER than the trace (thin gap between fine-pitch pads) additionally cannot host
        // any vertical movement: a near-vertical segment is ~2*half_width wide in X and clips
        // the obstacles bounding the cell on both sides (measured: the dominant reject cause,
        // route-through-pin-row conflicts). Such a cell is traversable only STRAIGHT through:
        // the entry door, the cell, and the exit door must share a Y-interval of trace width.
        if (cell.box.ur.x - cell.box.ll.x < min_pass && came_from[current] >= 0) {
          IntBox entry_box = cells.get(came_from[current]).box;
          int shared_lo = Math.max(Math.max(entry_box.ll.y, neighbor.box.ll.y), cell.box.ll.y);
          int shared_hi = Math.min(Math.min(entry_box.ur.y, neighbor.box.ur.y), cell.box.ur.y);
          if (shared_hi - shared_lo < min_pass) {
            continue;
          }
        }
        double candidate = g[current] + center_distance(cell.box, neighbor.box);
        if (candidate < g[neighbor.index]) {
          g[neighbor.index] = candidate;
          came_from[neighbor.index] = current;
          open.add(new double[]{candidate + heuristic(neighbor, targets.keySet()), neighbor.index});
        }
      }
    }
    if (reached == null) {
      return null;
    }
    List<FreeSpacePartition.Cell> path = new ArrayList<>();
    for (int at = reached.index; at != -1; at = came_from[at]) {
      path.add(0, cells.get(at));
    }
    ItemContact start_contact = starts.get(path.get(0));
    ItemContact target_contact = targets.get(reached);
    return new CellRoute(p_layer, path, start_contact.item, start_contact.tree_entry_no,
        target_contact.item, target_contact.tree_entry_no);
  }

  private record ItemContact(Item item, int tree_entry_no) {
  }

  /**
   * Cells that touch (within p_touch_margin) a shape of any of the items on the layer, mapped
   * to that item and shape index.
   */
  private Map<FreeSpacePartition.Cell, ItemContact> contact_cells(FreeSpacePartition p_partition,
      Set<Item> p_items, int p_layer, int p_touch_margin, boolean p_require_connection_shape) {
    Map<FreeSpacePartition.Cell, ItemContact> result = new HashMap<>();
    for (Item item : p_items) {
      int shape_count = item.tree_shape_count(tree);
      for (int i = 0; i < shape_count; i++) {
        if (item.shape_layer(i) != p_layer) {
          continue;
        }
        TileShape shape = item.get_tree_shape(tree, i);
        if (shape == null || shape.is_empty()) {
          continue;
        }
        IntBox probe = shape.bounding_box().offset(p_touch_margin);
        for (FreeSpacePartition.Cell cell : p_partition.cells_intersecting(probe)) {
          // The bounding-box probe over-selects around diagonal shapes; the door built later
          // needs the ACTUAL shape to intersect the cell (abutting closed regions share their
          // border segment, which is enough).
          if (shape.intersection(cell.box).is_empty()) {
            continue;
          }
          if (p_require_connection_shape) {
            // Locate's destination side seeds from the item's CONNECTION shape (for traces it
            // exists only near the endpoints), so a contact that only touches the tree shape
            // is unusable as a target.
            TileShape connection = ((app.freerouting.board.Connectable) item)
                .get_trace_connection_shape(tree, i);
            if (connection == null || connection.intersection(cell.box).is_empty()) {
              continue;
            }
          }
          ItemContact previous = result.get(cell);
          if (previous == null || (previous.item() instanceof Pin && !(item instanceof Pin))) {
            result.put(cell, new ItemContact(item, i));
          }
        }
      }
    }
    return result;
  }

  /**
   * Builds the backtrack chain the existing realization pipeline expects from a found cell
   * route, so that {@code LocateFoundConnectionAlgo} and {@code InsertFoundConnectionAlgo} --
   * which carry all the corner-placement and target-attachment subtleties -- run unchanged.
   *
   * <p>The chain is a pure object graph: rooms are built from cells, doors from shared cell
   * borders, and neither Locate nor Insert queries the search tree for any of them. Two
   * contract details are load-bearing:
   *
   * <ul>
   *   <li>Door sections must be allocated by calling {@code get_section_segments} with exactly
   *       the offset Locate will later use (the compensated half width); a count mismatch makes
   *       Locate silently reallocate the sections and wipe the backtrack pointers set here.
   *   <li>The walk runs destination-to-start following {@code backtrack_door} references and
   *       derives each room by reference identity via {@code door.other_room(...)}; it
   *       terminates at the first element whose {@code backtrack_door} is null, which must be
   *       the start {@code TargetItemExpansionDoor}.
   * </ul>
   *
   * @return the seed for {@code LocateFoundConnectionAlgo.get_instance}, or null when a door
   *     degenerates (the caller falls back to the classic engine)
   */
  public MazeSearchAlgo.Result materialize(CellRoute p_route, AutorouteControl p_ctrl) {
    int layer = p_route.layer;
    int half_width = p_ctrl.compensated_trace_half_width[layer];
    List<FreeSpacePartition.Cell> path = p_route.cells;

    List<CompleteFreeSpaceExpansionRoom> rooms = new ArrayList<>(path.size());
    for (int i = 0; i < path.size(); i++) {
      rooms.add(new CompleteFreeSpaceExpansionRoom(path.get(i).box, layer, i + 1));
    }

    TargetItemExpansionDoor start_door =
        new TargetItemExpansionDoor(p_route.start_item, p_route.start_tree_entry_no, rooms.get(0), tree);
    TargetItemExpansionDoor target_door = new TargetItemExpansionDoor(p_route.target_item,
        p_route.target_tree_entry_no, rooms.get(rooms.size() - 1), tree);
    if (start_door.get_shape().is_empty() || target_door.get_shape().is_empty()) {
      return null;
    }
    // Locate's destination-side starting point needs this intersection to be non-empty too.
    TileShape target_connection = ((app.freerouting.board.Connectable) p_route.target_item)
        .get_trace_connection_shape(tree, p_route.target_tree_entry_no);
    if (target_connection == null
        || target_connection.intersection(rooms.get(rooms.size() - 1).get_shape()).is_empty()) {
      return null;
    }

    // Aim the crossing point of every door at the straight line between the two attachment
    // shapes. The arbitrary middle section forced pad exits at bad angles, clipping the
    // clearance of NEIGHBOURING pads -- the dominant validation failure before this.
    app.freerouting.geometry.planar.FloatPoint aim_from = start_door.get_shape().centre_of_gravity();
    app.freerouting.geometry.planar.FloatPoint aim_to = target_door.get_shape().centre_of_gravity();
    ExpansionDoor[] doors = new ExpansionDoor[path.size() - 1];
    int[] door_section = new int[doors.length];
    for (int i = 0; i < doors.length; i++) {
      ExpansionDoor door = new ExpansionDoor(rooms.get(i), rooms.get(i + 1));
      var sections = door.get_section_segments(half_width);
      if (sections == null || sections.length == 0) {
        return null;
      }
      doors[i] = door;
      double t = (i + 1) / (double) (doors.length + 1);
      double aim_x = aim_from.x + t * (aim_to.x - aim_from.x);
      double aim_y = aim_from.y + t * (aim_to.y - aim_from.y);
      int best_section = 0;
      double best_distance = Double.MAX_VALUE;
      for (int sec = 0; sec < sections.length; sec++) {
        double mid_x = (sections[sec].a.x + sections[sec].b.x) / 2;
        double mid_y = (sections[sec].a.y + sections[sec].b.y) / 2;
        double dx = mid_x - aim_x;
        double dy = mid_y - aim_y;
        double distance = dx * dx + dy * dy;
        if (distance < best_distance) {
          best_distance = distance;
          best_section = sec;
        }
      }
      door_section[i] = best_section;
    }

    // Wire the backtrack pointers, destination first.
    MazeSearchElement target_element = target_door.get_maze_search_element(0);
    if (doors.length == 0) {
      target_element.backtrack_door = start_door;
      target_element.section_no_of_backtrack_door = 0;
    } else {
      target_element.backtrack_door = doors[doors.length - 1];
      target_element.section_no_of_backtrack_door = door_section[doors.length - 1];
      for (int i = doors.length - 1; i >= 1; i--) {
        MazeSearchElement element = doors[i].get_maze_search_element(door_section[i]);
        element.backtrack_door = doors[i - 1];
        element.section_no_of_backtrack_door = door_section[i - 1];
      }
      MazeSearchElement first = doors[0].get_maze_search_element(door_section[0]);
      first.backtrack_door = start_door;
      first.section_no_of_backtrack_door = 0;
    }
    // The start door's element keeps its default null backtrack_door, terminating the walk.
    return new MazeSearchAlgo.Result(target_door, 0);
  }

  private static double heuristic(FreeSpacePartition.Cell p_cell, Set<FreeSpacePartition.Cell> p_targets) {
    double best = Double.MAX_VALUE;
    for (FreeSpacePartition.Cell target : p_targets) {
      best = Math.min(best, box_distance(p_cell.box, target.box));
    }
    return best;
  }

  /**
   * Length of the shared vertical border between two horizontally adjacent boxes.
   */
  private static int border_length(IntBox p_a, IntBox p_b) {
    return Math.min(p_a.ur.y, p_b.ur.y) - Math.max(p_a.ll.y, p_b.ll.y);
  }

  private static double center_distance(IntBox p_a, IntBox p_b) {
    double dx = (p_a.ll.x + p_a.ur.x) / 2.0 - (p_b.ll.x + p_b.ur.x) / 2.0;
    double dy = (p_a.ll.y + p_a.ur.y) / 2.0 - (p_b.ll.y + p_b.ur.y) / 2.0;
    return Math.sqrt(dx * dx + dy * dy);
  }

  /**
   * Minimum distance between two boxes (0 when they touch or overlap) -- an admissible A*
   * heuristic for center-distance edge costs up to the half-extent slack, which only affects
   * path optimality, never reachability.
   */
  private static double box_distance(IntBox p_a, IntBox p_b) {
    double dx = Math.max(0, Math.max(p_b.ll.x - p_a.ur.x, p_a.ll.x - p_b.ur.x));
    double dy = Math.max(0, Math.max(p_b.ll.y - p_a.ur.y, p_a.ll.y - p_b.ur.y));
    return Math.sqrt(dx * dx + dy * dy);
  }
}
