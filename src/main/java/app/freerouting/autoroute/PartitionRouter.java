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
   * A found single-layer route: the room sequence to realize (overlapping maximal rectangles
   * from the partition's room cover), plus the item shapes it attaches to at both ends.
   */
  public static final class CellRoute {
    public final int layer;
    public final List<FreeSpacePartition.Room> rooms;
    /** Layer of each room in {@link #rooms}; consecutive entries differing means a via. */
    public final int[] layers;
    public final Item start_item;
    public final int start_tree_entry_no;
    public final Item target_item;
    public final int target_tree_entry_no;

    CellRoute(int p_layer, List<FreeSpacePartition.Room> p_rooms, int[] p_layers, Item p_start_item,
        int p_start_entry, Item p_target_item, int p_target_entry) {
      this.layer = p_layer;
      this.rooms = p_rooms;
      this.layers = p_layers;
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
      // Bulk mode: without it every single insert pays an immediate local slab rebuild and a
      // from-scratch build costs ~240 ms instead of the ~21 ms measured for one bulk rebuild
      // (observed as +47 s over a pass once the partition was invalidated per attempt).
      partition.begin_bulk();
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
      partition.end_bulk();
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
  public CellRoute try_route(Set<Item> p_start_set, Set<Item> p_dest_set, int[] p_half_width,
      int p_via_radius) {
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
      return try_route_multilayer(p_start_set, p_dest_set, p_half_width, p_via_radius);
    } finally {
      // Every attempt is followed by invalidate() from the caller (staleness-vs-quality
      // tradeoff measured in BatchAutorouter), so re-inserting the lifted net into
      // partitions that are about to be rebuilt wholesale is pure waste; just mark dirty.
      dirty = true;
    }
  }

  private static double route_length(CellRoute p_route) {
    double result = 0;
    for (int i = 0; i + 1 < p_route.rooms.size(); i++) {
      result += center_distance(p_route.rooms.get(i).box, p_route.rooms.get(i + 1).box);
    }
    return result;
  }

  /** Extra cost per layer change, discouraging vias a single-layer route can avoid. */
  private static final double VIA_COST = 200000;

  /**
   * A* over (layer, room) states across all signal layers. Same-layer transitions use the
   * room-cover admissions; cross-layer transitions are allowed where rooms on the two layers
   * overlap in a region that can host the via (overlap min-dimension >= via diameter plus
   * tolerance). Via locations are fixed later, at the joint window in materialize.
   */
  private CellRoute try_route_multilayer(Set<Item> p_start_set, Set<Item> p_dest_set,
      int[] p_half_width, int p_via_radius) {
    int layer_count = partitions.length;
    List<List<FreeSpacePartition.Room>> rooms_of = new ArrayList<>(layer_count);
    int[] offset = new int[layer_count + 1];
    List<Map<FreeSpacePartition.Room, ItemContact>> starts_of = new ArrayList<>(layer_count);
    List<Map<FreeSpacePartition.Room, ItemContact>> targets_of = new ArrayList<>(layer_count);
    int[] min_pass_of = new int[layer_count];
    boolean any_start = false;
    boolean any_target = false;
    for (int layer = 0; layer < layer_count; layer++) {
      FreeSpacePartition partition = partitions[layer];
      if (partition == null) {
        rooms_of.add(List.of());
        starts_of.add(Map.of());
        targets_of.add(Map.of());
        offset[layer + 1] = offset[layer];
        continue;
      }
      int half_width = Math.max(1, p_half_width[layer]);
      min_pass_of[layer] = 2 * (half_width + AutorouteEngine.TRACE_WIDTH_TOLERANCE) + 2;
      List<FreeSpacePartition.Room> rooms = partition.rooms();
      rooms_of.add(rooms);
      offset[layer + 1] = offset[layer] + rooms.size();
      Map<FreeSpacePartition.Room, ItemContact> starts = contact_rooms(partition, p_start_set, layer, half_width, true);
      Map<FreeSpacePartition.Room, ItemContact> targets = contact_rooms(partition, p_dest_set, layer, half_width, true);
      starts_of.add(starts);
      targets_of.add(targets);
      any_start |= !starts.isEmpty();
      any_target |= !targets.isEmpty();
    }
    if (!any_start || !any_target) {
      return null;
    }
    int state_count = offset[layer_count];
    double[] g = new double[state_count];
    int[] came_from = new int[state_count];
    boolean[] closed = new boolean[state_count];
    java.util.Arrays.fill(g, Double.MAX_VALUE);
    java.util.Arrays.fill(came_from, -1);
    List<IntBox> target_boxes = new ArrayList<>();
    for (int layer = 0; layer < layer_count; layer++) {
      for (FreeSpacePartition.Room room : targets_of.get(layer).keySet()) {
        target_boxes.add(room.box);
      }
    }
    PriorityQueue<double[]> open = new PriorityQueue<>((a, b) -> Double.compare(a[0], b[0]));
    for (int layer = 0; layer < layer_count; layer++) {
      int min_pass = min_pass_of[layer];
      for (FreeSpacePartition.Room start : starts_of.get(layer).keySet()) {
        if (start.box.ur.x - start.box.ll.x < min_pass || start.box.ur.y - start.box.ll.y < min_pass) {
          continue; // thin terminal room: Locate's erosion degenerates (measured)
        }
        int id = offset[layer] + start.index;
        g[id] = 0;
        open.add(new double[]{box_heuristic(start.box, target_boxes), id});
      }
    }
    int via_pass = 2 * (p_via_radius + AutorouteEngine.TRACE_WIDTH_TOLERANCE) + 2;
    int reached = -1;
    while (!open.isEmpty()) {
      int current = (int) open.poll()[1];
      if (closed[current]) {
        continue;
      }
      closed[current] = true;
      int layer = layer_of(current, offset, layer_count);
      FreeSpacePartition.Room room = rooms_of.get(layer).get(current - offset[layer]);
      if (targets_of.get(layer).containsKey(room)) {
        reached = current;
        break;
      }
      int min_pass = min_pass_of[layer];
      for (FreeSpacePartition.Room neighbor : partitions[layer].room_neighbors(room)) {
        int neighbor_id = offset[layer] + neighbor.index;
        if (closed[neighbor_id]) {
          continue;
        }
        if (neighbor.box.ur.x - neighbor.box.ll.x < min_pass
            || neighbor.box.ur.y - neighbor.box.ll.y < min_pass) {
          continue;
        }
        int dx = Math.min(room.box.ur.x, neighbor.box.ur.x) - Math.max(room.box.ll.x, neighbor.box.ll.x);
        int dy = Math.min(room.box.ur.y, neighbor.box.ur.y) - Math.max(room.box.ll.y, neighbor.box.ll.y);
        if (Math.max(dx, dy) < min_pass) {
          continue;
        }
        double candidate = g[current] + center_distance(room.box, neighbor.box);
        if (candidate < g[neighbor_id]) {
          g[neighbor_id] = candidate;
          came_from[neighbor_id] = current;
          open.add(new double[]{candidate + box_heuristic(neighbor.box, target_boxes), neighbor_id});
        }
      }
      // Cross-layer transitions: rooms on other layers overlapping enough to host the via.
      for (int other = 0; other < layer_count; other++) {
        if (other == layer || partitions[other] == null) {
          continue;
        }
        int other_min_pass = min_pass_of[other];
        for (FreeSpacePartition.Room via_room : partitions[other].rooms_intersecting(room.box)) {
          int via_id = offset[other] + via_room.index;
          if (closed[via_id]) {
            continue;
          }
          if (via_room.box.ur.x - via_room.box.ll.x < other_min_pass
              || via_room.box.ur.y - via_room.box.ll.y < other_min_pass) {
            continue;
          }
          int dx = Math.min(room.box.ur.x, via_room.box.ur.x) - Math.max(room.box.ll.x, via_room.box.ll.x);
          int dy = Math.min(room.box.ur.y, via_room.box.ur.y) - Math.max(room.box.ll.y, via_room.box.ll.y);
          if (Math.min(dx, dy) < via_pass) {
            continue; // the overlap cannot host the via barrel plus tolerance on both axes
          }
          double candidate = g[current] + center_distance(room.box, via_room.box) + VIA_COST;
          if (candidate < g[via_id]) {
            g[via_id] = candidate;
            came_from[via_id] = current;
            open.add(new double[]{candidate + box_heuristic(via_room.box, target_boxes), via_id});
          }
        }
      }
    }
    if (reached < 0) {
      return null;
    }
    List<FreeSpacePartition.Room> path = new ArrayList<>();
    List<Integer> path_layers = new ArrayList<>();
    for (int at = reached; at != -1; at = came_from[at]) {
      int layer = layer_of(at, offset, layer_count);
      path.add(0, rooms_of.get(layer).get(at - offset[layer]));
      path_layers.add(0, layer);
    }
    int[] layers = new int[path_layers.size()];
    for (int i = 0; i < layers.length; i++) {
      layers[i] = path_layers.get(i);
    }
    ItemContact start_contact = starts_of.get(layers[0]).get(path.get(0));
    ItemContact target_contact = targets_of.get(layers[layers.length - 1]).get(path.get(path.size() - 1));
    return new CellRoute(layers[0], path, layers, start_contact.item(), start_contact.tree_entry_no(),
        target_contact.item(), target_contact.tree_entry_no());
  }

  private static int layer_of(int p_state, int[] p_offset, int p_layer_count) {
    for (int layer = p_layer_count - 1; layer >= 0; layer--) {
      if (p_state >= p_offset[layer]) {
        return layer;
      }
    }
    return 0;
  }

  private static double box_heuristic(IntBox p_box, List<IntBox> p_targets) {
    double best = Double.MAX_VALUE;
    for (IntBox target : p_targets) {
      best = Math.min(best, box_distance(p_box, target));
    }
    return best;
  }

  private record ItemContact(Item item, int tree_entry_no) {
  }

  private static int contact_rank(Item p_item) {
    if (p_item instanceof app.freerouting.board.Via) {
      return 3;
    }
    if (p_item instanceof Pin) {
      return 2;
    }
    return 1;
  }

  /**
   * Rooms that touch (within p_touch_margin) a shape of any of the items on the layer, mapped
   * to that item and shape index.
   */
  private Map<FreeSpacePartition.Room, ItemContact> contact_rooms(FreeSpacePartition p_partition,
      Set<Item> p_items, int p_layer, int p_touch_margin, boolean p_require_connection_shape) {
    Map<FreeSpacePartition.Room, ItemContact> result = new HashMap<>();
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
        for (FreeSpacePartition.Room room : p_partition.rooms_intersecting(probe)) {
          // The bounding-box probe over-selects around diagonal shapes; the door built later
          // needs the ACTUAL shape to intersect the room (abutting closed regions share their
          // border segment, which is enough).
          if (shape.intersection(room.box).is_empty()) {
            continue;
          }
          if (p_require_connection_shape) {
            // Locate's destination side seeds from the item's CONNECTION shape (for traces it
            // exists only near the endpoints), so a contact that only touches the tree shape
            // is unusable as a target.
            TileShape connection = ((app.freerouting.board.Connectable) item)
                .get_trace_connection_shape(tree, i);
            if (connection == null || connection.intersection(room.box).is_empty()) {
              continue;
            }
          }
          // Attachment preference, by boundedness of the connection shape Locate will aim at:
          // vias and pins connect at a POINT (bounded), while a trace's connection shape is a
          // simplex of half-planes that is UNBOUNDED in some directions -- measured to make
          // Locate's final corner land on the 2^25 sentinel coordinate when the channel
          // approaches from an unbounded side (the dominant reject after windowed channels).
          // Pad exits are safe to prefer again because the escape-axis aim handles them.
          ItemContact previous = result.get(room);
          if (previous == null || contact_rank(item) > contact_rank(previous.item())) {
            result.put(room, new ItemContact(item, i));
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
    List<FreeSpacePartition.Room> path = p_route.rooms;

    // Rooms handed to Locate are the path rooms CLIPPED to the channel the route actually
    // needs: the bounding box of each room's entry joint and exit joint (terminal attachment
    // shapes at the ends), inflated by the pass width. Locate's 45-degree corner walk places
    // unchecked dogleg corners between the current point and the next nearest point; inside
    // one axis-aligned box any such dogleg is contained by convexity, but across the
    // 100k+-unit maximal rooms the walk drifts and the final legs ran straight down pin
    // columns (measured: the dominant reject signature). Clipping bounds the drift while
    // keeping every shape a subset of known-free space.
    int channel_margin = 2 * (half_width + AutorouteEngine.TRACE_WIDTH_TOLERANCE) + 2;
    IntBox[] joints = new IntBox[path.size() + 1];
    // Terminal joints come from the CONNECTION shapes (what Locate actually attaches to),
    // not the tree shapes: the channel must contain the attachment region or Locate's
    // approach degenerates to the sentinel corner. An unbounded trace-connection simplex has
    // a sentinel-sized bounding box; the intersection with the room bounds it.
    TileShape start_shape = ((app.freerouting.board.Connectable) p_route.start_item)
        .get_trace_connection_shape(tree, p_route.start_tree_entry_no);
    if (start_shape == null || start_shape.is_empty()) {
      return null;
    }
    joints[0] = start_shape.bounding_box().intersection(path.get(0).box);
    for (int i = 1; i < path.size(); i++) {
      joints[i] = path.get(i - 1).box.intersection(path.get(i).box);
    }
    TileShape target_shape = ((app.freerouting.board.Connectable) p_route.target_item)
        .get_trace_connection_shape(tree, p_route.target_tree_entry_no);
    if (target_shape == null || target_shape.is_empty()) {
      return null;
    }
    joints[path.size()] = target_shape.bounding_box().intersection(path.get(path.size() - 1).box);
    for (IntBox joint : joints) {
      if (joint.is_empty()) {
        return null;
      }
    }
    // Consecutive maximal rooms overlap over most of their length, so a raw joint is nearly
    // as large as the rooms and clipping to it changes nothing. Interior joints are therefore
    // narrowed to a WINDOW: the straight start-to-target aim line's interpolated point,
    // clamped into the joint (so the window stays in known-free overlap), inflated by the
    // pass width. The channels below then follow the aim line instead of the full corridors.
    double aim_from_x = (joints[0].ll.x + joints[0].ur.x) / 2.0;
    double aim_from_y = (joints[0].ll.y + joints[0].ur.y) / 2.0;
    double aim_to_x = (joints[path.size()].ll.x + joints[path.size()].ur.x) / 2.0;
    double aim_to_y = (joints[path.size()].ll.y + joints[path.size()].ur.y) / 2.0;
    // Array-pad escape: when a terminal is a detected array pad (BGA/ring/peripheral), the
    // only legal exit at fine pitch runs along the pad's escape axis -- a generic aim line
    // can point sideways into the neighbouring pad's clearance (measured as the dominant
    // residual reject). Move that terminal's aim origin to the escape point just beyond the
    // pad end, so the interpolated windows lead the channel out along the axis first.
    double[] start_escape = escape_aim(p_route.start_item, aim_to_x, aim_to_y, channel_margin);
    if (start_escape != null) {
      aim_from_x = start_escape[0];
      aim_from_y = start_escape[1];
    }
    double[] target_escape = escape_aim(p_route.target_item, aim_from_x, aim_from_y, channel_margin);
    if (target_escape != null) {
      aim_to_x = target_escape[0];
      aim_to_y = target_escape[1];
    }
    for (int i = 1; i < path.size(); i++) {
      double t = i / (double) path.size();
      int window_x = (int) Math.round(aim_from_x + t * (aim_to_x - aim_from_x));
      int window_y = (int) Math.round(aim_from_y + t * (aim_to_y - aim_from_y));
      window_x = Math.max(joints[i].ll.x, Math.min(joints[i].ur.x, window_x));
      window_y = Math.max(joints[i].ll.y, Math.min(joints[i].ur.y, window_y));
      joints[i] = new IntBox(window_x, window_y, window_x, window_y)
          .offset(channel_margin).intersection(joints[i]);
    }

    List<CompleteFreeSpaceExpansionRoom> rooms = new ArrayList<>(path.size());
    for (int i = 0; i < path.size(); i++) {
      IntBox channel = joints[i].union(joints[i + 1]).offset(channel_margin)
          .intersection(path.get(i).box);
      if (channel.is_empty()) {
        return null;
      }
      rooms.add(new CompleteFreeSpaceExpansionRoom(channel, p_route.layers[i], i + 1));
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
    ExpandableObject[] doors = new ExpandableObject[path.size() - 1];
    int[] door_section = new int[doors.length];
    for (int i = 0; i < doors.length; i++) {
      if (p_route.layers[i] != p_route.layers[i + 1]) {
        // Layer change: an ExpansionDrill at the joint window's centre. Locate ends the
        // current layer's trace at drill.location and continues on the next layer; Insert
        // turns the meeting point into a via. room_arr is indexed by absolute layer
        // (first_layer == 0), matching the backtrack walk's section semantics.
        int via_radius = Math.max(1, (int) Math.ceil(p_ctrl.max_via_radius));
        int via_x = (joints[i + 1].ll.x + joints[i + 1].ur.x) / 2;
        int via_y = (joints[i + 1].ll.y + joints[i + 1].ur.y) / 2;
        IntBox drill_shape = new IntBox(via_x - via_radius, via_y - via_radius,
            via_x + via_radius, via_y + via_radius);
        ExpansionDrill drill = new ExpansionDrill(drill_shape,
            new app.freerouting.geometry.planar.IntPoint(via_x, via_y), 0, partitions.length - 1);
        drill.room_arr[p_route.layers[i]] = rooms.get(i);
        drill.room_arr[p_route.layers[i + 1]] = rooms.get(i + 1);
        doors[i] = drill;
        door_section[i] = p_route.layers[i];
        continue;
      }
      ExpansionDoor door = new ExpansionDoor(rooms.get(i), rooms.get(i + 1));
      var sections = door.get_section_segments(p_ctrl.compensated_trace_half_width[p_route.layers[i]]);
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

  /**
   * The escape-axis aim point for a terminal item, or null when the item is not a detected
   * array pad. The point sits one channel-margin beyond the pad end along the escape axis;
   * for unsigned axes (ring/peripheral long-axis pads) the end nearer the other terminal is
   * chosen.
   */
  private double[] escape_aim(Item p_item, double p_toward_x, double p_toward_y, int p_margin) {
    if (!(p_item instanceof Pin pin)) {
      return null;
    }
    if (pad_array_detector == null) {
      pad_array_detector = new PadArrayDetector(board);
    }
    PadArrayDetector.PadEscape escape = pad_array_detector.escape_of(pin);
    if (escape == null || (escape.axis_x() == 0 && escape.axis_y() == 0)) {
      return null;
    }
    app.freerouting.geometry.planar.FloatPoint center = pin.get_center().to_float();
    int sign = 1;
    if (!escape.signed()) {
      double dot = escape.axis_x() * (p_toward_x - center.x) + escape.axis_y() * (p_toward_y - center.y);
      sign = dot >= 0 ? 1 : -1;
    }
    double reach = escape.half_extent() + p_margin;
    return new double[]{
        center.x + sign * escape.axis_x() * reach,
        center.y + sign * escape.axis_y() * reach};
  }

  private PadArrayDetector pad_array_detector;

  private static double heuristic(FreeSpacePartition.Room p_room, Set<FreeSpacePartition.Room> p_targets) {
    double best = Double.MAX_VALUE;
    for (FreeSpacePartition.Room target : p_targets) {
      best = Math.min(best, box_distance(p_room.box, target.box));
    }
    return best;
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
