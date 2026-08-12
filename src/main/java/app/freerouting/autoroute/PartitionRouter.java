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
    public final Item start_item;
    public final int start_tree_entry_no;
    public final Item target_item;
    public final int target_tree_entry_no;

    CellRoute(int p_layer, List<FreeSpacePartition.Room> p_rooms, Item p_start_item,
        int p_start_entry, Item p_target_item, int p_target_entry) {
      this.layer = p_layer;
      this.rooms = p_rooms;
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
  /**
   * Congestion prices for negotiation, keyed by room BOX (stable across partition rebuilds,
   * unlike Room object identity). Null disables pricing.
   */
  private Map<String, Double> room_prices;

  public void set_room_prices(Map<String, Double> p_prices) {
    this.room_prices = p_prices;
  }

  public static String box_key(IntBox p_box) {
    return p_box.ll.x + ":" + p_box.ll.y + ":" + p_box.ur.x + ":" + p_box.ur.y;
  }

  private double price_of(IntBox p_box) {
    if (room_prices == null) {
      return 0;
    }
    Double price = room_prices.get(box_key(p_box));
    return price == null ? 0 : price;
  }

  public PartitionRouter(RoutingBoard p_board, ShapeSearchTree p_tree) {
    this.board = p_board;
    this.tree = p_tree;
    this.partitions = new FreeSpacePartition[p_board.get_layer_count()];
  }

  /**
   * Builds everything overlay-mode queries touch (partitions, room covers, adjacency) so that
   * subsequent {@code try_route(..., p_lift=false)} calls are pure reads -- the precondition
   * for running dry-run searches on multiple threads.
   */
  public void warm_up() {
    ensure_fresh();
    for (FreeSpacePartition partition : partitions) {
      if (partition != null) {
        // rooms() builds cells, the room cover and the room adjacency in one go.
        partition.rooms();
      }
    }
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
        if (item instanceof app.freerouting.board.ConductionArea pour && !pour.get_is_obstacle()
            && app.freerouting.Freerouting.globalSettings != null
            && app.freerouting.Freerouting.globalSettings.featureFlags.reflowablePours) {
          // Under pour-reflow modelling only: passable pours stop being partition walls
          // (walling them erased free space on the pour-covered 8-layer board -- zero dry-run
          // routes). WITHOUT reflow mode they stay walls: routing through pours then commits
          // corridor-stealing routes the endgame pays for (measured: 989.72/2 -> 974.34/5).
          continue;
        }
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
  public CellRoute try_route(Set<Item> p_start_set, Set<Item> p_dest_set, int[] p_half_width) {
    return try_route(p_start_set, p_dest_set, p_half_width, true);
  }

  /**
   * As above; p_lift false runs the OVERLAY mode: no physical net-lift, no partition
   * mutation, no dirtying -- own-net transparency is applied at CONTACT level instead
   * (a contact is accepted when the item's connection shape intersects the room OR the
   * item's own footprint, which is exactly the region a lifted room would grow over).
   * The shared partition stays fresh across any number of overlay searches, so the
   * per-search lift -> dirty -> wholesale cells+rooms rebuild cycle (~70 ms at 2-layer,
   * ~122 ms at 8-layer) disappears; negotiation dry rounds use this mode.
   */
  public CellRoute try_route(Set<Item> p_start_set, Set<Item> p_dest_set, int[] p_half_width,
      boolean p_lift) {
    return try_route(p_start_set, p_dest_set, p_half_width, p_lift, null);
  }

  /**
   * As above; p_partner_room_keys (nullable) is the pair-corridor affinity input: rooms
   * whose {@link #box_key} is in the set get a 10 percent distance discount and a full
   * congestion-price waiver, drawing this member's search toward its differential
   * partner's corridor. Read-only; deterministic; thread-safe (the set is never mutated
   * during a round).
   */
  public CellRoute try_route(Set<Item> p_start_set, Set<Item> p_dest_set, int[] p_half_width,
      boolean p_lift, Set<String> p_partner_room_keys) {
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
    if (p_lift) {
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
    }
    try {
      CellRoute best = null;
      double best_cost = Double.MAX_VALUE;
      for (int layer = 0; layer < partitions.length; layer++) {
        if (partitions[layer] == null) {
          continue;
        }
        CellRoute route = try_route_on_layer(layer, p_start_set, p_dest_set,
            Math.max(1, p_half_width[layer]), !p_lift, p_partner_room_keys);
        if (route != null) {
          double cost = route_length(route);
          // Quality guard: a partition route much longer than the straight terminal distance
          // is a detour that blocks corridors the endgame needs -- committing such routes was
          // measured to drop the final score from 994.85/1 to baseline 989.72/2 when the
          // sentinel fix doubled the raw hit rate. Let the classic engine route those.
          double direct = center_distance(route.rooms.get(0).box,
              route.rooms.get(route.rooms.size() - 1).box);
          if (cost > 1.4 * direct + 20000) {
            continue;
          }
          if (cost < best_cost) {
            best_cost = cost;
            best = route;
          }
        }
      }
      return best;
    } finally {
      if (p_lift) {
        // Every lifted attempt is followed by invalidate() from the caller (staleness-vs-
        // quality tradeoff measured in BatchAutorouter), so re-inserting the lifted net into
        // partitions that are about to be rebuilt wholesale is pure waste; just mark dirty.
        // Overlay searches never mutate the partition, so it stays fresh.
        dirty = true;
      }
    }
  }

  private static double route_length(CellRoute p_route) {
    double result = 0;
    for (int i = 0; i + 1 < p_route.rooms.size(); i++) {
      result += center_distance(p_route.rooms.get(i).box, p_route.rooms.get(i + 1).box);
    }
    return result;
  }

  private CellRoute try_route_on_layer(int p_layer, Set<Item> p_start_set, Set<Item> p_dest_set,
      int p_half_width, boolean p_own_net_transparent, Set<String> p_partner_room_keys) {
    FreeSpacePartition partition = partitions[p_layer];
    // Rooms abutting an item shape, with the shape's tree entry number for door construction.
    // Both sides require the CONNECTION shape to reach the room: Locate walks toward the
    // start item's connection shape intersected with the start room (a trace's connection
    // shape exists only near its endpoints), and an empty intersection makes its target
    // corner land on the 2^25 sentinel coordinate -- measured as the dominant reject.
    Map<FreeSpacePartition.Room, ItemContact> starts =
        contact_rooms(partition, p_start_set, p_layer, p_half_width, p_own_net_transparent);
    Map<FreeSpacePartition.Room, ItemContact> targets =
        contact_rooms(partition, p_dest_set, p_layer, p_half_width, p_own_net_transparent);
    if (starts.isEmpty() || targets.isEmpty()) {
      return null;
    }

    // A* over the room cover: g = accumulated center distance, h = distance to the nearest
    // target box. Rooms are overlapping maximal rectangles, so admission is about interiors
    // and overlaps rather than shared borders:
    //  - an INTERMEDIATE room must be wide enough in both dimensions for Locate's erosion by
    //    the compensated half width to leave interior (thin rooms measured to degenerate
    //    corner placement); start/target rooms are exempt because attachment there is handled
    //    by the item connection shapes, not by erosion;
    //  - a transition is passable when the overlap region's LONG dimension hosts the trace
    //    width (the trace crosses perpendicular to the overlap's thin dimension).
    int min_pass = 2 * (p_half_width + AutorouteEngine.TRACE_WIDTH_TOLERANCE) + 2;
    List<FreeSpacePartition.Room> rooms = partition.rooms();
    double[] g = new double[rooms.size()];
    int[] came_from = new int[rooms.size()];
    java.util.Arrays.fill(g, Double.MAX_VALUE);
    java.util.Arrays.fill(came_from, -1);
    // Tie-break equal costs on room index: PriorityQueue pop order among equal keys is
    // arbitrary, and tie-prone costs have twice turned identity-ordered iteration into
    // nondeterministic final scores (ledger: heuristic experiment, commit-phase ordering).
    PriorityQueue<double[]> open = new PriorityQueue<>((a, b) -> {
      int by_cost = Double.compare(a[0], b[0]);
      return by_cost != 0 ? by_cost : Double.compare(a[1], b[1]);
    });
    for (FreeSpacePartition.Room start : starts.keySet()) {
      // Thin terminal rooms are excluded outright: Locate erodes start/target rooms exactly
      // like intermediate ones, and a thin terminal degenerates the whole corner walk
      // (measured: rejected plans' first segments ran ALONG the pin row, clipping the
      // neighbouring pins over 100k+ units). Better a clean no-route here than degenerate
      // geometry rejected after realization.
      if (start.box.ur.x - start.box.ll.x < min_pass || start.box.ur.y - start.box.ll.y < min_pass) {
        continue;
      }
      g[start.index] = 0;
      open.add(new double[]{heuristic(start, targets.keySet()), start.index});
    }
    FreeSpacePartition.Room reached = null;
    boolean[] closed = new boolean[rooms.size()];
    while (!open.isEmpty()) {
      int current = (int) open.poll()[1];
      if (closed[current]) {
        continue;
      }
      closed[current] = true;
      FreeSpacePartition.Room room = rooms.get(current);
      if (targets.containsKey(room)) {
        reached = room;
        break;
      }
      for (FreeSpacePartition.Room neighbor : partition.room_neighbors(room)) {
        if (closed[neighbor.index]) {
          continue;
        }
        if (neighbor.box.ur.x - neighbor.box.ll.x < min_pass
            || neighbor.box.ur.y - neighbor.box.ll.y < min_pass) {
          continue; // too thin for Locate's interior erosion (terminal rooms included)
        }
        int dx = Math.min(room.box.ur.x, neighbor.box.ur.x) - Math.max(room.box.ll.x, neighbor.box.ll.x);
        int dy = Math.min(room.box.ur.y, neighbor.box.ur.y) - Math.max(room.box.ll.y, neighbor.box.ll.y);
        if (Math.max(dx, dy) < min_pass) {
          continue; // the overlap cannot host a trace-wide crossing
        }
        boolean partner_room = p_partner_room_keys != null
            && p_partner_room_keys.contains(box_key(neighbor.box));
        double candidate = g[current]
            + center_distance(room.box, neighbor.box) * (partner_room ? 0.9 : 1.0)
            + (partner_room ? 0 : price_of(neighbor.box));
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
    List<FreeSpacePartition.Room> path = new ArrayList<>();
    for (int at = reached.index; at != -1; at = came_from[at]) {
      path.add(0, rooms.get(at));
    }
    ItemContact start_contact = starts.get(path.get(0));
    ItemContact target_contact = targets.get(reached);
    return new CellRoute(p_layer, path, start_contact.item, start_contact.tree_entry_no,
        target_contact.item, target_contact.tree_entry_no);
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
   * to that item and shape index. Iteration order of the returned map is deterministic:
   * items are visited in id order and rooms in partition list order (seeding order feeds the
   * A* tie-breaks, so identity-hash order here would make outcomes run-dependent).
   *
   * <p>p_own_net_transparent is the overlay mode's contact rule: the physical net-lift grows
   * rooms over the removed items' footprints, so a lifted room intersects the connection
   * shape whenever the shared-partition room OR the item's own footprint does. Testing
   * connection-vs-own-shape reproduces that acceptance without mutating the partition (the
   * pad-centre connection shape of a drill item always intersects its own pad).
   */
  private Map<FreeSpacePartition.Room, ItemContact> contact_rooms(FreeSpacePartition p_partition,
      Set<Item> p_items, int p_layer, int p_touch_margin, boolean p_own_net_transparent) {
    Map<FreeSpacePartition.Room, ItemContact> result = new java.util.LinkedHashMap<>();
    List<Item> items = new ArrayList<>(p_items);
    items.sort((a, b) -> Integer.compare(a.get_id_no(), b.get_id_no()));
    for (Item item : items) {
      int shape_count = item.tree_shape_count(tree);
      for (int i = 0; i < shape_count; i++) {
        if (item.shape_layer(i) != p_layer) {
          continue;
        }
        TileShape shape = item.get_tree_shape(tree, i);
        if (shape == null || shape.is_empty()) {
          continue;
        }
        // Locate's destination side seeds from the item's CONNECTION shape (for traces it
        // exists only near the endpoints), so a contact that only touches the tree shape
        // is unusable as a target.
        TileShape connection = ((app.freerouting.board.Connectable) item)
            .get_trace_connection_shape(tree, i);
        if (connection == null || connection.is_empty()) {
          continue;
        }
        boolean own_footprint_contact = p_own_net_transparent
            && !connection.intersection(shape).is_empty();
        IntBox probe = shape.bounding_box().offset(p_touch_margin);
        for (FreeSpacePartition.Room room : p_partition.rooms_intersecting(probe)) {
          // The bounding-box probe over-selects around diagonal shapes; the door built later
          // needs the ACTUAL shape to intersect the room (abutting closed regions share their
          // border segment, which is enough).
          if (shape.intersection(room.box).is_empty()) {
            continue;
          }
          if (!own_footprint_contact && connection.intersection(room.box).is_empty()) {
            continue;
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
    return materialize(p_route, p_ctrl, app.freerouting.Freerouting.globalSettings != null
        && app.freerouting.Freerouting.globalSettings.featureFlags.liveChannelValidation);
  }

  /**
   * As above; p_live_validation false realizes the plan from the channels exactly as the
   * search produced them (the flag-off geometry), which is the retry path when the validated
   * channels turn out not to be realizable.
   */
  public MazeSearchAlgo.Result materialize(CellRoute p_route, AutorouteControl p_ctrl,
      boolean p_live_validation) {
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
    // NOTE (measured): halving this margin (half_width + tol + 2) did NOT contain the
    // dogleg-clipping rejects -- 2-layer negotiation rejects went 13 -> 14 and conversions
    // 3 -> 2 (994.85/1 held by luck of the different trajectory). The 2x margin stands.
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
    boolean live_validation = p_live_validation;
    for (int i = 1; i < path.size(); i++) {
      double t = i / (double) path.size();
      int window_x = (int) Math.round(aim_from_x + t * (aim_to_x - aim_from_x));
      int window_y = (int) Math.round(aim_from_y + t * (aim_to_y - aim_from_y));
      // Under live validation the crossing point is clamped into the joint ERODED by the
      // compensated half width, per axis: the door crossing is where consecutive maximal rooms
      // overlap, so a raw clamp puts it against the item that bounds the overlap and the trace
      // body -- half a width around it -- overlaps that item. Where an axis is too thin to
      // erode, the joint's midpoint on that axis is the best available compromise.
      IntBox clamp_box = live_validation ? eroded_per_axis(joints[i], channel_margin / 2) : joints[i];
      window_x = Math.max(clamp_box.ll.x, Math.min(clamp_box.ur.x, window_x));
      window_y = Math.max(clamp_box.ll.y, Math.min(clamp_box.ur.y, window_y));
      joints[i] = new IntBox(window_x, window_y, window_x, window_y)
          .offset(channel_margin).intersection(joints[i]);
    }

    if (live_validation) {
      ++live_plan_count;
    }
    List<CompleteFreeSpaceExpansionRoom> rooms = new ArrayList<>(path.size());
    List<IntBox> channels = new ArrayList<>(path.size());
    for (int i = 0; i < path.size(); i++) {
      IntBox channel = joints[i].union(joints[i + 1]).offset(channel_margin)
          .intersection(path.get(i).box);
      if (channel.is_empty()) {
        return null;
      }
      if (live_validation) {
        channel = live_free_channel(channel, joints[i], joints[i + 1], layer, p_ctrl, half_width);
        if (channel == null) {
          ++live_plan_reject_count;
          return null;
        }
      }
      channels.add(channel);
      rooms.add(new CompleteFreeSpaceExpansionRoom(channel, layer, i + 1));
    }
    // Published for the checked realizer: these boxes are the plan's known-free space, so a
    // repaired corner that stays inside them stays inside space the partition proved free.
    this.last_channel_boxes = channels;

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

  /**
   * Live channel validation (failure class 2 of the materialization taxonomy): returns a
   * sub-box of p_channel whose TRACE CORRIDOR -- the box inflated by the pen half width the
   * pre-insert DRC uses -- is free of foreign obstacles in the LIVE default search tree, or
   * null when no such sub-box exists.
   *
   * <p>The partition's room cover proves free space against the compensated autoroute tree as
   * it stood when the partition was last rebuilt, and it models the trace as a POINT: a
   * centerline anywhere inside a free room still puts half a trace width plus clearance
   * outside it. Both gaps produce plans whose channels look clean in the partition's model and
   * fail the DRC on the live board. This validation closes both with the DRC's own query.
   *
   * <p>Two steps, both measured necessary (see the roadmap increment):
   *
   * <ol>
   *   <li><b>Compensation erosion.</b> The channel is eroded by the compensated half width, so
   *       a centerline anywhere inside it keeps the whole trace body inside the plan's own free
   *       room even where Locate falls back to the raw room shape (it does that whenever its
   *       own shrink empties) and around door-crossing doglegs, which it never checks. The two
   *       door crossing points are kept by construction, so connectivity survives.
   *   <li><b>Live tree check.</b> The eroded corridor is then queried against the LIVE default
   *       search tree with the DRC's own clearance rule, which catches what the room cover
   *       cannot model at all: items inserted since the partition was built, and clearance
   *       classes other than the one the partition's tree was built for.
   * </ol>
   *
   * <p>The pre-insert DRC still gates the commit afterwards; this can only remove space from a
   * plan, never authorise geometry the DRC would refuse.
   */
  private IntBox live_free_channel(IntBox p_channel, IntBox p_entry, IntBox p_exit, int p_layer,
      AutorouteControl p_ctrl, int p_compensated_half_width) {
    int pen_half_width = p_ctrl.trace_half_width[p_layer];
    int[] net_arr = new int[]{p_ctrl.net_no};
    int erosion = p_compensated_half_width + AutorouteEngine.TRACE_WIDTH_TOLERANCE + 1;
    IntBox current = compensated_channel(p_channel, p_entry, p_exit, erosion);
    if (current.area() < p_channel.area()) {
      ++live_shrink_count;
    }
    ShapeSearchTree live_tree = board.search_tree_manager.get_default_tree();
    if (live_free(current, p_layer, pen_half_width, net_arr, p_ctrl.trace_clearance_class_no,
        live_tree, p_channel)) {
      return current;
    }
    ++live_occupied_channel_count;
    if (LIVE_CHANNEL_DEBUG) {
      app.freerouting.logger.FRLogger.info("[live-channel-reject] net=" + p_ctrl.net_no
          + " layer=" + p_layer + " pen_hw=" + pen_half_width
          + " comp_hw=" + p_compensated_half_width + " channel=" + box_key(p_channel)
          + " validated=" + box_key(current) + " blocker=" + last_blocker_description);
    }
    return null; // no live-free channel: unrepairable in channel, rejected before geometry
  }

  /**
   * Whether the trace corridor of p_box -- the box inflated by the pen half width, which is
   * what the pre-insert DRC measures against -- is free of foreign obstacles in the live tree.
   * p_full_channel is the unshrunk channel, used only for the stale-vs-compensation
   * diagnostic split.
   */
  private boolean live_free(IntBox p_box, int p_layer, int p_pen_half_width, int[] p_net_arr,
      int p_clearance_class, ShapeSearchTree p_live_tree, IntBox p_full_channel) {
    IntBox corridor = p_box.offset(p_pen_half_width);
    for (app.freerouting.datastructures.ShapeTree.TreeEntry entry
        : p_live_tree.overlapping_tree_entries_with_clearance(corridor, p_layer, p_net_arr,
            p_clearance_class)) {
      if (!(entry.object instanceof Item item) || !item.is_trace_obstacle(p_net_arr[0])) {
        continue;
      }
      TileShape shape = item.get_tree_shape(p_live_tree, entry.shape_index_in_object);
      if (shape == null || shape.is_empty()) {
        continue;
      }
      // The tree query is bounding-box based and "may also return items which are nearly
      // overlapping" (its own words); without this exact test the predicate rejected every
      // plan on the reference board -- measured, the first version of this validation.
      if (shape.intersection(corridor).is_empty()) {
        continue;
      }
      if (LIVE_CHANNEL_DEBUG) {
        last_blocker_description = item.getClass().getSimpleName() + "#" + item.get_id_no()
            + " box=" + box_key(shape.bounding_box());
      }
      if (!shape.intersection(p_full_channel).is_empty()) {
        // The obstacle sits in the room cover's own free space: a stale partition (or a
        // clearance class the partition's tree was not built for), not the trace-width
        // compensation gap. Counted separately -- the two want different fixes.
        ++live_stale_channel_count;
      }
      return false;
    }
    return true;
  }

  /**
   * The channel a compensated trace can actually use: p_channel eroded by p_erosion (half the
   * compensated trace width plus tolerance), unioned with the two door crossing points so the
   * plan's connectivity survives, and falling back to those points alone when the channel is
   * too thin to erode at all. Sound by construction: the result is always a sub-box of the
   * channel, and always contains both crossing points.
   */
  static IntBox compensated_channel(IntBox p_channel, IntBox p_entry, IntBox p_exit,
      int p_erosion) {
    // The door crossing points are Locate's aim targets and must stay inside the channel; the
    // eroded box is unioned with them, which is the minimum the plan's connectivity needs.
    IntBox keep = joint_centre(p_entry).union(joint_centre(p_exit)).intersection(p_channel);
    IntBox eroded = new IntBox(p_channel.ll.x + p_erosion, p_channel.ll.y + p_erosion,
        p_channel.ur.x - p_erosion, p_channel.ur.y - p_erosion);
    return eroded.is_empty() ? keep : eroded.union(keep);
  }

  /**
   * p_box eroded by p_erosion on each axis independently; an axis too thin to erode collapses
   * to its midpoint instead of emptying the box.
   */
  static IntBox eroded_per_axis(IntBox p_box, int p_erosion) {
    int ll_x = p_box.ll.x + p_erosion;
    int ur_x = p_box.ur.x - p_erosion;
    if (ll_x > ur_x) {
      ll_x = ur_x = (p_box.ll.x + p_box.ur.x) / 2;
    }
    int ll_y = p_box.ll.y + p_erosion;
    int ur_y = p_box.ur.y - p_erosion;
    if (ll_y > ur_y) {
      ll_y = ur_y = (p_box.ll.y + p_box.ur.y) / 2;
    }
    return new IntBox(ll_x, ll_y, ur_x, ur_y);
  }

  /**
   * The joint's centre as a degenerate box -- the door crossing point Locate aims at, which
   * every shrunk channel must keep.
   */
  static IntBox joint_centre(IntBox p_joint) {
    int cx = (int) Math.round((p_joint.ll.x + (double) p_joint.ur.x) / 2.0);
    int cy = (int) Math.round((p_joint.ll.y + (double) p_joint.ur.y) / 2.0);
    return new IntBox(cx, cy, cx, cy);
  }

  /**
   * Per-reject channel/obstacle dump for diagnosis runs (-Dfr.livechannel.debug); off by
   * default so the validation costs one tree query per channel and nothing else.
   */
  private static final boolean LIVE_CHANNEL_DEBUG = Boolean.getBoolean("fr.livechannel.debug");
  private String last_blocker_description = "";
  private long live_plan_count;
  private long live_plan_reject_count;
  private long live_occupied_channel_count;
  private long live_stale_channel_count;
  private long live_shrink_count;

  /**
   * Live channel validation counters since the last {@link #reset_live_channel_counters()}:
   * plans validated, plans rejected early, channels found occupied on the live tree, of those
   * occupied because the partition was stale (rather than because of trace-width/clearance
   * compensation the room cover does not model), and channels successfully shrunk to a
   * live-free sub-box.
   */
  public long[] live_channel_counters() {
    return new long[]{live_plan_count, live_plan_reject_count, live_occupied_channel_count,
        live_stale_channel_count, live_shrink_count};
  }

  public void reset_live_channel_counters() {
    live_plan_count = 0;
    live_plan_reject_count = 0;
    live_occupied_channel_count = 0;
    live_stale_channel_count = 0;
    live_shrink_count = 0;
  }

  private List<IntBox> last_channel_boxes = List.of();

  /**
   * The windowed channel boxes of the most recently materialized plan (same order as its
   * room path) -- the legal search space for corner repair.
   */
  public List<IntBox> last_channel_boxes() {
    return last_channel_boxes;
  }

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
