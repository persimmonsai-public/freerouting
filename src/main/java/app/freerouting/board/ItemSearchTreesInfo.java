package app.freerouting.board;

import app.freerouting.datastructures.ShapeTree;
import app.freerouting.geometry.planar.TileShape;
import java.util.Collection;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * Stores information about the search trees of the board items, which is precalculated for performance reasons.
 *
 * <p>Backed by {@link CopyOnWriteArrayList} rather than a plain list: when the parallel
 * autorouter is active, multiple worker threads search against their own private scratch trees
 * (see {@code SearchTreeManager#build_scratch_search_tree}) but concurrently read and lazily
 * populate the precalculated-shape cache on the SAME shared {@code Item} objects. A plain list
 * would corrupt under that access pattern (concurrent {@code add()} calls racing, or a reader's
 * iterator crossing a writer's structural mutation); {@code CopyOnWriteArrayList} is safe for
 * exactly this shape of workload -- reads vastly outnumber writes, and each item typically
 * belongs to only a handful of trees, so the copy-on-write cost is small.
 */
class ItemSearchTreesInfo {

  private final Collection<SearchTreeInfo> tree_list;

  /**
   * Creates a new instance of ItemSearchTreeEntries
   */
  public ItemSearchTreesInfo() {
    this.tree_list = new CopyOnWriteArrayList<>();
  }

  /**
   * Returns the tree entries for the tree with identification number p_tree_no, or null, if for this tree no entries of this item are inserted.
   */
  public ShapeTree.Leaf[] get_tree_entries(ShapeTree p_tree) {
    for (SearchTreeInfo curr_tree_info : this.tree_list) {
      if (curr_tree_info.tree == p_tree) {
        return curr_tree_info.entry_arr;
      }
    }
    return null;
  }

  /**
   * Sets the item tree entries for the tree with identification number p_tree_no.
   */
  public void set_tree_entries(ShapeTree.Leaf[] p_tree_entries, ShapeTree p_tree) {
    for (SearchTreeInfo curr_tree_info : this.tree_list) {
      if (curr_tree_info.tree == p_tree) {
        curr_tree_info.entry_arr = p_tree_entries;
        return;
      }
    }
    SearchTreeInfo new_tree_info = new SearchTreeInfo(p_tree);
    new_tree_info.entry_arr = p_tree_entries;
    this.tree_list.add(new_tree_info);
  }

  /**
   * Returns the precalculated tiles shapes for the tree with identification number p_tree_no, or null, if the tile shapes of this tree are not yet precalculated.
   */
  public TileShape[] get_precalculated_tree_shapes(ShapeTree p_tree) {
    for (SearchTreeInfo curr_tree_info : this.tree_list) {
      if (curr_tree_info.tree == p_tree) {
        return curr_tree_info.precalculated_tree_shapes;
      }
    }
    return null;
  }

  /**
   * Sets the item tree entries for the tree with identification number p_tree_no.
   */
  public void set_precalculated_tree_shapes(TileShape[] p_tile_shapes, ShapeTree p_tree) {
    for (SearchTreeInfo curr_tree_info : this.tree_list) {
      if (curr_tree_info.tree == p_tree) {
        curr_tree_info.precalculated_tree_shapes = p_tile_shapes;
        return;
      }
    }
    SearchTreeInfo new_tree_info = new SearchTreeInfo(p_tree);
    new_tree_info.precalculated_tree_shapes = p_tile_shapes;
    this.tree_list.add(new_tree_info);
  }

  /**
   * Clears the stored information about the precalculated tree shapes for all search trees --
   * EXCEPT private per-worker scratch trees ({@link ShapeSearchTree#isPrivateScratchTree}).
   *
   * <p>This is called whenever an item's geometry structurally changes, to force every tree to
   * recompute its cached shapes on next use. That's correct for the board's real, shared trees.
   * But a scratch tree built by {@code SearchTreeManager#build_scratch_search_tree} is a frozen,
   * point-in-time snapshot owned by one parallel-autorouter worker -- its structural expectations
   * (which shape indices exist, how many) were fixed when the snapshot was taken and must not be
   * disturbed by some OTHER thread's later, unrelated commit changing this same shared item.
   * Wiping the scratch tree's cache here too would make a later {@code get_tree_shape} call on it
   * recompute against the item's CURRENT (changed) geometry -- inconsistent with what the
   * worker's own tree structure still expects -- and return null/mismatched shapes. Before this
   * exclusion, that surfaced under load as NullPointerExceptions deep in the maze search,
   * proportional to how often other workers were committing.
   */
  public void clear_precalculated_tree_shapes() {
    for (SearchTreeInfo curr_tree_info : this.tree_list) {
      if (curr_tree_info.tree instanceof ShapeSearchTree sst && sst.isPrivateScratchTree) {
        continue;
      }
      curr_tree_info.precalculated_tree_shapes = null;
    }
  }

  /**
   * Drops every entry EXCEPT private per-worker scratch trees ({@link ShapeSearchTree#isPrivateScratchTree}).
   *
   * <p>Called (via {@code Item#clear_search_tree_entries}) when an item is removed from the
   * board's real, shared search trees -- e.g. as a rip-up victim during a parallel autorouter
   * commit. The item really is gone from the shared trees, so their entries must go. But another
   * worker's private scratch-tree snapshot may still be actively searching with this same item
   * as a frozen obstacle; dropping its entries too (the old behavior: null out this whole object)
   * made that worker's very next shape lookup on this item fail with a NullPointerException,
   * rather than just seeing a stale-but-internally-consistent view of an item that no longer
   * exists on the real board (harmless -- the search's eventual candidate route is re-validated
   * against the real board at commit time regardless).
   *
   * @return true if any scratch-tree entries were retained (so the caller can keep this object
   *     instead of discarding it)
   */
  public boolean retain_only_private_scratch_tree_entries() {
    boolean retained_any = false;
    for (SearchTreeInfo curr_tree_info : this.tree_list) {
      if (curr_tree_info.tree instanceof ShapeSearchTree sst && sst.isPrivateScratchTree) {
        retained_any = true;
      } else {
        this.tree_list.remove(curr_tree_info);
      }
    }
    return retained_any;
  }

  private static class SearchTreeInfo {

    final ShapeTree tree;
    ShapeTree.Leaf[] entry_arr;
    TileShape[] precalculated_tree_shapes;

    SearchTreeInfo(ShapeTree p_tree) {
      tree = p_tree;
      entry_arr = null;
      precalculated_tree_shapes = null;
    }
  }
}