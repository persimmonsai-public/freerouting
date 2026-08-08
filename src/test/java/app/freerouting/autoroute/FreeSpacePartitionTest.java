package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.geometry.planar.IntBox;
import app.freerouting.geometry.planar.TileShape;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.TreeSet;
import org.junit.jupiter.api.Test;

/**
 * Property tests for {@link FreeSpacePartition} over randomized mutation histories.
 *
 * <p>The central property is <b>update equivalence</b>: after any sequence of inserts and
 * removes, the partition must equal one built from scratch over the same final obstacle set.
 * Everything the router will rely on follows from that plus the geometric invariants checked
 * alongside: cells are pairwise disjoint, they exactly cover the free area (verified by
 * independent area accounting), and adjacency is symmetric and geometrically real.
 *
 * <p>Obstacles here are axis-aligned boxes, for which the slab clipping is exact, so the area
 * accounting is an equality rather than a bound. The 45-degree clipping path is exercised by
 * the feasibility measurement on the real board rather than by these synthetic properties.
 */
class FreeSpacePartitionTest {

  private static final IntBox BOUNDS = new IntBox(0, 0, 10000, 10000);

  @Test
  void randomizedHistories_matchFreshBuild_andKeepInvariants() {
    Random rnd = new Random(42);
    for (int round = 0; round < 5; round++) {
      FreeSpacePartition partition = new FreeSpacePartition(BOUNDS);
      java.util.Map<Integer, List<TileShape>> live = new java.util.HashMap<>();

      int operations = 120;
      for (int op = 0; op < operations; op++) {
        if (!live.isEmpty() && rnd.nextInt(4) == 0) {
          Integer victim = live.keySet().iterator().next();
          live.remove(victim);
          partition.remove(victim);
        } else {
          int id = rnd.nextInt(60);
          List<TileShape> shapes = new ArrayList<>();
          int shapeCount = 1 + rnd.nextInt(3);
          for (int i = 0; i < shapeCount; i++) {
            shapes.add(randomBox(rnd));
          }
          live.put(id, shapes);
          partition.insert(id, shapes);
        }
      }

      // --- update equivalence: incremental result == built-from-scratch result ---
      FreeSpacePartition fresh = new FreeSpacePartition(BOUNDS);
      for (var entry : live.entrySet()) {
        fresh.insert(entry.getKey(), entry.getValue());
      }
      assertEquals(canonical(fresh), canonical(partition),
          "incrementally maintained partition diverged from a from-scratch build (round " + round + ")");

      List<FreeSpacePartition.Cell> cells = partition.cells();

      // --- disjointness ---
      for (int i = 0; i < cells.size(); i++) {
        for (int j = i + 1; j < cells.size(); j++) {
          IntBox a = cells.get(i).box;
          IntBox b = cells.get(j).box;
          boolean overlaps = a.ll.x < b.ur.x && b.ll.x < a.ur.x && a.ll.y < b.ur.y && b.ll.y < a.ur.y;
          assertTrue(!overlaps, "cells overlap: " + boxString(a) + " and " + boxString(b));
        }
      }

      // --- area accounting: cells exactly cover bounds minus the obstacle union ---
      long cellArea = 0;
      for (FreeSpacePartition.Cell c : cells) {
        cellArea += area(c.box);
      }
      long expectedFree = area(BOUNDS) - obstacleUnionArea(live);
      assertEquals(expectedFree, cellArea,
          "cell areas do not sum to the free area (round " + round + ", cells=" + cells.size() + ")");

      // --- adjacency: symmetric and geometrically real ---
      for (FreeSpacePartition.Cell c : cells) {
        for (FreeSpacePartition.Cell n : partition.neighbors(c)) {
          assertTrue(partition.neighbors(n).contains(c), "adjacency not symmetric");
          boolean sharesEdge = (c.box.ur.x == n.box.ll.x || n.box.ur.x == c.box.ll.x)
              && c.box.ll.y < n.box.ur.y && n.box.ll.y < c.box.ur.y;
          assertTrue(sharesEdge, "neighbors do not share a vertical border: "
              + boxString(c.box) + " and " + boxString(n.box));
        }
      }
    }
  }

  private IntBox randomBox(Random rnd) {
    int x = rnd.nextInt(9000);
    int y = rnd.nextInt(9000);
    int w = 50 + rnd.nextInt(2500);
    int h = 50 + rnd.nextInt(400);
    if (rnd.nextBoolean()) {
      int t = w;
      w = h;
      h = t;
    }
    return new IntBox(x, y, Math.min(10000, x + w), Math.min(10000, y + h));
  }

  private static long area(IntBox b) {
    return (long) (b.ur.x - b.ll.x) * (b.ur.y - b.ll.y);
  }

  private static String boxString(IntBox b) {
    return "(" + b.ll.x + "," + b.ll.y + ")-(" + b.ur.x + "," + b.ur.y + ")";
  }

  private static Set<String> canonical(FreeSpacePartition p) {
    Set<String> result = new HashSet<>();
    for (FreeSpacePartition.Cell c : p.cells()) {
      result.add(boxString(c.box));
    }
    return result;
  }

  /**
   * Independent union-area computation (slab sweep over the raw boxes), sharing no code with
   * the class under test.
   */
  private static long obstacleUnionArea(java.util.Map<Integer, List<TileShape>> live) {
    List<IntBox> boxes = new ArrayList<>();
    for (List<TileShape> shapes : live.values()) {
      for (TileShape s : shapes) {
        boxes.add(s.bounding_box());
      }
    }
    TreeSet<Integer> xs = new TreeSet<>();
    for (IntBox b : boxes) {
      xs.add(b.ll.x);
      xs.add(b.ur.x);
    }
    long total = 0;
    Integer left = xs.isEmpty() ? null : xs.first();
    while (left != null) {
      Integer right = xs.higher(left);
      if (right == null) {
        break;
      }
      List<int[]> intervals = new ArrayList<>();
      for (IntBox b : boxes) {
        if (b.ll.x <= left && b.ur.x >= right) {
          intervals.add(new int[]{b.ll.y, b.ur.y});
        }
      }
      intervals.sort(Comparator.comparingInt(a -> a[0]));
      long covered = 0;
      int cursor = Integer.MIN_VALUE;
      for (int[] iv : intervals) {
        int lo = Math.max(iv[0], cursor);
        if (iv[1] > lo) {
          covered += iv[1] - lo;
          cursor = iv[1];
        }
        cursor = Math.max(cursor, iv[1]);
      }
      total += covered * (long) (right - left);
      left = right;
    }
    return total;
  }
}
