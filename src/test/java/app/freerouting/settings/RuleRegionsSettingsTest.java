package app.freerouting.settings;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.util.gson.GsonProvider;
import org.junit.jupiter.api.Test;

/**
 * JSON plumbing and helper semantics for the router.rule_regions setting (region-scoped
 * rule overrides, docs/dense-bga-roadmap.md Phase 1 item 1).
 */
class RuleRegionsSettingsTest {

  @Test
  void gsonDeserializesTheFieldFromTheRouterSection() {
    RouterSettings settings = GsonProvider.GSON.fromJson(
        "{\"rule_regions\": [{\"layers\": \"*\", \"box_um\": [500, -7000, 19500, -3000],"
            + " \"clearance_um\": 80, \"trace_halfwidth_um\": 75}]}",
        RouterSettings.class);
    assertEquals(1, settings.getRuleRegions().length);
    RuleRegionSettings region = settings.getRuleRegions()[0];
    assertEquals("*", region.layers);
    assertArrayEquals(new double[] {500, -7000, 19500, -3000}, region.boxUm, 1e-9);
    assertEquals(80.0, region.clearanceUm, 1e-9);
    assertEquals(75.0, region.traceHalfwidthUm, 1e-9);
  }

  @Test
  void traceHalfwidthIsOptional() {
    RouterSettings settings = GsonProvider.GSON.fromJson(
        "{\"rule_regions\": [{\"box_um\": [0, 0, 100, 100], \"clearance_um\": 76.2}]}",
        RouterSettings.class);
    assertEquals(1, settings.getRuleRegions().length);
    assertEquals(76.2, settings.getRuleRegions()[0].clearanceUm, 1e-9);
    assertEquals(null, settings.getRuleRegions()[0].traceHalfwidthUm);
  }

  @Test
  void gsonSerializesTheFieldUnderItsSerializedName() {
    RouterSettings settings = new RouterSettings();
    RuleRegionSettings region = new RuleRegionSettings();
    region.boxUm = new double[] {0, 0, 100, 100};
    region.clearanceUm = 80.0;
    settings.ruleRegions = new RuleRegionSettings[] {region};
    String json = GsonProvider.GSON.toJson(settings);
    assertTrue(json.contains("\"rule_regions\""), "expected rule_regions in: " + json);
    assertTrue(json.contains("\"clearance_um\": 80.0"), "expected clearance_um in: " + json);
  }

  @Test
  void defaultIsNoRegions() {
    assertEquals(0, new RouterSettings().getRuleRegions().length);
  }

  @Test
  void cloneDeepCopiesTheRegions() {
    RouterSettings settings = new RouterSettings();
    RuleRegionSettings region = new RuleRegionSettings();
    region.layers = "0,1";
    region.boxUm = new double[] {1, 2, 3, 4};
    region.clearanceUm = 76.2;
    settings.ruleRegions = new RuleRegionSettings[] {region};
    RouterSettings copy = settings.clone();
    assertEquals(1, copy.getRuleRegions().length);
    assertNotSame(settings.ruleRegions[0], copy.ruleRegions[0]);
    assertNotSame(settings.ruleRegions[0].boxUm, copy.ruleRegions[0].boxUm);
    assertEquals("0,1", copy.ruleRegions[0].layers);
    assertEquals(76.2, copy.ruleRegions[0].clearanceUm, 1e-9);
  }

  @Test
  void mergeCopiesTheFieldFromAHigherPrioritySource() {
    RouterSettings target = new RouterSettings();
    RouterSettings source = new RouterSettings();
    RuleRegionSettings region = new RuleRegionSettings();
    region.boxUm = new double[] {0, 0, 10, 10};
    region.clearanceUm = 90.0;
    source.ruleRegions = new RuleRegionSettings[] {region};
    target.applyNewValuesFrom(source);
    assertEquals(1, target.getRuleRegions().length);
    assertEquals(90.0, target.getRuleRegions()[0].clearanceUm, 1e-9);
  }
}
