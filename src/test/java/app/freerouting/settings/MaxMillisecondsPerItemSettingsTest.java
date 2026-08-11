package app.freerouting.settings;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.util.gson.GsonProvider;
import org.junit.jupiter.api.Test;

/**
 * JSON plumbing and helper semantics for the router.max_milliseconds_per_item setting
 * (per-connection time budget for the batch autoroute stage).
 */
class MaxMillisecondsPerItemSettingsTest {

  @Test
  void gsonDeserializesTheFieldFromTheRouterSection() {
    // Same deserialization path as JsonFileSettings uses for the "router" section
    // of the global freerouting.json.
    RouterSettings settings = GsonProvider.GSON.fromJson(
        "{\"max_milliseconds_per_item\": 200}", RouterSettings.class);
    assertEquals(200, settings.getMaxMillisecondsPerItem());
  }

  @Test
  void gsonSerializesTheFieldUnderItsSerializedName() {
    RouterSettings settings = new RouterSettings();
    settings.maxMillisecondsPerItem = 350;
    String json = GsonProvider.GSON.toJson(settings);
    assertTrue(json.contains("\"max_milliseconds_per_item\": 350"),
        "expected max_milliseconds_per_item in: " + json);
  }

  @Test
  void defaultIsUnlimited() {
    assertEquals(0, new RouterSettings().getMaxMillisecondsPerItem());
  }

  @Test
  void zeroAndNegativeMeanUnlimited() {
    RouterSettings settings = new RouterSettings();
    settings.maxMillisecondsPerItem = 0;
    assertEquals(0, settings.getMaxMillisecondsPerItem());
    settings.maxMillisecondsPerItem = -5;
    assertEquals(0, settings.getMaxMillisecondsPerItem());
  }

  @Test
  void cloneCarriesTheField() {
    RouterSettings settings = new RouterSettings();
    settings.maxMillisecondsPerItem = 200;
    assertEquals(200, settings.clone().getMaxMillisecondsPerItem());
  }

  @Test
  void mergeCopiesTheFieldFromAHigherPrioritySource() {
    RouterSettings target = new RouterSettings();
    RouterSettings source = new RouterSettings();
    source.maxMillisecondsPerItem = 750;
    target.applyNewValuesFrom(source);
    assertEquals(750, target.getMaxMillisecondsPerItem());
  }
}
