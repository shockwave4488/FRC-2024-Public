package shockwave.leds.canvas;

import java.awt.Color;
import java.util.Optional;
import java.util.stream.IntStream;

public class LEDSection implements LEDCanvas {

  public static class Builder {
    private final LEDManager manager;
    private final LEDCanvas leds;
    private final boolean[] taken;
    private boolean mask;

    Builder(LEDManager manager, LEDCanvas leds) {
      this.manager = manager;
      this.leds = leds;
      this.taken = new boolean[leds.getNumLeds()];
      this.mask = false;
    }

    public LEDSection.Builder takeSpecific(int... leds) {
      for (int ledIndex : leds) {
        taken[ledIndex] = true;
      }
      return this;
    }

    public LEDSection.Builder takeRange(int min, int max) {
      for (int ledIndex = min; ledIndex < max; ledIndex++) {
        taken[ledIndex] = true;
      }
      return this;
    }

    public LEDSection.Builder takeRangeLen(int min, int len) {
      return takeRange(min, min + len);
    }

    public LEDSection.Builder mask() {
      mask = true;
      return this;
    }

    public LEDSection build() throws IllegalArgumentException {
      manager.takeLEDs(taken);
      return new LEDSection(leds, taken, mask);
    }

    public Optional<LEDSection> tryBuild() {
      try {
        return Optional.of(build());
      } catch (IllegalArgumentException e) {
        return Optional.empty();
      }
    }
  }

  private final LEDCanvas leds;
  private final int[] indexConverter;

  LEDSection(LEDCanvas leds, boolean[] taken, boolean mask) {
    this.leds = leds;

    if (mask) {
      this.indexConverter = IntStream.range(0, taken.length).map(i -> taken[i] ? i : -1).toArray();
    } else {
      this.indexConverter = IntStream.range(0, taken.length).filter(i -> taken[i]).toArray();
    }
  }

  @Override
  public int getNumLeds() {
    return indexConverter.length;
  }

  @Override
  public void setRGB(int ledIndex, int r, int g, int b) {
    ledIndex = indexConverter[ledIndex];
    if (ledIndex >= 0) {
      leds.setRGB(ledIndex, r, g, b);
    }
  }

  @Override
  public Color getColor(int ledIndex) {
    return leds.getColor(indexConverter[ledIndex]);
  }
}
