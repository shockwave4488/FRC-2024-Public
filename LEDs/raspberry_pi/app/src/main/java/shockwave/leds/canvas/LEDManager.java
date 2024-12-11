package shockwave.leds.canvas;

public class LEDManager {

  private final LEDCanvas leds;
  private final boolean[] taken;

  public LEDManager(LEDCanvas leds) {
    this.leds = leds;
    this.taken = new boolean[leds.getNumLeds()];
  }

  public LEDSection.Builder newSection() {
    return new LEDSection.Builder(this, leds);
  }

  public LEDSection newSectionFromRemaining(boolean mask) {
    boolean[] inverse = new boolean[taken.length];
    for (int ledIndex = 0; ledIndex < taken.length; ledIndex++) {
      inverse[ledIndex] = !taken[ledIndex];
      taken[ledIndex] = true;
    }
    return new LEDSection(leds, inverse, mask);
  }

  void takeLEDs(boolean[] toTake) throws IllegalArgumentException {
    if (taken.length != toTake.length) {
      throw new IllegalArgumentException("toTake.length must match the number of leds");
    }
    for (int ledIndex = 0; ledIndex < taken.length; ledIndex++) {
      if (taken[ledIndex] && toTake[ledIndex]) {
        throw new IllegalArgumentException("Cannot take an already reserved LED: " + ledIndex);
      }
    }
    for (int ledIndex = 0; ledIndex < taken.length; ledIndex++) {
      taken[ledIndex] |= toTake[ledIndex];
    }
  }
}
