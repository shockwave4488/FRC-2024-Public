package shockwave.leds.canvas;

import java.awt.Color;

public class LEDCloner implements LEDCanvas {

  private final int numLeds;
  private final LEDCanvas[] clones;

  public LEDCloner(LEDCanvas... clones) {
    if (clones.length == 0) {
      throw new IllegalArgumentException("There must be at least one LEDCanvas!");
    }
    this.numLeds = clones[0].getNumLeds();
    for (int i = 1; i < clones.length; i++) {
      if (clones[i].getNumLeds() != numLeds) {
        throw new IllegalArgumentException("All of the clones must have the same number of leds!");
      }
    }

    this.clones = clones;
  }

  @Override
  public int getNumLeds() {
    return numLeds;
  }

  @Override
  public void setRGB(int i, int r, int g, int b) {
    for (LEDCanvas clone : clones) {
      clone.setRGB(i, r, g, b);
    }
  }

  @Override
  public Color getColor(int i) {
    return clones[0].getColor(i);
  }
}
