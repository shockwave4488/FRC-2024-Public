package shockwave.leds;

import shockwave.leds.canvas.LEDCanvas;
import shockwave.leds.controllers.LEDController;

public class EQ {

  private final int width;
  private final int height;
  private final LEDCanvas canvas;
  private int startColor;
  private int endColor;

  public EQ(int width, int height, LEDCanvas canvas) {
    if (canvas.getNumLeds() != width * height) {
      throw new IllegalArgumentException("The canvas must match the width and height!");
    }

    this.width = width;
    this.height = height;
    this.canvas = canvas;
    this.startColor = Colors.GREEN;
    this.endColor = Colors.RED;
  }

  /**
   * The parts in the middle of the EQ are interpolated
   *
   * @param start The color used on the lower parts of the EQ
   * @param end The color used on the upper parts of the EQ
   * @return this
   */
  public EQ setColorRange(int start, int end) {
    startColor = start;
    endColor = end;
    return this;
  }

  public int getWidth() {
    return width;
  }

  public void setLevel(int column, double level) {
    level = Math.max(0, Math.min(1, level));

    int offset = column * height;
    int endI = (int) Math.round((height - 1) * level);
    for (int i = 0; i <= endI; i++) {
      canvas.setRGB(
          i + offset,
          LEDController.interpolateColor(startColor, endColor, (float) i / (height - 1)));
    }
    for (int i = endI + 1; i < height; i++) {
      canvas.setRGB(i + offset, 0);
    }
  }

  public void setGlobalLevel(double level) {
    for (int i = 0; i < width; i++) {
      setLevel(i, level);
    }
  }
}
