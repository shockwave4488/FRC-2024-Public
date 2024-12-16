package shockwave.leds.canvas;

import java.awt.Color;

public class LEDReverser implements LEDCanvas {

  private final LEDCanvas canvas;

  public LEDReverser(LEDCanvas canvas) {
    this.canvas = canvas;
  }

  @Override
  public int getNumLeds() {
    return canvas.getNumLeds();
  }

  @Override
  public void setRGB(int i, int r, int g, int b) {
    canvas.setRGB(canvas.getNumLeds() - 1 - i, r, g, b);
  }

  @Override
  public Color getColor(int i) {
    return canvas.getColor(canvas.getNumLeds() - 1 - i);
  }
}
