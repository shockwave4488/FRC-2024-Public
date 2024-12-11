package shockwave.leds.canvas;

import java.awt.Color;

public class LEDFill implements LEDCanvas {

  private final LEDCanvas canvas;

  public LEDFill(LEDCanvas canvas) {
    this.canvas = canvas;
  }

  @Override
  public int getNumLeds() {
    return canvas.getNumLeds() == 0 ? 0 : 1;
  }

  @Override
  public void setRGB(int i, int r, int g, int b) {
    canvas.fillRGB(r, g, b);
  }

  @Override
  public Color getColor(int i) {
    return canvas.getColor(0);
  }
}
