package shockwave.leds.canvas;

import java.awt.Color;

/** Use to prevent strange out-of-bound index errors from unsafe animations */
public class CheckedLEDCanvas implements LEDCanvas {

  private final LEDCanvas canvas;

  public CheckedLEDCanvas(LEDCanvas canvas) {
    this.canvas = canvas;
  }

  @Override
  public int getNumLeds() {
    return canvas.getNumLeds();
  }

  @Override
  public void setRGB(int ledIndex, int r, int g, int b) {
    if (ledIndex < 0 || ledIndex >= canvas.getNumLeds()) {
      return;
    }
    canvas.setRGB(ledIndex, r, g, b);
  }

  @Override
  public Color getColor(int ledIndex) {
    if (ledIndex < 0 || ledIndex >= canvas.getNumLeds()) {
      return Color.BLACK;
    }
    return canvas.getColor(ledIndex);
  }
}
