package shockwave.leds.controllers;

import java.util.function.IntSupplier;
import shockwave.leds.canvas.LEDCanvas;

/** Causes repeated gradients to move along an LEDCanvas, like a sawtooth wave */
public class MovingGradientLEDController implements LEDController {

  private final LEDCanvas canvas;
  private final int startColor;
  private final int endColor;
  private final int repeats;
  private final IntSupplier millis;
  private double progress;

  public MovingGradientLEDController(
      LEDCanvas canvas, int startColor, int endColor, int repeats, IntSupplier millis) {
    this.canvas = canvas;
    this.startColor = startColor;
    this.endColor = endColor;
    this.repeats = repeats;
    this.millis = millis;
    this.progress = 0;
  }

  @Override
  public void render() {
    int offset = (int) (canvas.getNumLeds() * progress);
    int gradientLen = canvas.getNumLeds() / repeats;
    for (int i = 0; i < canvas.getNumLeds(); i++) {
      canvas.setRGB(
          i,
          LEDController.interpolateColor(
              startColor, endColor, Math.floorMod(i + offset, gradientLen) / (float) gradientLen));
    }
    progress += LEDController.getProgressInc(millis.getAsInt());
  }
}
