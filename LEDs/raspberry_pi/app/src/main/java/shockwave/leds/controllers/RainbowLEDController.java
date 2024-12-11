package shockwave.leds.controllers;

import shockwave.leds.canvas.LEDCanvas;

public class RainbowLEDController implements LEDController {

  private final LEDCanvas canvas;
  private final int millis;
  private double progress;

  public RainbowLEDController(LEDCanvas canvas, int millis) {
    this.canvas = canvas;
    this.millis = millis;
  }

  @Override
  public void render() {
    for (int i = 0; i < canvas.getNumLeds(); i++) {
      canvas.setHSV(i, (float) i / canvas.getNumLeds() + (float) progress, 1, 1);
    }
    progress += LEDController.getProgressInc(millis);
  }
}
