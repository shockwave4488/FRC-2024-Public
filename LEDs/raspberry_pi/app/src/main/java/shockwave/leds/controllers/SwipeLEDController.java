package shockwave.leds.controllers;

import shockwave.leds.canvas.LEDCanvas;

public class SwipeLEDController implements LEDController {

  private final LEDCanvas canvas;
  private final int color;
  private final int millis;
  private double progress;
  private boolean finished;

  public SwipeLEDController(LEDCanvas canvas, int color, int millis) {
    this.canvas = canvas;
    this.color = color;
    this.millis = millis;
    this.progress = 0;
    this.finished = false;
  }

  @Override
  public void render() {
    canvas.rangeRGB(
        0, Math.min(canvas.getNumLeds(), (int) (canvas.getNumLeds() * progress)), color);
    if (progress >= 1) {
      finished = true;
    } else {
      progress += LEDController.getProgressInc(millis);
    }
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
