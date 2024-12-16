package shockwave.leds.controllers;

import java.util.function.DoubleUnaryOperator;
import shockwave.leds.canvas.LEDCanvas;

/**
 * Fades the entire LEDCanvas between two colors, defaulting to a linear transition. Use functions
 * like <code>EASE_IN_OUT</code> to change the transition's easing
 */
public class FadeLEDController implements LEDController {

  public static final DoubleUnaryOperator LINEAR = DoubleUnaryOperator.identity();
  public static final DoubleUnaryOperator EASE_IN_OUT =
      progress -> Math.pow(Math.sin(Math.PI / 2 * progress), 2);
  public static final DoubleUnaryOperator EASE_IN_OUT_2 = EASE_IN_OUT.andThen(EASE_IN_OUT);

  private final LEDCanvas canvas;
  private final int startColor;
  private final int endColor;
  private final int millis;
  private final DoubleUnaryOperator easing;
  private double progress;

  public FadeLEDController(
      LEDCanvas canvas, int startColor, int endColor, int millis, DoubleUnaryOperator easing) {
    this.canvas = canvas;
    this.startColor = startColor;
    this.endColor = endColor;
    this.millis = millis;
    this.easing = easing;
    this.progress = 0;
  }

  public FadeLEDController(LEDCanvas canvas, int startColor, int endColor, int millis) {
    this(canvas, startColor, endColor, millis, LINEAR);
  }

  @Override
  public void render() {
    canvas.fillRGB(
        LEDController.interpolateColor(
            startColor, endColor, (float) easing.applyAsDouble(progress)));
    progress += LEDController.getProgressInc(millis);
  }

  @Override
  public boolean isFinished() {
    return progress > 1;
  }
}
