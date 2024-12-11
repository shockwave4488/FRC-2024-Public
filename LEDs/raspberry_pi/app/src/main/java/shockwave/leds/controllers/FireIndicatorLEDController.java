package shockwave.leds.controllers;

import java.util.function.Consumer;
import shockwave.leds.canvas.LEDCanvas;

/**
 * Causes an LEDCanvas to blink when randomly triggering other FireLEDControllers. Intended for the
 * indicator triangles on Vortex, but these didn't end up getting added. However, the ability to
 * trigger multiple FireLEDControllers in sync is still used
 */
public class FireIndicatorLEDController implements LEDController {

  private static final int PULSE_LENGTH = 40;

  private final LEDCanvas canvas;
  private final Consumer<Boolean> startFire;
  private int fireTicks;

  public FireIndicatorLEDController(LEDCanvas canvas, Consumer<Boolean> startFire) {
    this.canvas = canvas;
    this.startFire = startFire;
    this.fireTicks = 0;
  }

  @Override
  public void render() {
    boolean startFire = (Math.random() < 0.0025);
    this.startFire.accept(startFire);

    if (startFire) {
      fireTicks = PULSE_LENGTH;
    }

    if (fireTicks >= 0) {
      canvas.fillRGB(fireTicks * 255 / PULSE_LENGTH, 0, 0);
      fireTicks--;
    }
  }
}
