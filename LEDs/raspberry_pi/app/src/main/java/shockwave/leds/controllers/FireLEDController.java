package shockwave.leds.controllers;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;
import java.util.Random;
import java.util.function.Supplier;
import shockwave.leds.Colors;
import shockwave.leds.canvas.LEDCanvas;

/**
 * Causes sections of "fire" to move along a LEDCanvas, where each section is mostly based on the
 * passed <code>seed</code>, but has some slight variations. Within the sections, the pixels are
 * somewhat clumped
 */
public class FireLEDController implements LEDController {

  private final LEDCanvas canvas;
  private final Supplier<Boolean> startFire;
  private final Random rand;
  private final List<Integer> pulses;

  public FireLEDController(LEDCanvas canvas, Supplier<Boolean> startFire, long seed) {
    this.canvas = canvas;
    this.startFire = startFire;
    this.rand = new Random(seed);
    this.pulses = new ArrayList<>();
  }

  @Override
  public void render() {
    canvas.fillRGB(0);
    for (ListIterator<Integer> iterator = pulses.listIterator(); iterator.hasNext(); ) {
      int pulse = iterator.next();
      if (pulse >= canvas.getNumLeds()) {
        iterator.remove();
      } else {
        if (pulse >= 0) {
          canvas.setRGB(pulse, Colors.RED);
        }
        iterator.set(pulse + 1);
      }
    }
    if (startFire.get()) {
      pulses.add(0);
      boolean prevPulse = false;
      for (int i = -20; i < 0; i++) {
        prevPulse = ((rand.nextDouble() < (prevPulse ? 0.75 : 0.5)) != (Math.random() < 0.25));
        if (prevPulse) {
          pulses.add(i);
        }
      }
    }
  }
}
