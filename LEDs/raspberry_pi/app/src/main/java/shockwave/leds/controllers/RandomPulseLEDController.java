package shockwave.leds.controllers;

import java.util.ArrayList;
import java.util.List;
import shockwave.leds.canvas.LEDCanvas;

/** Causes randomly spaced dots of color to move along an LEDCanvas */
public class RandomPulseLEDController implements LEDController {

  private static class Pulse {
    public int position;
    public int velocity;

    public Pulse(int position, int velocity) {
      this.position = position;
      this.velocity = velocity;
    }
  }

  private final LEDCanvas canvas;
  private final int color;
  private final List<Pulse> pulses;

  public RandomPulseLEDController(LEDCanvas canvas, int color) {
    this.canvas = canvas;
    this.color = color;
    this.pulses = new ArrayList<>();
  }

  @Override
  public void render() {
    canvas.fillRGB(0);
    pulses.removeIf(pulse -> pulse.position < 0 || pulse.position >= canvas.getNumLeds());
    for (Pulse pulse : pulses) {
      canvas.setRGB(pulse.position, color);
      pulse.position += pulse.velocity;
    }
    if (Math.random() < 0.01) {
      int position = (Math.random() < 0.5 ? 0 : canvas.getNumLeds() - 1);
      int velocity = (position == 0 ? 1 : -1);
      pulses.add(new Pulse(position, velocity));
    }
  }
}
