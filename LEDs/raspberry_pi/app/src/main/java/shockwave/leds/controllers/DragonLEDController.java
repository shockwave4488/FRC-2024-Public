package shockwave.leds.controllers;

import java.util.ArrayList;
import java.util.List;
import shockwave.leds.canvas.LEDCanvas;

public class DragonLEDController implements LEDController {

  public static final int MAIN_NORMAL = 0xAA8800;
  public static final int MAIN_ANGRY = 0x880000;
  public static final int GLINT_NORMAL = 0xFFFF00;
  public static final int GLINT_ANGRY = 0xFF0000;
  private static final float PULSE_RADIUS = 2.0f;

  private final LEDCanvas canvas;
  private final List<Double> pulses;
  private int mainColor;
  private int glintColor;

  public DragonLEDController(LEDCanvas canvas, boolean angry) {
    this.canvas = canvas;
    this.pulses = new ArrayList<>();
    setAngry(angry);
  }

  public void setAngry(boolean angry) {
    mainColor = angry ? MAIN_ANGRY : MAIN_NORMAL;
    glintColor = angry ? GLINT_ANGRY : GLINT_NORMAL;
  }

  @Override
  public void render() {
    for (int i = 0; i < canvas.getNumLeds(); i++) {
      double dist = getDistToNearestPulse(i);
      canvas.setRGB(
          i,
          LEDController.interpolateColor(mainColor, glintColor, 1 - (float) dist / PULSE_RADIUS));
    }
    pulses.replaceAll(pulse -> pulse - 0.25);
    pulses.removeIf(pulse -> pulse < -PULSE_RADIUS);
    if (Math.random() < 0.01) {
      pulses.add((double) canvas.getNumLeds() + PULSE_RADIUS);
    }
  }

  private double getDistToNearestPulse(int i) {
    double minDist = Integer.MAX_VALUE;
    for (double pulse : pulses) {
      double dist = Math.abs(i - pulse);
      if (dist < minDist) {
        minDist = dist;
      }
    }
    return minDist;
  }
}
