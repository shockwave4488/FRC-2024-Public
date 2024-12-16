package shockwave.leds.controllers;

import java.util.function.DoubleSupplier;
import shockwave.leds.EQ;

public class EQController implements LEDController {

  private final EQ eq;
  private final DoubleSupplier targetlevel;
  private double[] levels;

  public EQController(EQ eq, DoubleSupplier targetLevel) {
    this.eq = eq;
    this.targetlevel = targetLevel;
    this.levels = new double[eq.getWidth()];
  }

  @Override
  public void render() {
    double targetLevel = this.targetlevel.getAsDouble();

    for (int i = 0; i < eq.getWidth(); i++) {
      levels[i] += (Math.random() * 2 - 1 + Math.signum(targetLevel - levels[i]) / 5.0) / 5.0;
      levels[i] = Math.max(0, Math.min(1, levels[i]));
      eq.setLevel(i, levels[i]);
    }
  }
}
