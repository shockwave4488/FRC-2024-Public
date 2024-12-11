package shockwave.leds.controllers;

import java.awt.Color;

public interface LEDController {
  public static int interpolateColor(int start, int end, float amount) {
    if (amount <= 0) {
      return start;
    }
    if (amount >= 1) {
      return end;
    }
    float[] startHSV =
        Color.RGBtoHSB((start >> 16) & 0xFF, (start >> 8) & 0xFF, start & 0xFF, new float[3]);
    float[] endHSV =
        Color.RGBtoHSB((end >> 16) & 0xFF, (end >> 8) & 0xFF, end & 0xFF, new float[3]);
    return Color.HSBtoRGB(
        startHSV[0] * (1 - amount) + endHSV[0] * amount,
        startHSV[1] * (1 - amount) + endHSV[1] * amount,
        startHSV[2] * (1 - amount) + endHSV[2] * amount);
  }

  /**
   * Gets the amount a progress variable in the range 0.0-1.0 should be incremented every frame,
   * based on 60fps
   *
   * @param totalMillis The amount of time it should take for a progress variable to go from 0.0 to
   *     1.0
   */
  public static double getProgressInc(int totalMillis) {
    return 1 / 60.0f / (totalMillis / 1000.0f);
  }

  public void render();

  public default boolean isFinished() {
    return false;
  }
}
