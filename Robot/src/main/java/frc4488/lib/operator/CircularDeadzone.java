package frc4488.lib.operator;

import edu.wpi.first.math.Pair;
import java.util.function.UnaryOperator;

public class CircularDeadzone implements I2DDeadzoneCalculator {
  private final double DEFAULT_DEADZONE;
  private final UnaryOperator<Double> scale;

  public CircularDeadzone(double DEFAULT_DEADZONE, UnaryOperator<Double> scale) {
    this.DEFAULT_DEADZONE = DEFAULT_DEADZONE;
    this.scale = scale;
  }

  public CircularDeadzone(double DEFAULT_DEADZONE) {
    this(DEFAULT_DEADZONE, value -> value);
  }

  public Pair<Double, Double> deadzone(double xVal, double yVal) {
    return deadzone(xVal, yVal, DEFAULT_DEADZONE);
  }

  public Pair<Double, Double> deadzone(double xVal, double yVal, double deadzone) {
    double distance = Math.sqrt(Math.pow(xVal, 2) + Math.pow(yVal, 2));
    boolean negativeX = xVal < 0;
    boolean negativeY = yVal < 0;
    if (distance < deadzone) {
      xVal = 0;
      yVal = 0;
    } else {
      distance = (distance - deadzone) / (1 - deadzone);
      distance = scale.apply(distance);
      double theta = Math.abs(Math.atan(yVal / xVal));
      yVal = Math.sin(theta) * distance;
      xVal = Math.cos(theta) * distance;
      xVal = negativeX ? xVal * -1 : xVal;
      yVal = negativeY ? yVal * -1 : yVal;
    }
    return Pair.of(xVal, yVal);
  }

  public boolean isPastDeadzone(double xVal, double yVal) {
    double distance = Math.sqrt(Math.pow(xVal, 2) + Math.pow(yVal, 2));
    return (distance > DEFAULT_DEADZONE);
  }
}
