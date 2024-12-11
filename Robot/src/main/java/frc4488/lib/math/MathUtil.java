package frc4488.lib.math;

import frc4488.robot.constants.Constants;
import java.util.List;

public class MathUtil {

  public static double calcCircularDelta(double value, double ref) {
    double delta = ref - edu.wpi.first.math.MathUtil.angleModulus(value);
    if (delta > Math.PI) {
      delta = Constants.TAU - delta;
    } else if (delta < -Math.PI) {
      delta += Constants.TAU;
    }
    return delta;
  }

  /** <code>avg</code> must equal the average of the supplied <code>values</code> */
  public static double calcVariance(double avg, List<Double> values, boolean circular) {
    if (values.isEmpty()) {
      return 0;
    }
    if (circular) {
      avg = edu.wpi.first.math.MathUtil.angleModulus(avg);
    }
    double sum = 0;
    for (double value : values) {
      double delta;
      if (circular) {
        delta = calcCircularDelta(value, avg);
      } else {
        delta = avg - value;
      }
      sum += delta * delta;
    }
    return sum / values.size();
  }

  /** <code>avg</code> must equal the average of the supplied <code>values</code> */
  public static double calcVariance(double avg, List<Double> values) {
    return calcVariance(avg, values, false);
  }
}
