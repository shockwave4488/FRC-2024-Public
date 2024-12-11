package frc4488.lib.sensors.absoluteencoder;

import frc4488.robot.constants.Constants;

public interface AbsoluteEncoder {
  /**
   * Gets the angle in radians without an offset
   *
   * @return Angle in radians
   */
  public double getAngle();

  /**
   * Gets the angle of the encoder in radians with the offset
   *
   * @return Adjusted angle in radians
   */
  public default double getAngleOffset() {
    return (getAngle() - getOffset() + Constants.TAU) % Constants.TAU;
  }

  public double getOffset();

  public void setOffset(double offset);
}
