package frc4488.lib.sensors;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc4488.lib.sensors.absoluteencoder.AbsoluteEncoder;
import frc4488.robot.constants.Constants;

public class AbsoluteEncoderWraparound implements AbsoluteEncoder {

  private final AbsoluteEncoder encoder;
  private int wrapCount;
  private double lastMeasurement;

  public AbsoluteEncoderWraparound(AbsoluteEncoder encoder) {
    this.encoder = encoder;

    CommandScheduler.getInstance().getDefaultButtonLoop().bind(this::periodic);
  }

  @Override
  public double getAngle() {
    return encoder.getAngle() + wrapCount * Constants.TAU;
  }

  @Override
  public double getOffset() {
    return encoder.getOffset();
  }

  @Override
  public void setOffset(double offset) {
    encoder.setOffset(offset);
  }

  private void periodic() {
    double currentAngle = encoder.getAngle();
    double angleDiff = currentAngle - lastMeasurement;
    if (Math.abs(angleDiff) > Math.PI) {
      wrapCount -= Math.abs(angleDiff) / angleDiff;
    }
    lastMeasurement = currentAngle;
  }
}
