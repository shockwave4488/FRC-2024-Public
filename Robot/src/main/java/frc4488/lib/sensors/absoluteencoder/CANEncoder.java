package frc4488.lib.sensors.absoluteencoder;

import com.ctre.phoenix6.hardware.CANcoder;
import frc4488.robot.constants.Constants;

public class CANEncoder implements AbsoluteEncoder {
  private double offset;
  private CANcoder cancoder;

  public CANEncoder(int deviceId, String canbus, double offset) {
    this.offset = offset;
    cancoder = new CANcoder(deviceId, canbus);
  }

  @Override
  public double getAngle() {
    return cancoder.getAbsolutePosition().getValueAsDouble() * Constants.TAU;
  }

  @Override
  public double getOffset() {
    return this.offset;
  }

  @Override
  public void setOffset(double offset) {
    this.offset = offset;
  }
}
