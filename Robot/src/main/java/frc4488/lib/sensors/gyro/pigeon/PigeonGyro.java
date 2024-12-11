package frc4488.lib.sensors.gyro.pigeon;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc4488.lib.sensors.gyro.IGyro;

public class PigeonGyro implements IGyro {
  private final Pigeon2 pigeon;

  public PigeonGyro(int deviceId, String canbus) {
    pigeon = new Pigeon2(deviceId, canbus);
    var config = new Pigeon2Configuration();
    pigeon.getConfigurator().apply(config);
  }

  @Override
  public Rotation2d getRoll() {
    return new Rotation2d(pigeon.getRotation3d().getX());
  }

  @Override
  public Rotation2d getPitch() {
    return new Rotation2d(pigeon.getRotation3d().getY());
  }

  @Override
  public Rotation2d getYaw() {
    return pigeon.getRotation2d();
  }

  @Override
  public void reset() {
    pigeon.reset();
  }

  @Override
  public double getYawRateRadiansPerSec() {
    return 0.0;
  }

  @Override
  public void setYawAdjustment(Rotation2d adjustment) {}
}
