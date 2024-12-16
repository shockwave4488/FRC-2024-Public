package frc4488.lib.sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IGyro {
  Rotation2d getRoll();

  Rotation2d getPitch();

  Rotation2d getYaw();

  void reset();

  double getYawRateRadiansPerSec();

  void setYawAdjustment(Rotation2d adjustment);
}
