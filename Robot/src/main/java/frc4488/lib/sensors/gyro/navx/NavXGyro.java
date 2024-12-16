package frc4488.lib.sensors.gyro.navx;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import frc4488.lib.sensors.gyro.IGyro;

public class NavXGyro implements IGyro {
  private final NavX navX;

  public NavXGyro(SPI.Port spi_port_id) {
    navX = new NavX(spi_port_id);
  }

  @Override
  public Rotation2d getRoll() {
    return navX.getRoll();
  }

  @Override
  public Rotation2d getPitch() {
    return navX.getPitch();
  }

  @Override
  public Rotation2d getYaw() {
    return navX.getYaw();
  }

  @Override
  public void reset() {
    navX.reset();
  }

  @Override
  public double getYawRateRadiansPerSec() {
    return navX.getYawRateRadiansPerSec();
  }

  @Override
  public void setYawAdjustment(Rotation2d adjustment) {
    navX.setYawAdjustment(adjustment);
  }
}
