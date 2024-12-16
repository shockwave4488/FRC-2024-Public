package frc4488.lib.sensors.gyro.navx;

interface IBoardCapabilities {
  public boolean isOmniMountSupported();

  public boolean isBoardYawResetSupported();

  public boolean isDisplacementSupported();

  public boolean isAHRSPosTimestampSupported();
}
