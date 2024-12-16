package frc4488.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc4488.lib.logging.LogManager;
import frc4488.lib.wpiextensions.ShockwaveSubsystemBase;

public class SmartPCM extends ShockwaveSubsystemBase {

  private final Compressor compressor;

  public SmartPCM(int PCM_ID) {
    compressor = new Compressor(PCM_ID, PneumaticsModuleType.CTREPCM);
  }

  public void startCompressor() {
    if (isSStopped()) {
      return;
    }
    compressor.enableDigital();
  }

  public void stopCompressor() {
    compressor.disable();
  }

  @Override
  public void updateSmartDashboard() {}

  @Override
  public void zeroSensors() {}

  @Override
  public void setUpTrackables(LogManager logger) {}

  @Override
  public void onStart(boolean sStopped) {
    if (sStopped) {
      return;
    }
    startCompressor();
  }

  @Override
  public void onStop(boolean sStopped) {
    stopCompressor();
  }

  @Override
  public void onSStop(boolean robotEnabled) {
    compressor.disable();
  }

  @Override
  public void onSRestart(boolean robotEnabled) {
    if (robotEnabled) {
      compressor.enableDigital();
    }
  }
}
