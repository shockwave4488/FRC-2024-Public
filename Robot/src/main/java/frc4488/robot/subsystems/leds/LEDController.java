package frc4488.robot.subsystems.leds;

import frc4488.lib.logging.LogManager;
import frc4488.lib.wpiextensions.ShockwaveSubsystemBase;
import frc4488.robot.commands.LEDs.SetLEDMode;

public abstract class LEDController extends ShockwaveSubsystemBase {

  private LEDMode mode;

  /**
   * The subclass is responsible for calling {@link #onModeChange(LEDMode)} for the default mode
   * once it is done initializing
   */
  public LEDController(LEDMode defaultMode) {
    mode = defaultMode;
    setDefaultCommand(new SetLEDMode(this, defaultMode));
  }

  /**
   * Sets the mode
   *
   * @param mode The new LED mode
   */
  public void setMode(LEDMode mode) {
    if (mode == null) {
      mode = LEDMode.blank();
    }
    if (this.mode.id() == mode.id() && this.mode.arg() == mode.arg()) {
      return;
    }
    this.mode = mode;
    onModeChange(mode);
  }

  protected abstract void onModeChange(LEDMode mode);

  /**
   * @return The current mode
   */
  public LEDMode getMode() {
    return mode;
  }

  @Override
  public void updateSmartDashboard() {}

  @Override
  public void zeroSensors() {}

  @Override
  public void setUpTrackables(LogManager logger) {}

  @Override
  public void onStart(boolean sStopped) {}

  @Override
  public void onStop(boolean sStopped) {}

  @Override
  public void onSStop(boolean robotEnabled) {}

  @Override
  public void onSRestart(boolean robotEnabled) {}
}
