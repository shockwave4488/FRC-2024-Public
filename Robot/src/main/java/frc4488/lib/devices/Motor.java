package frc4488.lib.devices;

import edu.wpi.first.wpilibj.RobotState;
import frc4488.robot.subsystems.SStoppable;

/**
 * Wraps around motors to make sure that s-stop is followed. Subsystems should setup the actual
 * motor in the constructor, and then only save the Motor wrapper rather than the actual motor in a
 * field. <br>
 * <br>
 * <strong>WARNING: CALL ALL FOUR STATE METHODS:</strong> <code>onStart</code>, <code>onStop</code>,
 * <code>onSStop</code>, <code>onSRestart</code>
 */
public abstract class Motor<T> implements SStoppable {

  private final boolean disabledBrake;
  private final boolean enabledBrake;
  private final boolean sStoppedBrake;
  private Boolean currentNeutralMode;
  private boolean coastMode;
  private boolean sStopped;

  public Motor(boolean disabledBrake, boolean enabledBrake, boolean sStoppedBrake) {
    this.disabledBrake = disabledBrake;
    this.enabledBrake = enabledBrake;
    this.sStoppedBrake = sStoppedBrake;
  }

  /**
   * <strong>WARNING: MAKE SURE TO CHECK {@link #isSStopped()} BEFORE DOING STUFF</strong> <br>
   * <br>
   * Deprecation to make sure the caller understands that they must be careful
   */
  @Deprecated
  public abstract T getInternalMotor();

  public void setPercent(double percent) {
    if (!sStopped) {
      setPercentInternal(percent);
    }
  }

  /**
   * This will get reset to <code>disabledBrake</code>, <code>enabledBrake</code>, or <code>
   * sStoppedBrake</code> when any of those events occur
   *
   * @see #applyNeutralMode()
   * @see #applyNeutralMode(boolean)
   */
  public void setNeutralMode(boolean brake) {
    if (!sStopped) {
      if (RobotState.isDisabled() && coastMode) {
        brake = false;
      }

      // Setting the neutral mode is slow
      if (currentNeutralMode != null && currentNeutralMode == brake) {
        return;
      }
      currentNeutralMode = brake;
      setNeutralModeInternal(brake);
    }
  }

  /**
   * @see #setNeutralMode(boolean)
   * @see #applyNeutralMode(boolean)
   */
  public void applyNeutralMode() {
    if (!sStopped) {
      applyNeutralModeInternal();
    }
  }

  /**
   * This will get reset to <code>disabledBrake</code>, <code>enabledBrake</code>, or <code>
   * sStoppedBrake</code> when any of those events occur
   *
   * @see #setNeutralMode(boolean)
   * @see #applyNeutralMode()
   */
  public void applyNeutralMode(boolean brake) {
    setNeutralMode(brake);
    applyNeutralMode();
  }

  public void onStart() {
    setNeutralMode(enabledBrake);
  }

  public void onStop() {
    setNeutralMode(!coastMode && disabledBrake);
  }

  /**
   * This is for enabling coast mode via a button while the robot is disabled. Use {@link
   * #applyNeutralMode(boolean)} to set coast/brake directly.
   */
  public void setCoastModeButtonState(boolean coastMode) {
    this.coastMode = coastMode;
    if (RobotState.isDisabled()) {
      setNeutralMode(!coastMode && disabledBrake);
    }
  }

  @Override
  public void onSStop(boolean robotEnabled) {
    if (sStopped) {
      return;
    }
    sStopped = true;
    currentNeutralMode = sStoppedBrake;
    setNeutralModeInternal(sStoppedBrake);
    applyNeutralModeInternal();
  }

  @Override
  public void onSRestart(boolean robotEnabled) {
    if (!sStopped) {
      return;
    }
    sStopped = false;
    setNeutralMode(robotEnabled ? enabledBrake : (!coastMode && disabledBrake));
  }

  public boolean isSStopped() {
    return sStopped;
  }

  protected abstract void setPercentInternal(double percent);

  protected abstract void setNeutralModeInternal(boolean brake);

  protected abstract void applyNeutralModeInternal();

  public abstract void setPosition(double position);

  /** Unit: Rotations */
  public abstract double getPosition();

  /** Note: Will always return 0 when s-stopped */
  public abstract double getPercent();

  /**
   * @return The current going into the motor from the battery
   */
  public abstract double getSupplyCurrent();
}
