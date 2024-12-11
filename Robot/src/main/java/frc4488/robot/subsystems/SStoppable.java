package frc4488.robot.subsystems;

public interface SStoppable {
  /**
   * MUST SET ALL MOTORS TO EITHER COAST OR BRAKE MODE, DEPENDING ON WHAT IS SAFER. The motors
   * cannot be re-enabled until {@link #onSRestart()} is called.
   */
  public void onSStop(boolean robotEnabled);

  /** Re-enables motors (taking into account if the robot is actually enabled) */
  public void onSRestart(boolean robotEnabled);
}
