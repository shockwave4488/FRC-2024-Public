package frc4488.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc4488.lib.drive.SwerveParameters;
import frc4488.lib.misc.Timed;
import frc4488.robot.subsystems.SStoppable;

/** This class creates a template for which all swerve code should follow. */
public interface ISwerveModule extends SStoppable {

  /** Gets the state the module is in. */
  SwerveModuleState getState();

  /** Gets the position of the module. */
  Timed<SwerveModulePosition> getPosition();

  /**
   * Sets which state the module should be in.
   *
   * @param desiredState State the module should be in. May change depending on where in the code it
   *     is used.
   */
  void setDesiredState(SwerveModuleState desiredState);

  SwerveModuleState getDesiredState();

  /** Gets which angle the module should be facing */
  double getDesiredAngle();

  /** Gets which speed the module should be going in meters/sec */
  double getDesiredSpeed();

  /**
   * @return The speed of the module's wheel in meters/sec
   */
  double getGroundSpeed();

  /** Returns the module's position on the robot */
  public SwerveParameters.ModulePosition getRobotPosition();

  /** Returns the Translation2d location of the module */
  public Translation2d getLocation();

  /**
   * @return The angle of the module in degrees within the range (-180, 180]
   */
  double getAngleDegreesMod();

  /**
   * Updates values on SmartDashboard. If these aren't showing up, double check that the call to
   * this method in SwerveDrive.java is uncommented.
   */
  default void updateSmartDashboard() {}

  /** Call this method to run code upon enabling the robot */
  default void onStart(boolean sStopped) {}

  /** Call this method to run code upon disabling the robot */
  default void onStop(boolean sStopped) {}

  public void zeroSensors();

  public boolean isEncoderConnected();
}
