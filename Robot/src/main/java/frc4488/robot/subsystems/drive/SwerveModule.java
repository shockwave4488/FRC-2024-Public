package frc4488.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc4488.lib.drive.SwerveParameters;
import frc4488.lib.drive.SwerveParameters.ModulePosition;
import frc4488.lib.logging.LeveledSmartDashboard;
import frc4488.lib.logging.LogFile;
import frc4488.lib.logging.LogManager;
import frc4488.lib.misc.Timed;
import frc4488.robot.constants.Constants;

public abstract class SwerveModule implements ISwerveModule {
  protected final SwerveParameters parameters;
  protected final Translation2d moduleLocation;
  protected final LogFile log;
  protected final int angleEncoderResolution;
  protected SwerveModuleState desiredState;

  /** m/s */
  protected double desiredModuleSpeed;

  /** Radians */
  protected double desiredModuleAngle;

  public SwerveModule(SwerveParameters parameters, LogManager logger, int angleEncoderResolution) {
    this.parameters = parameters;
    this.moduleLocation = new Translation2d(parameters.moduleX, parameters.moduleY);
    this.log =
        logger
            .getLogFile("SwerveModules/" + parameters.modulePosition.getCompressedName())
            .setDefaultFrequency(3)
            .addTracker("Desired Speed", () -> desiredModuleSpeed)
            .addTracker("Desired Angle", () -> Math.toDegrees(desiredModuleAngle))
            .addTracker("Actual Speed", this::getGroundSpeed)
            .addTracker("Actual Angle", this::getAngleDegreesMod);
    this.angleEncoderResolution = angleEncoderResolution;
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(getGroundSpeed(), Rotation2d.fromRadians(getAngleRadiansMod()));
  }

  @Override
  public Timed<SwerveModulePosition> getPosition() {
    Timed<Double> rotations = getDriveRadians();
    return new Timed<>(
        rotations.time(),
        new SwerveModulePosition(
            rotations.value() * parameters.wheelDiameter / 2,
            Rotation2d.fromRadians(getAngleRadiansMod())));
  }

  /**
   * Sets which state the module should be in.
   *
   * @param desiredState State the module should be in. May change depending on where in the code it
   *     is used.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    this.desiredState = desiredState;

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(getAngleRadiansMod()));

    desiredModuleSpeed = state.speedMetersPerSecond;
    desiredModuleAngle = state.angle.getRadians();
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  protected double metersPerSecToRPM(double metersPerSec) {
    return (metersPerSec * 60 * parameters.driveGearRatio) / (Math.PI * parameters.wheelDiameter);
  }

  @Override
  public double getDesiredAngle() {
    return desiredModuleAngle;
  }

  @Override
  public double getDesiredSpeed() {
    return desiredModuleSpeed;
  }

  public abstract double getDriveRPM();

  @Override
  public double getGroundSpeed() {
    return (getDriveRPM() / (60 * parameters.driveGearRatio)) * Math.PI * parameters.wheelDiameter;
  }

  /**
   * @return The angle of the module's wheel in radians (do not assume this is bounded)
   */
  public abstract double getAngleRadians();

  /**
   * @return The angle of the module within the range (-pi, pi]
   */
  public double getAngleRadiansMod() {
    return MathUtil.angleModulus(getAngleRadians());
  }

  @Override
  public double getAngleDegreesMod() {
    return Math.toDegrees(getAngleRadiansMod());
  }

  /**
   * @return Drive encoder's position in radians (accounting for gear ratio).
   */
  public abstract Timed<Double> getDriveRadians();

  @Override
  public ModulePosition getRobotPosition() {
    return parameters.modulePosition;
  }

  @Override
  public Translation2d getLocation() {
    return moduleLocation;
  }

  @Override
  public void updateSmartDashboard() {
    ModulePosition modulePosition = parameters.modulePosition;
    LeveledSmartDashboard.INFO.putNumber(
        modulePosition + " Drive Desired Speed (m/s)", getDesiredSpeed());
    LeveledSmartDashboard.INFO.putNumber(
        modulePosition + " Drive Actual Speed (deg/s)",
        Math.toDegrees(getDriveRPM() / 60 * Constants.TAU));
    LeveledSmartDashboard.INFO.putNumber(
        modulePosition + " Desired Angle (rad)", getDesiredAngle());
    LeveledSmartDashboard.INFO.putNumber(
        modulePosition + " Drive Actual Speed (m/s)", getGroundSpeed());
    LeveledSmartDashboard.INFO.putNumber(
        modulePosition + " Actual Angle (rad)", getAngleRadiansMod());
  }
}
