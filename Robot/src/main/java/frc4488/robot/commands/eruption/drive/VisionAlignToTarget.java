package frc4488.robot.commands.eruption.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc4488.lib.sensors.gyro.IGyro;
import frc4488.lib.sensors.vision.VisionCameras.TargetCamera;
import frc4488.robot.constants.Constants.DriveTrainConstants;
import frc4488.robot.constants.Constants2022.FieldConstants;
import frc4488.robot.subsystems.drive.SwerveDrive;
import java.util.function.BooleanSupplier;

/**
 * This command will take control of the swerve drive's rotation to make the swerve drive face a
 * vision target. It will only end itself once it is facing the target. So, this command should be
 * ran in parallel with another command that finishes if the limelight loses the vision target.
 */
public class VisionAlignToTarget extends Command {
  private final SwerveDrive swerve;
  private final TargetCamera limelight;
  private final IGyro gyro;
  private final double rotationMultiplier;
  private final BooleanSupplier notMoving;
  private boolean hasResetPose;
  private static final double DEFAULT_DONE_TOLERANCE = 2.5;
  private double currentTolerance = DEFAULT_DONE_TOLERANCE;
  private Rotation2d limelightXAngle;
  private Rotation2d limelightYAngle;
  private double translationalDist;
  public static final int MIN_AT_SETPOINT_CYCLES = 10;
  /* prevents setpoint from being updated if the change in limelight.getX() is less than this value. */
  private static final double ANTI_OSCILLATION_THRESHOLD = 1; // degrees
  private static final double DEFAULT_PID_P = DriveTrainConstants.SWERVE_DRIVE_ROTATION_P;
  private static final double DEFAULT_PID_I = DriveTrainConstants.SWERVE_DRIVE_ROTATION_I;
  private static final double DEFAULT_PID_D = DriveTrainConstants.SWERVE_DRIVE_ROTATION_D;
  private PIDController pidController =
      new PIDController(DEFAULT_PID_P, DEFAULT_PID_I, DEFAULT_PID_D);

  public boolean pidAtSetpoint() {
    return pidController.atSetpoint();
  }

  /**
   * A command that rotates the robot to face a vision target. Thank you to 2910's
   * VisionRotateToTargetCommand class (found here
   * https://github.com/FRCTeam2910/2021CompetitionRobot/blob/master/src/main/java/org/frcteam2910/c2020/commands/VisionRotateToTargetCommand.java)
   * for providing an initial inspiration for this command.
   *
   * @param swerve
   * @param limelight
   * @param gyro
   */
  public VisionAlignToTarget(
      SwerveDrive swerve,
      TargetCamera limelight,
      IGyro gyro,
      double rotationMultiplier,
      BooleanSupplier notMoving) {
    this.swerve = swerve;
    this.limelight = limelight;
    this.gyro = gyro;
    this.rotationMultiplier = rotationMultiplier;
    this.notMoving = notMoving;

    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(DEFAULT_DONE_TOLERANCE);

    // Shouldn't require the limelight here because no changes are applied to it.
    addRequirements(swerve.rotationRequirement);
  }

  @Override
  public void initialize() {
    pidController.reset();
    /* limelightXAngle & limelightYAngle are set to illogical values to guarantee that
    hasSignificantChange returns true and that these values are updated upon the first cycle that
    the limelight sees the target */
    limelightXAngle = new Rotation2d(Math.PI);
    limelightYAngle = new Rotation2d(Math.PI);
    currentTolerance = DEFAULT_DONE_TOLERANCE;
  }

  @Override
  public void execute() {
    double rotPower = 0;
    boolean hasTarget = limelight.hasTargets();

    if (hasTarget) {
      Rotation2d actualLimelightX = limelight.getBestTarget().get().getX();
      Rotation2d actualLimelightY = limelight.getBestTarget().get().getY();
      if (hasSignificantChange(actualLimelightX, limelightXAngle)) {
        limelightXAngle = actualLimelightX;
        // Returns a positive value when the target is on the left side of the screen
      }
      if (hasSignificantChange(actualLimelightY, limelightYAngle)) {
        limelightYAngle = actualLimelightY;
        translationalDist =
            limelight.getEstimatedDistance(
                    limelight.getBestTarget().get(), FieldConstants.TARGET_HEIGHT_METERS)
                + FieldConstants.HUB_RADIUS_METERS;
        currentTolerance =
            Math.toDegrees(
                Math.abs(
                    Math.atan(FieldConstants.HUB_SAFE_SHOT_RADIUS_METERS / translationalDist)));
        hasResetPose = false;
      }
      Rotation2d currentAngle = gyro.getYaw();
      // SmartDashboard.putNumber("LL X Difference", limelightXAngle);
      double targetAngle = currentAngle.plus(limelightXAngle).getDegrees();
      if (targetAngle > 180) {
        targetAngle -= 360;
      } else if (targetAngle < -180) {
        targetAngle += 360;
      }
      // SmartDashboard.putNumber("LL Target Angle", targetAngle);
      rotPower = pidController.calculate(currentAngle.getDegrees(), targetAngle);
      rotPower *= rotationMultiplier;
      rotPower = Math.min(Math.max(-rotationMultiplier, rotPower), rotationMultiplier);
    }

    // tolerance calculated based on distance so we can have a prediction if we'll make our shot
    // based on our pidController
    pidController.setTolerance(currentTolerance);

    // Condition under which it should be safe to reset the position of the swerve drive based on
    // vision and gyro.
    if (!hasResetPose && hasTarget && notMoving.getAsBoolean()) {
      Pose2d robotPose =
          limelight.getRobotPoseFromDistance(
              limelight.getBestTarget().get(),
              FieldConstants.HUB_CENTER,
              translationalDist,
              gyro.getYaw());
      swerve.consumeVisionEstimate(robotPose, Timer.getFPGATimestamp());
      hasResetPose = true;
    }

    swerve.setRotationSpeed(rotPower);
  }

  private boolean hasSignificantChange(
      Rotation2d currentLimelightValue, Rotation2d storedLimelightValue) {
    return Math.abs((currentLimelightValue.getDegrees() - storedLimelightValue.getDegrees()))
        > ANTI_OSCILLATION_THRESHOLD;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }
}
