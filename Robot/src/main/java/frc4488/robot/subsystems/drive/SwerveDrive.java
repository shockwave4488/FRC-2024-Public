package frc4488.robot.subsystems.drive;

import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4488.lib.drive.SwerveParameters;
import frc4488.lib.drive.SwerveParameters.ModulePosition;
import frc4488.lib.logging.LeveledSmartDashboard;
import frc4488.lib.logging.LogFile;
import frc4488.lib.logging.LogLevel;
import frc4488.lib.logging.LogManager;
import frc4488.lib.math.PoseEstimator;
import frc4488.lib.misc.Timed;
import frc4488.lib.misc.Util;
import frc4488.lib.sensors.gyro.IGyro;
import frc4488.lib.wpiextensions.ShockwaveSubsystemBase;
import frc4488.robot.commands.drive.SwerveModifierCommand.SwerveModifier;
import frc4488.robot.constants.Constants.DriveTrainConstants;
import frc4488.robot.constants.Constants.VisionConstants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrive extends ShockwaveSubsystemBase {
  private final Translation2d[] moduleLocations;
  private final ISwerveModule[] swerveModules;
  private final Map<ModulePosition, ISwerveModule> swerveModuleMap;
  private final Map<ModulePosition, Boolean> sStoppedModules;

  private final IGyro gyro;
  private final Rotation2d gyroAdjustment;
  private final SwerveDriveKinematics kinematics;
  private final PoseEstimator poseEstimator;

  public final SubsystemBase driveRequirement = childRequirement();
  public final SubsystemBase rotationRequirement = childRequirement();
  public final SubsystemBase modifierRequirement = childRequirement();

  private LogFile log;

  private boolean fieldRelative = true;
  private SwerveModifier modifier = SwerveModifier.forNone();
  private Optional<Double> overrideXSpeed = Optional.empty();
  private Optional<Double> overrideYSpeed = Optional.empty();
  private Optional<Double> overrideRotSpeed = Optional.empty();

  private SwerveModuleState[] curModuleStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  private Pose2d currentPose;
  private Field2d field = new Field2d();
  private double fieldPoseX;
  private double fieldPoseY;

  /**
   * The drive class for swerve robots
   *
   * @param gyro A NavX that's used to get the angle of the robot
   * @param modules Array of swerve modules in the order: front left, front right, back left, back
   *     right
   */
  public SwerveDrive(IGyro gyro, Rotation2d gyroAdjustment, ISwerveModule[] modules) {
    this.gyro = gyro;
    this.gyroAdjustment = gyroAdjustment;
    swerveModules = modules;
    swerveModuleMap =
        Arrays.stream(modules)
            .collect(
                Collectors.toUnmodifiableMap(ISwerveModule::getRobotPosition, module -> module));
    moduleLocations =
        Stream.of(modules).map(module -> module.getLocation()).toArray(Translation2d[]::new);
    kinematics = new SwerveDriveKinematics(moduleLocations);
    gyro.reset();
    poseEstimator =
        new PoseEstimator(
            kinematics,
            gyro.getYaw(),
            getUntimedModulePositions(),
            new Pose2d(),
            VisionConstants.HISTORY_LENGTH_TIME,
            VisionConstants.HISTORY_LENGTH_CAP,
            VisionConstants.HISTORY_LENGTH_MIN,
            VisionConstants.MAX_POSE_VARIANCE_PER);
    currentPose = poseEstimator.getCachedPose();
    sStoppedModules = new HashMap<>();
    for (ModulePosition pos : ModulePosition.values()) {
      sStoppedModules.put(pos, false);
    }
  }

  public ISwerveModule getModule(SwerveParameters.ModulePosition pos) {
    for (ISwerveModule module : swerveModules) {
      if (module.getRobotPosition() == pos) {
        return module;
      }
    }
    return null;
  }

  /**
   * @param xSpeed Speed of the robot in the x direction (forward) in m/s.
   * @param ySpeed Speed of the robot in the y direction (sideways) in m/s.
   * @param rot Angular rate of the robot in radians/sec.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param centerOffset Offset from the robot center for point of rotation
   */
  public void assignModuleStates(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed *= modifier.moveScale();
    ySpeed *= modifier.moveScale();
    rot *= modifier.rotationScale();
    Translation2d centerOffset = modifier.centerOffset();
    if (fieldRelative) {
      centerOffset = centerOffset.rotateBy(gyro.getYaw().unaryMinus());
    }
    SwerveModuleState[] swerveModuleStates =
        kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getYaw())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            centerOffset);
    assignModuleStates(swerveModuleStates);
    LeveledSmartDashboard.INFO.putNumber("xSpeed", xSpeed);
    LeveledSmartDashboard.INFO.putNumber("ySpeed", ySpeed);
    LeveledSmartDashboard.INFO.putNumber("rot", rot);
  }

  public void assignModuleStates(Map<ModulePosition, SwerveModuleState> desiredStates) {
    assignModuleStates(
        Stream.of(swerveModules)
            .map(module -> desiredStates.get(module.getRobotPosition()))
            .toArray(SwerveModuleState[]::new));
  }

  public void assignModuleStates(ChassisSpeeds desiredStates, boolean fieldRelative) {
    assignModuleStates(
        desiredStates.vxMetersPerSecond,
        desiredStates.vyMetersPerSecond,
        desiredStates.omegaRadiansPerSecond,
        fieldRelative);
  }

  /**
   * Used to directly set the state of (and move) the swerve modules
   *
   * @param desiredStates A list of desired states for each of the swerve modules, following the
   *     order passed into {@link SwerveDriveKinematics}.
   */
  public void assignModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates,
        kinematics.toChassisSpeeds(desiredStates),
        DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
        DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
        DriveTrainConstants.SWERVE_ROTATION_MAX_SPEED);
    curModuleStates = desiredStates;
  }

  private void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(states[i]);
    }
  }

  /**
   * @param xSpeed Speed of the robot in the x direction (forward) in m/s.
   * @param ySpeed Speed of the robot in the y direction (sideways) in m/s.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void setTranslationSpeeds(double xSpeed, double ySpeed, boolean fieldRelative) {
    overrideXSpeed = Optional.of(xSpeed);
    overrideYSpeed = Optional.of(ySpeed);
    this.fieldRelative = fieldRelative;
  }

  /**
   * @param rotSpeed Angular rate of the robot in radians/sec.
   */
  public void setRotationSpeed(double rotSpeed) {
    overrideRotSpeed = Optional.of(rotSpeed);
  }

  public void setModifier(SwerveModifier modifier) {
    this.modifier = modifier;
  }

  public void clearModifier() {
    setModifier(SwerveModifier.forNone());
  }

  public SwerveModifier getModifier() {
    return modifier;
  }

  public boolean isFieldRelative() {
    return fieldRelative;
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return kinematics.toChassisSpeeds(curModuleStates);
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    ChassisSpeeds chassisSpeeds = getRobotRelativeChassisSpeeds();
    Translation2d fieldRelativeSpeeds =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(gyro.getYaw());
    return new ChassisSpeeds(
        fieldRelativeSpeeds.getX(),
        fieldRelativeSpeeds.getY(),
        chassisSpeeds.omegaRadiansPerSecond);
  }

  public void stop() {
    overrideXSpeed = Optional.of(0.);
    overrideYSpeed = Optional.of(0.);
    overrideRotSpeed = Optional.of(0.);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    poseEstimator.updateBase(gyro.getYaw(), getTimeCombinedModulePositions());
    poseEstimator.recalculatePose();
  }

  @Override
  public void onStart(boolean sStopped) {
    for (ISwerveModule module : swerveModules) {
      module.onStart(sStopped || sStoppedModules.get(module.getRobotPosition()));
    }
  }

  @Override
  public void onStop(boolean sStopped) {
    for (ISwerveModule module : swerveModules) {
      module.onStop(sStopped || sStoppedModules.get(module.getRobotPosition()));
    }
  }

  @Override
  public void zeroSensors() {
    for (ISwerveModule module : swerveModules) {
      module.zeroSensors();
    }
    gyro.reset();
    resetOdometry(poseEstimator.recalculatePose());
  }

  @Override
  public void updateSmartDashboard() {
    for (ISwerveModule module : swerveModules) {
      module.updateSmartDashboard();
    }
    LeveledSmartDashboard.HIGH.putNumber("CurrentPoseX", fieldPoseX);
    LeveledSmartDashboard.HIGH.putNumber("CurrentPoseY", fieldPoseY);
    LeveledSmartDashboard.HIGH.putNumber("CurrentPoseYaw", currentPose.getRotation().getDegrees());
    LeveledSmartDashboard.HIGH.putData("Field", field);
    LeveledSmartDashboard.HIGH.putNumber("Gyro yaw", gyro.getYaw().getDegrees());
    LeveledSmartDashboard.INFO.putNumber("Gyro pitch", gyro.getPitch().getDegrees());
    LeveledSmartDashboard.INFO.putNumber("Gyro roll", gyro.getRoll().getDegrees());
  }

  @Override
  public void setUpTrackables(LogManager logger) {
    int loggingFrequency = 5;

    log =
        logger
            .getLogFile("SwerveDrive/Position")
            .setDefaultFrequency(loggingFrequency)
            .addTracker("Robot_X_Coordinate", () -> fieldPoseX)
            .addTracker("Robot_Y_Coordinate", () -> fieldPoseY)
            .addTracker("Robot_Angle_(degrees)", () -> gyro.getYaw().getDegrees());

    /**
     * Keeping this here for now in case we need to sort logging files by data type
     *
     * <p>logger.makeLogFile("Swerve Module Desired Speeds", loggingFrequency) .withTrackable("Front
     * Left Desired Speed", m_frontLeft::getDesiredSpeed) .withTrackable("Front Right Desired
     * Speed", m_frontRight::getDesiredSpeed) .withTrackable("Back Left Desired Speed",
     * m_backLeft::getDesiredSpeed) .withTrackable("Back Right Desired Speed",
     * m_backRight::getDesiredSpeed);
     *
     * <p>logger.makeLogFile("Swerve Module Desired Angles", loggingFrequency) .withTrackable("Front
     * Left Desired Angle", m_frontLeft::getDesiredAngle) .withTrackable("Front Right Desired
     * Angle", m_frontRight::getDesiredAngle) .withTrackable("Back Left Desired Angle",
     * m_backLeft::getDesiredAngle) .withTrackable("Back Right Desired Angle",
     * m_backRight::getDesiredAngle);
     *
     * <p>logger.makeLogFile("Swerve Module Actual Speeds", loggingFrequency) .withTrackable("Front
     * Left Actual Speed", m_frontLeft::getSpeed) .withTrackable("Front RightActual Speed",
     * m_frontRight::getSpeed) .withTrackable("Back Left Actual Speed", m_backLeft::getSpeed)
     * .withTrackable("Back Right Actual Speed", m_backRight::getSpeed);
     *
     * <p>logger.makeLogFile("Swerve Module Actual Angles", loggingFrequency) .withTrackable("Front
     * Left Actual Angle", m_frontLeft::getAbsoluteAngleDegrees) .withTrackable("Front Right Actual
     * Angle", m_frontRight::getAbsoluteAngleDegrees) .withTrackable("Back Left Actual Angle",
     * m_backLeft::getAbsoluteAngleDegrees) .withTrackable("Back Right Actual Angle",
     * m_backRight::getAbsoluteAngleDegrees);
     */
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getOdometry() {
    return currentPose;
  }

  public Rotation2d getGyroYaw() {
    return gyro.getYaw();
  }

  public Rotation2d getGyroAdjustment() {
    return gyroAdjustment;
  }

  @SuppressWarnings("unchecked")
  private Timed<SwerveModulePosition>[] getModulePositions() {
    return (Timed<SwerveModulePosition>[])
        Stream.of(swerveModules).map(module -> module.getPosition()).toArray(Timed[]::new);
  }

  private Timed<SwerveModulePosition[]> getTimeCombinedModulePositions() {
    Timed<SwerveModulePosition>[] positions = getModulePositions();
    double time = 0;
    for (Timed<SwerveModulePosition> position : positions) {
      time += position.time();
    }
    time /= positions.length;
    for (Timed<SwerveModulePosition> position : positions) {
      double deltaTime = Math.abs(time - position.time());
      if (deltaTime > 0.1) {
        log.println(
            LogLevel.WARN,
            "Motor position timestamp is very different from average: " + deltaTime + "s");
      }
    }
    return new Timed<>(
        time, Stream.of(positions).map(Timed::value).toArray(SwerveModulePosition[]::new));
  }

  private SwerveModulePosition[] getUntimedModulePositions() {
    return Stream.of(swerveModules)
        .map(module -> module.getPosition().value())
        .toArray(SwerveModulePosition[]::new);
  }

  public void consumeVisionEstimate(Pose2d visionMeasurement, double timestamp) {
    poseEstimator.updateVision(visionMeasurement, timestamp);
    poseEstimator.recalculatePose();
  }

  /**
   * @param target
   * @param threshold in radians
   * @return boolean
   */
  public boolean isYawStable(Rotation2d target, double threshold) {
    return Util.isInRangeWithThreshold(getGyroYaw().getRadians(), target.getRadians(), threshold);
  }

  @SuppressFBWarnings("EI_EXPOSE_REP")
  public Field2d getField() {
    return field;
  }

  @Override
  public void periodic() {
    updateOdometry();
    currentPose = poseEstimator.getCachedPose();
    field.setRobotPose(currentPose);
    fieldPoseX = currentPose.getX();
    fieldPoseY = currentPose.getY();

    checkAndAdjustSpeeds();

    setModuleStates(curModuleStates);

    overrideXSpeed = Optional.empty();
    overrideYSpeed = Optional.empty();
    overrideRotSpeed = Optional.empty();
  }

  public void checkAndAdjustSpeeds() {
    /*
    Note: Instead of all this, we could just convert the states in setModuleStates(SwerveModuleState[])
    to ChassisSpeeds, but that would cause an extra conversion to take place only to go right back
    to SwerveModuleStates. Inverse kinematics may not be a lossless conversion, so that method isn't
    ideal and we should limit performing inverse kinematics as much as possible.
    */
    List<Optional<Double>> presentForceSpeeds =
        List.of(overrideXSpeed, overrideYSpeed, overrideRotSpeed).stream()
            .filter(optionalSpeed -> optionalSpeed.isPresent())
            .toList();
    // All three speeds have been set, so we can ignore previous states
    if (presentForceSpeeds.size() == 3) {
      assignModuleStates(
          overrideXSpeed.get(), overrideYSpeed.get(), overrideRotSpeed.get(), fieldRelative);
      // Some of the speeds are the same, so perform forward kinematics to modify the previously
      // used ChassisSpeeds.
      // Don't change states anything if no speeds have been set this periodic loop
    } else if (!presentForceSpeeds.isEmpty()) {
      ChassisSpeeds prevSpeeds =
          fieldRelative ? getFieldRelativeChassisSpeeds() : getRobotRelativeChassisSpeeds();
      assignModuleStates(
          overrideXSpeed.orElse(prevSpeeds.vxMetersPerSecond).doubleValue(),
          overrideYSpeed.orElse(prevSpeeds.vyMetersPerSecond).doubleValue(),
          overrideRotSpeed.orElse(prevSpeeds.omegaRadiansPerSecond).doubleValue(),
          fieldRelative);
    }
  }

  /**
   * Sets your position on the field
   *
   * @param pose The position on the field you want the robot to think it's at
   */
  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(gyro.getYaw(), getUntimedModulePositions(), pose);
  }

  @Override
  public void onSStop(boolean robotEnabled) {
    for (ISwerveModule module : swerveModules) {
      if (!sStoppedModules.get(module.getRobotPosition())) {
        module.onSStop(robotEnabled);
      }
    }
  }

  @Override
  public void onSRestart(boolean robotEnabled) {
    for (ISwerveModule module : swerveModules) {
      if (!sStoppedModules.get(module.getRobotPosition())) {
        module.onSRestart(robotEnabled);
      }
    }
  }

  public void setSStopped(ModulePosition pos, boolean sStopped) {
    boolean prevSStopped = sStoppedModules.put(pos, sStopped);
    if (prevSStopped != sStopped && !isSStopped()) {
      ISwerveModule module = swerveModuleMap.get(pos);
      if (sStopped) {
        module.onSStop(RobotState.isEnabled());
      } else {
        module.onSRestart(RobotState.isEnabled());
      }
    }
  }

  public boolean isSpecificallySStopped(ModulePosition pos) {
    return sStoppedModules.get(pos);
  }
}
