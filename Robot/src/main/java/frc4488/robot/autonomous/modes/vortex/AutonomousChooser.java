package frc4488.robot.autonomous.modes.vortex;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc4488.lib.autonomous.AutoPIDControllerContainer;
import frc4488.lib.autonomous.PathPlannerUtil;
import frc4488.lib.commands.LogCommand;
import frc4488.lib.logging.LogLevel;
import frc4488.lib.logging.LogManager;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.lib.sensors.gyro.IGyro;
import frc4488.lib.sensors.vision.Limelight;
import frc4488.robot.commands.other.InitPositionFromTag;
import frc4488.robot.commands.vortex.shooter.Shoot;
import frc4488.robot.constants.Constants;
import frc4488.robot.constants.Constants2024.RobotConstants.ShooterConstants;
import frc4488.robot.subsystems.drive.SwerveDrive;
import frc4488.robot.subsystems.vortex.Arm;
import frc4488.robot.subsystems.vortex.Intake;
import frc4488.robot.subsystems.vortex.Intake.RollerState;
import frc4488.robot.subsystems.vortex.Shooter;
import java.util.function.Supplier;

public class AutonomousChooser {

  private final SwerveDrive swerve;
  private final Limelight camera;
  private final Intake intake;
  private final Shooter shooter;
  private final Arm arm;
  private final IGyro gyro;
  private final AutoPIDControllerContainer pid;
  private final Supplier<TrajectoryConfig> config;
  private final PreferencesParser prefs;
  private final LogManager logger;
  private static final double SHOOT_TIMEOUT = 2;
  private static final double PODIUM_SHOT_TIMEOUT = 2;
  private static final double AUTO_SHOT_TIMEOUT = 2.5;

  public AutonomousChooser(
      SwerveDrive swerve,
      Limelight camera,
      Intake intake,
      Shooter shooter,
      Arm arm,
      IGyro gyro,
      AutoPIDControllerContainer pid,
      Supplier<TrajectoryConfig> config,
      PreferencesParser prefs,
      LogManager logger) {
    this.swerve = swerve;
    this.camera = camera;
    this.intake = intake;
    this.arm = arm;
    this.shooter = shooter;
    this.gyro = gyro;
    this.pid = pid;
    this.config = config;
    this.prefs = prefs;
    this.logger = logger;

    AutoBuilder.configureHolonomic(
        swerve::getOdometry,
        swerve::resetOdometry,
        swerve::getRobotRelativeChassisSpeeds,
        speeds -> swerve.assignModuleStates(speeds, false),
        new HolonomicPathFollowerConfig(
            new PIDConstants(
                prefs.getDouble("AutoPosPathP"),
                prefs.getDouble("AutoPosPathI"),
                prefs.getDouble("AutoPosPathD")),
            new PIDConstants(prefs.getDouble("AutoTurnP"), 0.0, 0.0),
            Constants.DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
            prefs.getDouble("DriveBaseRadius"),
            new ReplanningConfig()),
        // We aren't flipping the path here because our code maintains an origin based on whichever
        // alliance we are on. PathPlanner's origin is ALWAYS blue, which throws off our pose
        // estimation.
        () -> false,
        swerve.driveRequirement);
    registerCommands();
  }

  public enum AutonomousMode {
    PATH_PLANNER_CURVE("Path Planner Curve Test"),
    NOTHING("Do Nothing"),
    ONE_NOTE_SOURCE_SIDE("1 Note Source-side"),
    ONE_NOTE_AMP_SIDE("1 Note Amp-Side"),
    ONE_NOTE_CENTER("1 Note Center"),
    TWO_NOTE_SOURCE_SIDE("2 Note Source-side"),
    TWO_NOTE_AMP_SIDE("2 Note Amp-Side"),
    TWO_NOTE_CENTER("2 Note Center"),
    THREE_NOTE_WITH_TWO_CENTER_NOTES("3 Note With Center 4&5 (Source)"),
    THREE_NOTE_WITH_ONE_CENTER_NOTE("3 Note With Center 5 (Source)"),
    SHORTER_THREE_NOTE_WITH_ONE_CENTER_NOTE("Shorter 3 Note With Center 5 (Source)"),
    FOUR_NOTE_WITH_THREE_CENTER_NOTES("(UNTESTED) 4 Note With Center 3-5 (Source)"),
    FOUR_NOTE_SOURCE_WITH_CLOSE_NOTES("4 Note With Close 1-3 (Source)"),
    FOUR_NOTE_SOURCE_WITH_CLOSE_NOTES_AND_ONE_CENTER(
        "(UNTESTED) 4 Note With Close 1-3 and Center 5 (Source)"),
    STRAIGHT("Straight");

    private final String name;

    private AutonomousMode(String name) {
      this.name = name;
    }

    public String getNiceName() {
      return name;
    }
  }

  private void registerCommands() {
    NamedCommands.registerCommand("intakeGo", intakeStartCommand());
    NamedCommands.registerCommand("intakeStop", intakeStopCommand());
    NamedCommands.registerCommand("intakeStopArmUp", intakeStopWithArmCommand());
    NamedCommands.registerCommand("podiumShot", podiumShotCommand());
    NamedCommands.registerCommand("shoot", shootCommand());
    NamedCommands.registerCommand("autoShot", currentSpikeAutoShotCommand());
  }

  // Example Auto
  public Command getPathPlannerCurveTestCommand() {
    // Assign a command that builds the desired .auto auto
    Command fullCurveTestAuto = PathPlannerUtil.buildAutoWithAlliance("CurvySpinnyPathyThing");
    // Add the rotation requirement for swerve
    fullCurveTestAuto.addRequirements(swerve.rotationRequirement);

    // Write the path to the log
    logger.getMainLog().println(LogLevel.INFO, "CurvySpinnyPathyThing");

    // Start each command sequence with a pose reset. This can be using april tags or according
    // to the first path segment of the auto.
    // Next, run the built auto command. Any Named Commands part of waypoints will run if recorded
    // as named commands
    // in the registerCommands() method above.
    return resetRobotPose(PathPlannerUtil.getAutoStartingPoseWithAlliance("CurvySpinnyPathyThing"))
        .andThen(fullCurveTestAuto);
  }

  // Example auto using paths
  public Command getStraightPath() {
    PathPlannerPath autoPath = PathPlannerPath.fromPathFile("Straight");

    Command followPath = AutoBuilder.followPath(autoPath);

    return resetRobotPose(PathPlannerUtil.getAutoStartingPoseWithAlliance("Straight"))
        .andThen(followPath);
  }

  // One Note Autos
  public Command getOneNoteAutoSourceSideCommand() {
    logger.getMainLog().println(LogLevel.INFO, "OneNoteSourceAuto");

    return resetRobotPose(PathPlannerUtil.getAutoStartingPoseWithAlliance("TwoNoteSourceAuto"))
        .andThen(shootCommand());
  }

  public Command getOneNoteCenterCommand() {
    logger.getMainLog().println(LogLevel.INFO, "OneNoteCenterAuto");

    return resetRobotPose(PathPlannerUtil.getAutoStartingPoseWithAlliance("TwoNoteCenterAuto"))
        .andThen(shootCommand());
  }

  public Command getOneNoteAmpSideCommand() {
    logger.getMainLog().println(LogLevel.INFO, "OneNoteAmpAuto");

    return resetRobotPose(PathPlannerUtil.getAutoStartingPoseWithAlliance("TwoNoteAmpAuto"))
        .andThen(shootCommand());
  }

  // Two Note Autos
  public Command getTwoNoteAutoSourceSideCommand() {
    Command autoPath = PathPlannerUtil.buildAutoWithAlliance("TwoNoteSourceAuto");
    autoPath.addRequirements(swerve.rotationRequirement);

    logger.getMainLog().println(LogLevel.INFO, "TwoNoteSourceAuto");

    return resetRobotPose(PathPlannerUtil.getAutoStartingPoseWithAlliance("TwoNoteSourceAuto"))
        .andThen(shootCommand().andThen(autoPath).andThen(shootCommand()));
  }

  public Command getTwoNoteCenterCommand() {
    Command autoPath = PathPlannerUtil.buildAutoWithAlliance("TwoNoteCenterAuto");
    autoPath.addRequirements(swerve.rotationRequirement);

    logger.getMainLog().println(LogLevel.INFO, "TwoNoteCenterAuto");

    return resetRobotPose(PathPlannerUtil.getAutoStartingPoseWithAlliance("TwoNoteCenterAuto"))
        .andThen(shootCommand().andThen(autoPath).andThen(shootCommand()));
  }

  public Command getTwoNoteAmpSideCommand() {
    Command autoPath = PathPlannerUtil.buildAutoWithAlliance("TwoNoteAmpAuto");
    autoPath.addRequirements(swerve.rotationRequirement);

    logger.getMainLog().println(LogLevel.INFO, "TwoNoteAmpAuto");

    return resetRobotPose(PathPlannerUtil.getAutoStartingPoseWithAlliance("TwoNoteAmpAuto"))
        .andThen(shootCommand().andThen(autoPath).andThen(shootCommand()));
  }

  // Three Note Autos
  public Command getThreeNoteWithOneCenterNoteCommand() {
    Command autoPath =
        PathPlannerUtil.buildAutoWithAlliance("ThreeNoteSourceCloseThreeAndCenterFiveAuto");
    autoPath.addRequirements(swerve.rotationRequirement);

    logger.getMainLog().println(LogLevel.INFO, "ThreeNoteSourceCloseThreeAndCenterFiveAuto");

    return resetRobotPose(
            PathPlannerUtil.getAutoStartingPoseWithAlliance(
                "ThreeNoteSourceCloseThreeAndCenterFiveAuto"))
        .andThen(shootCommand())
        .andThen(autoPath);
  }

  public Command getShorterThreeNoteWithOneCenterNoteCommand() {
    Command autoPath =
        PathPlannerUtil.buildAutoWithAlliance("ShorterThreeNoteSourceCloseThreeAndCenterFiveAuto");
    autoPath.addRequirements(swerve.rotationRequirement);

    logger.getMainLog().println(LogLevel.INFO, "ShorterThreeNoteCloseThreeAndCenterFiveAuto");

    return resetRobotPose(
            PathPlannerUtil.getAutoStartingPoseWithAlliance(
                "ShorterThreeNoteSourceCloseThreeAndCenterFiveAuto"))
        .andThen(shootCommand())
        .andThen(autoPath);
  }

  public Command getThreeNoteWithTwoCenterNotesCommand() {
    Command autoPath =
        PathPlannerUtil.buildAutoWithAlliance("ThreeNoteSourceWithCenterFourAndFiveAuto");
    autoPath.addRequirements(swerve.rotationRequirement);

    logger.getMainLog().println(LogLevel.INFO, "ThreeNoteSourceWithCenterFourAndFiveAuto");

    return resetRobotPose(
            PathPlannerUtil.getAutoStartingPoseWithAlliance(
                "ThreeNoteSourceWithCenterFourAndFiveAuto"))
        .andThen(shootCommand())
        .andThen(autoPath);
  }

  // Four Note Auto
  public Command getFourNoteWithThreeCenterNotesCommand() {
    Command autoPath =
        PathPlannerUtil.buildAutoWithAlliance("FourNoteSourceWithCenterThreeToFiveAuto");
    autoPath.addRequirements(swerve.rotationRequirement);

    logger.getMainLog().println(LogLevel.INFO, "FourNoteSourceWithCenterThreeToFiveAuto");

    return resetRobotPose(
            PathPlannerUtil.getAutoStartingPoseWithAlliance(
                "FourNoteSourceWithCenterThreeToFiveAuto"))
        .andThen(shootCommand())
        .andThen(autoPath);
  }

  public Command getFourNoteSourceWithCloseNotesCommand() {
    Command autoPath = PathPlannerUtil.buildAutoWithAlliance("FourNoteSourceCloseThreeToOneAuto");
    autoPath.addRequirements(swerve.rotationRequirement);

    logger.getMainLog().println(LogLevel.INFO, "FourNoteSourceCloseThreeToOneAuto");

    return resetRobotPose(
            PathPlannerUtil.getAutoStartingPoseWithAlliance("FourNoteSourceCloseThreeToOneAuto"))
        .andThen(shootCommand())
        .andThen(autoPath);
  }

  public Command getFourNoteSourceWithCloseNotesAndOneCenterNoteCommand() {
    Command autoPath =
        PathPlannerUtil.buildAutoWithAlliance("FourNoteSourceCloseThreeToOneWithCenterThreeAuto");
    autoPath.addRequirements(swerve.rotationRequirement);

    logger.getMainLog().println(LogLevel.INFO, "FourNoteSourceCloseThreeToOneWithCenterThreeAuto");

    return resetRobotPose(
            PathPlannerUtil.getAutoStartingPoseWithAlliance(
                "FourNoteSourceCloseThreeToOneWithCenterThreeAuto"))
        .andThen(shootCommand())
        .andThen(autoPath);
  }

  // Default
  public Command getDoNothingCommand() {
    return resetRobotPoseFromTag(new Pose2d(new Translation2d(1, 1), new Rotation2d(0)));
  }

  public Command getCommand(AutonomousMode mode) {
    if (mode == null) {
      return getDoNothingCommand();
    }

    Command autoCommand =
        switch (mode) {
          case PATH_PLANNER_CURVE -> getPathPlannerCurveTestCommand();
          case ONE_NOTE_SOURCE_SIDE -> getOneNoteAutoSourceSideCommand();
          case ONE_NOTE_AMP_SIDE -> getOneNoteAmpSideCommand();
          case ONE_NOTE_CENTER -> getOneNoteCenterCommand();
          case TWO_NOTE_SOURCE_SIDE -> getTwoNoteAutoSourceSideCommand();
          case TWO_NOTE_AMP_SIDE -> getTwoNoteAmpSideCommand();
          case TWO_NOTE_CENTER -> getTwoNoteCenterCommand();
          case THREE_NOTE_WITH_ONE_CENTER_NOTE -> getThreeNoteWithOneCenterNoteCommand();
          case THREE_NOTE_WITH_TWO_CENTER_NOTES -> getThreeNoteWithTwoCenterNotesCommand();
          case SHORTER_THREE_NOTE_WITH_ONE_CENTER_NOTE -> getShorterThreeNoteWithOneCenterNoteCommand();
          case FOUR_NOTE_WITH_THREE_CENTER_NOTES -> getFourNoteWithThreeCenterNotesCommand();
          case FOUR_NOTE_SOURCE_WITH_CLOSE_NOTES -> getFourNoteSourceWithCloseNotesCommand();
          case FOUR_NOTE_SOURCE_WITH_CLOSE_NOTES_AND_ONE_CENTER -> getFourNoteSourceWithCloseNotesAndOneCenterNoteCommand();
          case STRAIGHT -> getStraightPath();
          default -> getDoNothingCommand();
        };
    autoCommand.setName(mode.getNiceName().replaceAll("\\s+", ""));
    return autoCommand;
  }

  private Command resetRobotPose(Pose2d pose) {
    return new InstantCommand(
        () -> {
          gyro.setYawAdjustment(new Rotation2d());
          gyro.setYawAdjustment(pose.getRotation().minus(gyro.getYaw()));
          swerve.resetOdometry(pose);
        });
  }

  private Command resetRobotPoseFromTag(Pose2d fallback) {
    return LogCommand.race(
        InitPositionFromTag.create(swerve, gyro, camera),
        new WaitCommand(0.5).andThen(resetRobotPose(fallback)));
  }

  private Command intakeStartCommand() {
    return new InstantCommand(() -> shooter.brake())
        .andThen(intake.loadWithSensor())
        .andThen(intake.intakeCommand(RollerState.OFF));
  }

  private Command intakeStopWithArmCommand() {
    return intake
        .intakeCommand(RollerState.OFF)
        .andThen(arm.getMoveToCommand(Arm.Position.SPEAKER))
        .andThen(() -> shooter.setRPS(ShooterConstants.WAIT_SPEED));
  }

  private Command intakeStopCommand() {
    return intake
        .intakeCommand(RollerState.OFF)
        .andThen(arm.getMoveToCommand(Arm.Position.CROUCH))
        .andThen(() -> shooter.setRPS(ShooterConstants.WAIT_SPEED));
  }

  private Command shootCommand() {
    // TODO: Find a way to run this in paths without needing a timeout.
    return Shoot.subwooferShot(shooter, arm, intake)
        .withTimeout(SHOOT_TIMEOUT)
        .andThen(resetSubsystemsCommand());
  }

  private Command podiumShotCommand() {
    // TODO: Find a way to run this in paths without needing a timeout.
    return Shoot.podiumShot(arm, intake, shooter, swerve, gyro, pid.thetaPidController, () -> 0.0)
        .withTimeout(PODIUM_SHOT_TIMEOUT)
        .andThen(resetSubsystemsCommand());
  }

  private Command autoShotCommand() {
    return intake
        .intakeCommand(RollerState.OFF)
        .withTimeout(0.05)
        .andThen(
            Shoot.trackArmAndDrive(
                    arm,
                    intake,
                    shooter,
                    swerve,
                    gyro,
                    pid.thetaPidController,
                    ShooterConstants.SPEED,
                    () -> 0.0)
                .alongWith(Shoot.shot(shooter, arm, intake)))
        .withTimeout(AUTO_SHOT_TIMEOUT)
        .andThen(resetSubsystemsCommand());
  }

  private Command currentSpikeAutoShotCommand() {
    return intake
        .intakeCommand(RollerState.OFF)
        .withTimeout(0.05)
        .andThen(
            Shoot.trackArmAndDrive(
                    arm,
                    intake,
                    shooter,
                    swerve,
                    gyro,
                    pid.thetaPidController,
                    ShooterConstants.SPEED,
                    () -> 0.0)
                .raceWith(
                    shooter
                        .detectCurrentSpikeCommand()
                        .andThen(shooter.detectCurrentSpikeCommand())
                        .deadlineWith(Shoot.shot(shooter, arm, intake)))
                .withTimeout(AUTO_SHOT_TIMEOUT))
        .andThen(resetSubsystemsCommand());
  }

  public Command resetSubsystemsCommand() {
    return arm.getMoveToCommand(Arm.Position.INTAKE)
        .alongWith(intake.intakeCommand(RollerState.OFF))
        .alongWith(new InstantCommand(() -> shooter.coastOut()))
        .withTimeout(0.05);
  }

  public String getAutoModeChooserKey() {
    return "/Shuffleboard/Competition/Auto mode";
  }
}
