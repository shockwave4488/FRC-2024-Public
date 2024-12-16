package frc4488.robot.autonomous.modes.supercell;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc4488.lib.autonomous.AutoPIDControllerContainer;
import frc4488.lib.autonomous.PathPlannerUtil;
import frc4488.lib.commands.LogCommand;
import frc4488.lib.geometry.PoseUtil;
import frc4488.lib.logging.LogLevel;
import frc4488.lib.logging.LogManager;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.lib.sensors.gyro.IGyro;
import frc4488.lib.sensors.vision.Limelight;
import frc4488.robot.commands.drive.SwerveDriveToPosition;
import frc4488.robot.commands.other.InitPositionFromTag;
import frc4488.robot.commands.supercell.AutoScoreCommandBuilder;
import frc4488.robot.commands.supercell.arm.MoveArmWithPID;
import frc4488.robot.commands.supercell.drive.DriveAndBalanceOnChargeStation;
import frc4488.robot.commands.supercell.drive.DriveAndBalanceOnChargeStation.ApproachBehavior;
import frc4488.robot.commands.supercell.intake.IntakeCommand;
import frc4488.robot.constants.Constants;
import frc4488.robot.constants.Constants2023.FieldConstants;
import frc4488.robot.constants.Constants2023.GamePiece;
import frc4488.robot.constants.Constants2023.RobotConstants.ArmConstants.ArmSetpoint;
import frc4488.robot.constants.Constants2023.ScoreLevel;
import frc4488.robot.constants.Constants2023.ScorePosition;
import frc4488.robot.robotspecifics.supercell.SelectedConstants;
import frc4488.robot.robotspecifics.supercell.SupercellRobotContainer;
import frc4488.robot.subsystems.drive.SwerveDrive;
import frc4488.robot.subsystems.supercell.Arm;
import frc4488.robot.subsystems.supercell.Intake;
import frc4488.robot.subsystems.supercell.Intake.Speed;
import java.util.function.Supplier;

public class AutonomousChooser {

  private final SwerveDrive swerve;
  private final Arm arm;
  private final Intake intake;
  private final Limelight camera;
  private final IGyro gyro;
  private final AutoPIDControllerContainer pid;
  private final Supplier<TrajectoryConfig> config;
  private final SelectedConstants selectedConstants;
  private final PreferencesParser prefs;
  private final LogManager logger;

  public AutonomousChooser(
      SwerveDrive swerve,
      Arm arm,
      Intake intake,
      Limelight camera,
      IGyro gyro,
      AutoPIDControllerContainer pid,
      Supplier<TrajectoryConfig> config,
      SelectedConstants selectedConstants,
      PreferencesParser prefs,
      LogManager logger) {
    this.swerve = swerve;
    this.arm = arm;
    this.intake = intake;
    this.camera = camera;
    this.gyro = gyro;
    this.pid = pid;
    this.config = config;
    this.selectedConstants = selectedConstants;
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
        () -> DriverStation.getAlliance().map(alliance -> alliance == Alliance.Red).orElse(false),
        swerve.driveRequirement);
  }

  public enum AutonomousMode {
    PATH_PLANNER_CURVE("Path Planner Curve Test"),
    SCORE_AND_BALANCE("Score And Balance"),
    SCORE_AND_PICKUP_TOP("Score And Pickup Top"),
    SCORE_AND_PICKUP_BOTTOM("Score And Pickup Bottom"),
    SCORE_AND_DRIVE_OVER_CHARGE_STATION("Score And Drive Over Charge Station"),
    SCORE_AND_LEAVE_COMMUNITY_TOP("Score And Leave Community Top"),
    SCORE_AND_LEAVE_COMMUNITY_BOTTOM("Score And Leave Community Bottom"),
    SCORE_CENTER("Score Center And BackUp"),
    SCORE_CUBE_AND_BALANCE("Score Cube And Balance"),
    DO_NOTHING("Do Nothing");

    private final String name;

    private AutonomousMode(String name) {
      this.name = name;
    }

    public String getNiceName() {
      return name;
    }
  }

  public Command getDoNothingCommand() {
    return new InstantCommand();
  }

  public Command getAutoScoreCommand(ScoreLevel level, ScorePosition position) {
    return LogCommand.proxy(
        AutoScoreCommandBuilder.createForAnyTag(
                level,
                position,
                swerve,
                gyro,
                intake,
                pid,
                camera,
                () -> true,
                () -> true,
                () -> false)
            .withArm(arm));
  }

  public Command getMoveToCommand(Supplier<Pose2d> pos, boolean reversed) {
    return new SwerveDriveToPosition(
        swerve,
        pid,
        () -> config.get().setReversed(reversed),
        pos,
        () -> new Translation2d[0],
        false);
  }

  public Command getScoreAndBackUpCommand() {
    return LogCommand.sequence(
        resetRobotPoseFromTag(new Pose2d(2.25, 2.15, new Rotation2d(Math.PI))),
        getAutoScoreCommand(ScoreLevel.HIGH, ScorePosition.CONE_LEFT),
        LogCommand.proxy(
            getMoveToCommand(
                () ->
                    swerve
                        .getOdometry()
                        .plus(new Transform2d(new Translation2d(-0.5, 0), new Rotation2d())),
                true)));
  }

  public Command getScoreAndLeaveCommunityCommand(double timeoutSeconds, boolean top) {
    return LogCommand.sequence(
        resetRobotPoseFromTag(new Pose2d(2.25, 2.15, new Rotation2d(Math.PI))),
        getAutoScoreCommand(
            ScoreLevel.HIGH, top ? ScorePosition.CONE_RIGHT : ScorePosition.CONE_LEFT),
        LogCommand.proxy(
            LogCommand.endAfter(
                new RunCommand(
                    () -> swerve.setTranslationSpeeds(2, 0, true), swerve.driveRequirement),
                timeoutSeconds)));
  }

  public Command getScoreAndBalanceCommand() {
    return LogCommand.sequence(
        resetRobotPoseFromTag(new Pose2d(2.25, 2.15, new Rotation2d(Math.PI))),
        getAutoScoreCommand(ScoreLevel.HIGH, ScorePosition.CONE_RIGHT),
        LogCommand.proxy(
            DriveAndBalanceOnChargeStation.create(
                swerve,
                gyro,
                ApproachBehavior.fromVelocity(swerve, () -> 1.8)
                    .withDelayTime(swerve, gyro, 1.8, 1)
                    .withTimeout(4))));
  }

  public Command getScoreCubeAndBalanceCommand() {
    return LogCommand.sequence(
        resetRobotPoseFromTag(new Pose2d(2.25, 2.15, new Rotation2d(Math.PI))),
        LogCommand.proxy(
            LogCommand.sequence(
                LogCommand.endAfter(IntakeCommand.launchCube(intake), 0.5),
                new InstantCommand(() -> intake.setSpeed(Speed.STOPPED), intake),
                DriveAndBalanceOnChargeStation.create(
                    swerve,
                    gyro,
                    ApproachBehavior.fromVelocity(swerve, () -> 1.8)
                        .withDelayTime(swerve, gyro, 1.8, 1)
                        .withTimeout(4)))));
  }

  public Command getScoreAndPickupCommand(boolean top) {
    return LogCommand.sequence(
        resetRobotPoseFromTag(new Pose2d(2.25, top ? 3.91 : 0.48, new Rotation2d(Math.PI))),
        getAutoScoreCommand(
            ScoreLevel.HIGH, top ? ScorePosition.CONE_RIGHT : ScorePosition.CONE_LEFT),
        getMoveToCommand(
            () -> PoseUtil.addTranslation(swerve.getOdometry(), new Translation2d(1, 0)), true),
        LogCommand.proxy(
            new SwerveDriveToPosition(
                swerve,
                pid,
                () -> config.get().setReversed(true),
                () -> PoseUtil.addTranslation(swerve.getOdometry(), new Translation2d(1, 0)),
                () -> new Translation2d[0],
                false)),
        LogCommand.proxy(
            LogCommand.sequence(
                LogCommand.parallel(
                    MoveArmWithPID.createForAuto(arm, ArmSetpoint.PIECE_PICKUP),
                    getMoveToCommand(
                        () ->
                            new Pose2d(
                                FieldConstants.PRESET_PIECE_X - 1.5,
                                FieldConstants.getInstance()
                                    .presetPiecePositions
                                    .get(top ? 1 : 4)
                                    .getY(),
                                new Rotation2d()),
                        false)),
                LogCommand.race(
                    LogCommand.proxy(
                        selectedConstants.gamePieceChoosers.get(top ? 1 : 4).getSelected()
                                == GamePiece.Cube
                            ? IntakeCommand.in(intake, GamePiece.Cube)
                            : SupercellRobotContainer.intakeConeWithHold(intake, prefs)),
                    getMoveToCommand(
                        () ->
                            new Pose2d(
                                FieldConstants.PRESET_PIECE_X,
                                FieldConstants.getInstance()
                                    .presetPiecePositions
                                    .get(top ? 1 : 4)
                                    .getY(),
                                new Rotation2d()),
                        false)))));
  }

  public Command getPathPlannerCurveTestCommand() {
    Command fullCurveTestAuto = PathPlannerUtil.buildAutoWithAlliance("CurvySpinnyPathyThing");
    fullCurveTestAuto.addRequirements(swerve.rotationRequirement);

    logger.getMainLog().println(LogLevel.INFO, "CurvySpinnyThing");

    return resetRobotPose(PathPlannerUtil.getAutoStartingPoseWithAlliance("CurvySpinnyPathyThing"))
        .andThen(fullCurveTestAuto);
  }

  public Command getCommand(AutonomousMode mode) {
    if (mode == null) {
      return getDoNothingCommand();
    }

    Command autoCommand =
        switch (mode) {
          case PATH_PLANNER_CURVE:
            yield getPathPlannerCurveTestCommand();
          case SCORE_AND_BALANCE:
            yield getScoreAndBalanceCommand();
          case SCORE_AND_PICKUP_TOP:
            yield getScoreAndPickupCommand(true);
          case SCORE_AND_PICKUP_BOTTOM:
            yield getScoreAndPickupCommand(false);
          case SCORE_AND_DRIVE_OVER_CHARGE_STATION:
            yield getScoreAndLeaveCommunityCommand(2.5, false);
          case SCORE_AND_LEAVE_COMMUNITY_TOP:
            yield getScoreAndLeaveCommunityCommand(2.0, true);
          case SCORE_AND_LEAVE_COMMUNITY_BOTTOM:
            yield getScoreAndLeaveCommunityCommand(2.0, false);
          case SCORE_CENTER:
            yield getScoreAndBackUpCommand();
          case SCORE_CUBE_AND_BALANCE:
            yield getScoreCubeAndBalanceCommand();
          default:
            yield getDoNothingCommand();
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
}
