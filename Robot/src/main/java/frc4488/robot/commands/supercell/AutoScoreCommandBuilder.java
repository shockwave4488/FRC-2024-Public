package frc4488.robot.commands.supercell;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc4488.lib.autonomous.AutoPIDControllerContainer;
import frc4488.lib.commands.CommandUtil;
import frc4488.lib.commands.LogCommand;
import frc4488.lib.math.PoseEstimationUtil;
import frc4488.lib.misc.Util;
import frc4488.lib.sensors.gyro.IGyro;
import frc4488.lib.sensors.vision.VisionCameras.AprilTagCamera;
import frc4488.lib.sensors.vision.VisionTargets.AprilTagTarget;
import frc4488.robot.commands.drive.DriveRelativeToAprilTag;
import frc4488.robot.commands.drive.DriveRelativeToAprilTag.RelativeGoalPose;
import frc4488.robot.commands.drive.RotateToAngle;
import frc4488.robot.commands.drive.SwerveModifierCommand.SwerveModifier;
import frc4488.robot.commands.supercell.arm.MoveArmWithPID;
import frc4488.robot.commands.supercell.intake.IntakeCommand;
import frc4488.robot.constants.Constants.DriveTrainConstants;
import frc4488.robot.constants.Constants2023;
import frc4488.robot.constants.Constants2023.GamePiece;
import frc4488.robot.constants.Constants2023.ScoreLevel;
import frc4488.robot.constants.Constants2023.ScorePosition;
import frc4488.robot.subsystems.drive.SwerveDrive;
import frc4488.robot.subsystems.supercell.Arm;
import frc4488.robot.subsystems.supercell.Intake;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public class AutoScoreCommandBuilder {
  public final ScoreLevel level;
  public final ScorePosition pos;
  public final SwerveDrive swerve;
  public final IGyro gyro;
  public final Intake intake;
  public final AutoPIDControllerContainer pid;
  public final Supplier<TrajectoryConfig> config;
  public final Supplier<Optional<? extends AprilTagTarget>> tag;
  public final Transform3d robotCenterToCamera;
  public final BooleanSupplier confirmPathFollowing;
  public final BooleanSupplier confirmPieceDrop;
  public final BooleanSupplier shouldCancel;
  public final GamePiece piece;
  public final Command autoScoreCommand;

  private static boolean setTargetIfVisible(
      AtomicReference<AprilTag> targetPose,
      SwerveDrive swerve,
      Transform3d robotCenterToCamera,
      Supplier<Optional<? extends AprilTagTarget>> tagSupplier) {
    Optional<? extends AprilTagTarget> possibleTarget = tagSupplier.get();
    if (possibleTarget.isPresent()) {
      // Keeps the drive working, even if the target disappears
      targetPose.set(possibleTarget.get().toAprilTag(swerve.getOdometry(), robotCenterToCamera));
      return true;
    }
    return false;
  }

  public AutoScoreCommandBuilder(
      ScoreLevel level,
      ScorePosition pos,
      SwerveDrive swerve,
      IGyro gyro,
      Intake intake,
      AutoPIDControllerContainer pid,
      Supplier<TrajectoryConfig> config,
      Supplier<Optional<? extends AprilTagTarget>> tag,
      Transform3d robotCenterToCamera,
      BooleanSupplier confirmPathFollowing,
      BooleanSupplier confirmPieceDrop,
      BooleanSupplier shouldCancel) {
    this.level = level;
    this.pos = pos;
    this.swerve = swerve;
    this.gyro = gyro;
    this.intake = intake;
    this.pid = pid;
    this.config = config;
    this.tag = tag;
    this.robotCenterToCamera = robotCenterToCamera;
    this.confirmPathFollowing = confirmPathFollowing;
    this.confirmPieceDrop = confirmPieceDrop;
    this.shouldCancel = shouldCancel;
    piece = (level == ScoreLevel.LOW ? GamePiece.Cube : pos.getPiece());
    AtomicReference<AprilTag> targetPose = new AtomicReference<>();
    autoScoreCommand =
        CommandUtil.withName(
            LogCommand.endWhen(
                LogCommand.sequence(
                    CommandUtil.withName(
                        new WaitUntilCommand(
                            () ->
                                confirmPathFollowing.getAsBoolean()
                                    && setTargetIfVisible(
                                        targetPose, swerve, robotCenterToCamera, tag)),
                        "WaitForTagAndConfirmation"),
                    // "AutoScore > Arm -> Drive",
                    LogCommand.proxy(
                        CommandUtil.withName(
                            new DriveRelativeToAprilTag(
                                swerve,
                                pid,
                                () -> config.get().setReversed(false),
                                targetPose::get,
                                RelativeGoalPose.identity()
                                    .translate(
                                        () ->
                                            new Translation2d(
                                                0.936
                                                    + level.getOffsetFromHigh()
                                                    + piece.getIntakeOffset(),
                                                pos.getOffset()
                                                    + (piece == GamePiece.Cone
                                                        ? intake.getConePosition()
                                                        : 0)))
                                    .withRotation(() -> new Rotation2d(Math.PI))),
                            "Align")),
                    // "AutoScore > Drive -> Confirm Drop",
                    LogCommand.deadline(
                        LogCommand.sequence(
                            CommandUtil.withName(
                                new WaitUntilCommand(confirmPieceDrop), "PieceDropWait"),
                            // "AutoScore > Confirm Drop -> Drop",
                            LogCommand.endAfter(
                                LogCommand.proxy(
                                    LogCommand.either(
                                        CommandUtil.withName(
                                            IntakeCommand.out(intake, GamePiece.Cube), "DropCube"),
                                        CommandUtil.withName(
                                            IntakeCommand.out(intake, GamePiece.Cone), "DropCone"),
                                        () -> piece == GamePiece.Cube)),
                                1)),
                        LogCommand.parallel(
                            LogCommand.proxy(
                                new RotateToAngle(
                                    swerve,
                                    gyro,
                                    pid.thetaPidController,
                                    () -> new Rotation2d(Math.PI))),
                            LogCommand.proxy(
                                CommandUtil.withName(
                                    new StartEndCommand(
                                        () -> swerve.setModifier(SwerveModifier.forSpeed(0.25)),
                                        swerve::clearModifier,
                                        swerve.modifierRequirement),
                                    "LowerSpeedModifier"))))),
                shouldCancel),
            "AutoAlignAndDrop");
  }

  public static AutoScoreCommandBuilder createForAnyTag(
      ScoreLevel level,
      ScorePosition pos,
      SwerveDrive swerve,
      IGyro gyro,
      Intake intake,
      AutoPIDControllerContainer pid,
      AprilTagCamera camera,
      BooleanSupplier confirmPathFollowing,
      BooleanSupplier confirmPieceDrop,
      BooleanSupplier shouldCancel) {
    return new AutoScoreCommandBuilder(
        level,
        pos,
        swerve,
        gyro,
        intake,
        pid,
        () ->
            new TrajectoryConfig(
                DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED / 3,
                DriveTrainConstants.SWERVE_DRIVE_MAX_ACCEL / 2),
        () ->
            PoseEstimationUtil.getBestTarget(
                camera, Constants2023.FieldConstants.cubeScoringTagIds),
        camera.getCameraPositionConsts().robotCenterToCamera,
        confirmPathFollowing,
        confirmPieceDrop,
        shouldCancel);
  }

  public static void bindToButtonBox(
      SwerveDrive swerve,
      IGyro gyro,
      Intake intake,
      AutoPIDControllerContainer pid,
      AprilTagCamera camera,
      Function<AutoScoreCommandBuilder, Command> commandFromBuilder,
      CommandGenericHID box,
      BooleanSupplier shouldCancel,
      int[][] bindings) {
    int y = 0;
    for (ScoreLevel level : ScoreLevel.values()) {
      int x = 0;
      for (ScorePosition pos : ScorePosition.values()) {
        int binding = bindings[y][x];
        if (binding <= 0) {
          continue;
        }
        Trigger buttonTrigger = box.button(binding);
        box.button(binding)
            .onTrue(
                commandFromBuilder.apply(
                    createForAnyTag(
                        level,
                        pos,
                        swerve,
                        gyro,
                        intake,
                        pid,
                        camera,
                        buttonTrigger,
                        buttonTrigger,
                        shouldCancel)));
        x++;
      }
      y++;
    }
  }

  public Command withArm(Arm arm) {
    AtomicReference<Double> safeXValue = new AtomicReference<>();
    return CommandUtil.withName(
        LogCommand.sequence(
            MoveArmWithPID.createForAuto(arm, level.getArmSetpoint()),
            autoScoreCommand,
            LogCommand.either(
                LogCommand.sequence(
                    new InstantCommand(
                        () ->
                            safeXValue.set(
                                0.35
                                    - level.getArmSetpoint().getArmOutDistance()
                                    + piece.getIntakeOffset()
                                    + swerve.getOdometry().getTranslation().getX())),
                    new ScheduleCommand(
                        Util.returnAfterModifying(
                            CommandUtil.withName(
                                new WaitUntilCommand(
                                    () ->
                                        swerve.getOdometry().getTranslation().getX()
                                                - level.getArmSetpoint().getArmOutDistance()
                                                    * Math.cos(gyro.getYaw().getRadians() - Math.PI)
                                            > safeXValue.get()),
                                "ArmSafetyWait"),
                            cmd -> cmd.addRequirements(arm)))),
                new InstantCommand(),
                () -> level != ScoreLevel.LOW)),
        "AutoScore");
  }
}
