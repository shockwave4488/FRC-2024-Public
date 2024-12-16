package frc4488.robot.commands.supercell.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc4488.lib.autonomous.AutoPIDControllerContainer;
import frc4488.lib.commands.CommandUtil;
import frc4488.lib.commands.LogCommand;
import frc4488.lib.sensors.gyro.IGyro;
import frc4488.lib.sensors.vision.VisionCameras.AprilTagCamera;
import frc4488.lib.sensors.vision.VisionTargets.AprilTagTarget;
import frc4488.robot.commands.drive.DriveRelativeToAprilTag;
import frc4488.robot.commands.drive.DriveRelativeToAprilTag.RelativeGoalPose;
import frc4488.robot.commands.drive.RotateToAngle;
import frc4488.robot.commands.drive.SwerveModifierCommand.SwerveModifier;
import frc4488.robot.commands.supercell.arm.MoveArmWithPID;
import frc4488.robot.constants.Constants.DriveTrainConstants;
import frc4488.robot.constants.Constants2023.DoubleSubstationSide;
import frc4488.robot.constants.Constants2023.FieldConstants;
import frc4488.robot.constants.Constants2023.RobotConstants.ArmConstants;
import frc4488.robot.constants.Constants2023.RobotConstants.ArmConstants.ArmSetpoint;
import frc4488.robot.constants.Constants2023.RobotConstants.IntakeConstants;
import frc4488.robot.subsystems.drive.SwerveDrive;
import frc4488.robot.subsystems.supercell.Arm;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class AlignToSubstation {
  private AlignToSubstation() {}

  public static Command from(
      SwerveDrive swerve,
      IGyro gyro,
      AprilTagCamera camera,
      Arm arm,
      AutoPIDControllerContainer pidControllers,
      Supplier<DoubleSubstationSide> substationSide) {
    AtomicReference<AprilTagTarget> target = new AtomicReference<>();
    return CommandUtil.withName(
        LogCommand.parallel(
            LogCommand.sequence(
                new WaitUntilCommand(
                    () -> {
                      Optional<? extends AprilTagTarget> possibleTarget =
                          camera.getBestAprilTagTarget();
                      if (possibleTarget.isPresent()
                          && possibleTarget.get().getId()
                              == FieldConstants.getInstance().substationTagId) {
                        target.set(possibleTarget.get());
                        return true;
                      }
                      return false;
                    }),
                LogCommand.proxy(
                    new DriveRelativeToAprilTag(
                        swerve,
                        pidControllers,
                        () ->
                            new TrajectoryConfig(
                                DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED / 2,
                                DriveTrainConstants.SWERVE_DRIVE_MAX_ACCEL / 2),
                        () ->
                            target
                                .get()
                                .toAprilTag(
                                    swerve.getOdometry(),
                                    camera.getCameraPositionConsts().robotCenterToCamera),
                        RelativeGoalPose.identity()
                            .translate(
                                () ->
                                    new Translation2d(
                                        -(ArmConstants.ARM_LENGTH
                                                + Units.inchesToMeters(
                                                    IntakeConstants.EXTENSION_LENGTH))
                                            * Math.cos(ArmSetpoint.SUBSTATION.angleRadians),
                                        substationSide.get().offsetToSideCenter))
                            .withRotation(() -> new Rotation2d()))),
                LogCommand.proxy(
                    LogCommand.parallel(
                        new RotateToAngle(
                            swerve,
                            gyro,
                            pidControllers.thetaPidController,
                            () -> new Rotation2d()),
                        new StartEndCommand(
                            () -> swerve.setModifier(SwerveModifier.forSpeed(0.5)),
                            swerve::clearModifier,
                            swerve.modifierRequirement)))),
            new MoveArmWithPID(arm, ArmSetpoint.SUBSTATION)),
        "AlignToSubstation");
  }
}
