package frc4488.robot.commands.vortex.shooter;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc4488.lib.commands.DoneCycleCommand;
import frc4488.lib.commands.LogCommand;
import frc4488.lib.controlsystems.DoneCycleMachine;
import frc4488.lib.math.PoseEstimationUtil;
import frc4488.lib.sensors.gyro.IGyro;
import frc4488.robot.commands.drive.RotateTowardsPosition;
import frc4488.robot.commands.drive.SmartDriveToPosition;
import frc4488.robot.constants.Constants;
import frc4488.robot.constants.Constants2024;
import frc4488.robot.subsystems.drive.SwerveDrive;
import frc4488.robot.subsystems.vortex.Arm;
import frc4488.robot.subsystems.vortex.Arm.Position;
import frc4488.robot.subsystems.vortex.Intake;
import frc4488.robot.subsystems.vortex.Intake.RollerState;
import frc4488.robot.subsystems.vortex.Shooter;
import java.util.function.Supplier;

public class Shoot {
  public Shoot() {}

  private static Command moveNoteBack(Intake intake, Shooter shooter) {
    return intake
        .intakeCommand(Intake.RollerState.SLOW_REVERSE)
        .alongWith(shooter.reverseCommand())
        .until(() -> !intake.hasNote())
        .withTimeout(0.5)
        .finallyDo(interrupted -> intake.setRollerState(Intake.RollerState.OFF))
        .withName("Move Note Back");
  }

  public static Command fixNotePosition(Intake intake, Shooter shooter) {
    return intake
        .intakeCommand(RollerState.FORWARD)
        .withTimeout(0.5)
        .andThen(
            intake
                .intakeCommand(RollerState.SLOW_REVERSE)
                .alongWith(shooter.reverseCommand())
                .until(intake::hasNote))
        .andThen(new WaitUntilCommand(() -> !intake.hasNote()));
  }

  public static Command shot(Shooter shooter, Arm arm, Intake intake) {
    return LogCommand.sequence(
        moveNoteBack(intake, shooter),
        new DoneCycleCommand<>(shooter.shootSubwooferCommand(), true)
            .withDoneCycles(
                DoneCycleMachine.supplierWithMinCycles(
                    () -> arm.isStable(false) && shooter.isStable(false, false), 5)),
        intake.intakeCommand(RollerState.FORWARD));
  }

  /** Sequence of commands that scores into the subwoofer. */
  public static Command subwooferShot(Shooter shooter, Arm arm, Intake intake) {
    // Move the arm to the subwoofer position and prep the shooter in parallel
    return LogCommand.sequence(
        moveNoteBack(intake, shooter),
        new DoneCycleCommand<>(
                arm.getMoveToCommand(Position.SPEAKER).alongWith(shooter.shootSubwooferCommand()),
                true)
            // wait until the arm & shooter are stable
            .withDoneCycles(
                DoneCycleMachine.supplierWithMinCycles(
                    () -> (arm.isStable(true) && shooter.isStable(false, true)), 5))
            // run the intake rollers to push the note further in
            .andThen(intake.intakeCommand(RollerState.FORWARD)));
  }

  /** Sequence of commands that scores into the amp. */
  public static Command ampShot(Shooter shooter, Arm arm, Intake intake) {
    // Move the arm to the amp position and prep the shooter in parallel
    return new DoneCycleCommand<>(
            arm.getMoveToCommand(Position.AMP).alongWith(shooter.shootAmpCommand()), true)
        // wait until the arm & shooter are stable
        .withDoneCycles(
            DoneCycleMachine.supplierWithMinCycles(
                () -> (arm.isStable(true) && shooter.isStable(false, false)), 5))
        // run the intake rollers to push the note further in
        .andThen(intake.intakeCommand(RollerState.FORWARD));
  }

  public static Command trackArmAndDrive(
      Arm arm,
      Intake intake,
      Shooter shooter,
      SwerveDrive swerve,
      IGyro gyro,
      ProfiledPIDController thetaController,
      double shootingMps,
      Supplier<Double> angleOffset) {
    return LogCommand.parallel(
            moveNoteBack(intake, shooter).andThen(shooter.shootSubwooferCommand()).asProxy(),
            arm.getTrackCommand(swerve, shootingMps, angleOffset),
            new RotateTowardsPosition(
                    swerve,
                    gyro,
                    thetaController,
                    () -> {
                      Translation2d speaker =
                          Constants2024.FieldConstants.getInstance()
                              .speakerOpeningCenter
                              .toTranslation2d();
                      return PoseEstimationUtil.accountForRobotVelocity(
                              speaker, swerve, shootingMps)
                          .orElse(speaker);
                    },
                    Rotation2d.fromRadians(Math.PI))
                .keepUpdating(true))
        .withName("Track Arm and Drive");
  }

  public static Command podiumShot(
      Arm arm,
      Intake intake,
      Shooter shooter,
      SwerveDrive swerve,
      IGyro gyro,
      ProfiledPIDController thetaController,
      Supplier<Double> angleOffset) {
    RotateTowardsPosition rotateCommand =
        new RotateTowardsPosition(
            swerve,
            gyro,
            thetaController,
            () -> Constants2024.FieldConstants.getInstance().speakerOpeningCenter.toTranslation2d(),
            Rotation2d.fromRadians(Math.PI));
    return LogCommand.sequence(
        moveNoteBack(intake, shooter),
        new DoneCycleCommand<>(
                arm.getMoveToCommand(Position.PODIUM, angleOffset)
                    .alongWith(rotateCommand)
                    .alongWith(shooter.shootSubwooferCommand()),
                true)
            // wait until the arm & shooter are stable, and the robot is pointing to the speaker
            .withDoneCycles(
                DoneCycleMachine.supplierWithMinCycles(
                    () ->
                        (arm.isStable(false) && shooter.isStable(false, false))
                            && swerve.isYawStable(rotateCommand.getTargetAngle(), 0.1),
                    5))
            // run the intake rollers to push the note further in
            .andThen(intake.intakeCommand(RollerState.FORWARD)));
  }

  public static Command getTouchAprilTagCommand(
      Supplier<Integer> tag,
      double xOffset,
      double yOffset,
      Rotation2d angleOffset,
      SwerveDrive swerve) {
    return SmartDriveToPosition.create(
        swerve,
        new PathConstraints(
            Constants.DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED / 3,
            Constants.DriveTrainConstants.SWERVE_DRIVE_MAX_ACCEL / 3,
            Constants.DriveTrainConstants.SWERVE_ROTATION_MAX_SPEED / 3,
            Constants.DriveTrainConstants.SWERVE_ROTATION_MAX_ACCEL / 3),
        () ->
            Constants2024.FieldConstants.getInstance()
                .aprilTags
                .getTagPose(tag.get())
                .orElseThrow()
                .toPose2d()
                .transformBy(
                    new Transform2d(
                        Constants2024.RobotConstants.X_SIZE / 2 + 0.02 + xOffset,
                        yOffset,
                        angleOffset.plus((new Rotation2d(Math.PI))))),
        true);
  }
}
