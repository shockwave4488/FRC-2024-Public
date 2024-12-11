package frc4488.robot.commands.supercell.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc4488.lib.commands.LogCommand;
import frc4488.lib.sensors.gyro.IGyro;
import frc4488.robot.commands.drive.LockedSwerveDrive;
import frc4488.robot.commands.drive.LockedSwerveDrive.LockedMode;
import frc4488.robot.constants.Constants2023.RobotConstants.AutonomousConstants;
import frc4488.robot.subsystems.drive.SwerveDrive;
import java.util.function.DoubleSupplier;

public class BalanceOnChargeStation {
  /**
   * Gets the tilt of the robot on an inclined plane by combining pitch and roll information.
   * Negative when slanted upwards, regardless of yaw.
   *
   * @param zeroRollAngle Yaw at which driving up the inclined plane will result in no roll.
   */
  public static Rotation2d getTilt(
      IGyro gyro, Rotation2d zeroRollAngle, Rotation2d gyroAdjustment) {
    Rotation2d angleOffset = gyro.getYaw().plus(gyroAdjustment).minus(zeroRollAngle);
    return gyro.getPitch()
        .times(angleOffset.getCos())
        .plus(gyro.getRoll().times(angleOffset.getSin()));
  }

  public static Rotation2d getTilt(IGyro gyro, Rotation2d gyroAdjustment) {
    return getTilt(
        gyro, new Rotation2d(), gyroAdjustment); // Charge station is parallel with the y-axis
  }

  private static boolean isBalanced(IGyro gyro, Rotation2d gyroAdjustment) {
    return Math.abs(getTilt(gyro, gyroAdjustment).getRadians()) < BALANCED_RANGE.getRadians();
  }

  public static Command driveAtHorizontalSpeed(SwerveDrive swerve, DoubleSupplier metersPerSecond) {
    return Commands.run(
        () -> swerve.setTranslationSpeeds(metersPerSecond.getAsDouble(), 0, true),
        swerve.driveRequirement);
  }

  public static int getDriveDirection(IGyro gyro, Rotation2d gyroAdjustment) {
    // By not passing in 180 degrees even if on the other side of the charge station, we can use
    // the tilt (negative next to the driver station, positive on the other side) to decide
    // which way to drive.
    return (int) -Math.signum(getTilt(gyro, gyroAdjustment).getDegrees());
  }

  private static final Rotation2d BALANCED_RANGE = Rotation2d.fromDegrees(2.5);

  public static Command create(SwerveDrive swerve, IGyro gyro) {
    return LogCommand.repeat(
        LogCommand.sequence(
            LogCommand.endWhen(
                getTiltPIDCommand(swerve, gyro),
                () -> isBalanced(gyro, swerve.getGyroAdjustment())),
            LogCommand.endWhen(
                new LockedSwerveDrive(swerve, LockedMode.XShape),
                () -> !isBalanced(gyro, swerve.getGyroAdjustment()))));
  }

  private static Command getTiltPIDCommand(SwerveDrive swerve, IGyro gyro) {
    return new PIDCommand(
        new PIDController(
            AutonomousConstants.AUTO_BALANCE_P,
            AutonomousConstants.AUTO_BALANCE_I,
            AutonomousConstants.AUTO_BALANCE_D),
        () -> getTilt(gyro, swerve.getGyroAdjustment()).getDegrees(),
        0,
        value -> swerve.setTranslationSpeeds(value, 0, true),
        swerve.driveRequirement);
  }
}
