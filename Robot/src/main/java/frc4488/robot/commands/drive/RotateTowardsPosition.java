package frc4488.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc4488.lib.commands.CommandComposer;
import frc4488.lib.controlsystems.SupplierCache;
import frc4488.lib.sensors.gyro.IGyro;
import frc4488.robot.subsystems.drive.SwerveDrive;
import java.util.function.Supplier;

public class RotateTowardsPosition extends CommandComposer<RotateToAngle> {
  private static Rotation2d getAngleToPosition(SwerveDrive swerve, Translation2d position) {
    return position.minus(swerve.getOdometry().getTranslation()).getAngle();
  }

  public final SupplierCache<Translation2d> targetPos;
  private Rotation2d targetAngle;
  private boolean keepUpdating;

  public Rotation2d getTargetAngle() {
    return targetAngle;
  }

  public RotateTowardsPosition(
      SwerveDrive swerve,
      IGyro gyro,
      ProfiledPIDController thetaController,
      Supplier<Translation2d> position,
      Rotation2d rotationOffset) {
    targetPos = new SupplierCache<>(position);
    setComposedCommand(
        new RotateToAngle(
            swerve,
            gyro,
            thetaController,
            () -> {
              targetAngle = getAngleToPosition(swerve, targetPos.current()).plus(rotationOffset);
              return targetAngle;
            }));
  }

  public RotateTowardsPosition(
      SwerveDrive swerve,
      IGyro gyro,
      ProfiledPIDController thetaController,
      Supplier<Translation2d> position) {
    this(swerve, gyro, thetaController, position, new Rotation2d());
  }

  public RotateTowardsPosition keepUpdating(boolean keepUpdating) {
    this.keepUpdating = keepUpdating;
    return this;
  }

  @Override
  public void initialize() {
    targetPos.update();
    super.initialize();
  }

  @Override
  public void execute() {
    if (keepUpdating) {
      targetPos.update();
    }
    composedCommand.targetAngle.update();
    super.execute();
  }
}
