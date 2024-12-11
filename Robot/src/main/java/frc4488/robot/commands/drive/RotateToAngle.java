package frc4488.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc4488.lib.controlsystems.DoneCycleMachine;
import frc4488.lib.controlsystems.DoneCycleMachineConditions.NumberCondition;
import frc4488.lib.controlsystems.SupplierCache;
import frc4488.lib.sensors.gyro.IGyro;
import frc4488.robot.subsystems.drive.SwerveDrive;
import java.util.function.Supplier;

public class RotateToAngle extends Command {
  private static final Rotation2d DEFAULT_ANGLE_RANGE = Rotation2d.fromDegrees(4);

  private final SwerveDrive swerve;
  private final IGyro gyro;
  private ProfiledPIDController thetaController;
  private final boolean updateContinuously;

  public final SupplierCache<Rotation2d> targetAngle;

  public RotateToAngle(
      SwerveDrive swerve,
      IGyro gyro,
      ProfiledPIDController thetaController,
      Supplier<Rotation2d> desiredAngle) {
    this(swerve, gyro, thetaController, desiredAngle, false);
  }

  public RotateToAngle(
      SwerveDrive swerve,
      IGyro gyro,
      ProfiledPIDController thetaController,
      Supplier<Rotation2d> desiredAngle,
      boolean updateContinuously) {
    this.swerve = swerve;
    this.gyro = gyro;
    this.thetaController = thetaController;
    this.updateContinuously = updateContinuously;
    targetAngle = new SupplierCache<>(desiredAngle);
    addRequirements(swerve.rotationRequirement);
  }

  public DoneCycleMachine<NumberCondition> getYawDoneCycleMachine(
      int minDoneCycles, Rotation2d angleDoneRange) {
    return DoneCycleMachine.withMinCycles(
            new NumberCondition(
                () -> swerve.getOdometry().getRotation().getRadians(),
                () -> targetAngle.current().getRadians()),
            minDoneCycles)
        .configureCondition(
            condition -> condition.doneRange.setTolerance(angleDoneRange.getRadians()));
  }

  public DoneCycleMachine<NumberCondition> getYawDoneCycleMachine(int minDoneCycles) {
    return getYawDoneCycleMachine(minDoneCycles, DEFAULT_ANGLE_RANGE);
  }

  @Override
  public void initialize() {
    thetaController.reset(
        swerve.getOdometry().getRotation().getRadians(), gyro.getYawRateRadiansPerSec());
    targetAngle.update();
  }

  @Override
  public void execute() {
    if (updateContinuously) {
      targetAngle.update();
    }
    double currentAngle = swerve.getOdometry().getRotation().getRadians();
    double rotSpeed = thetaController.calculate(currentAngle, targetAngle.current().getRadians());
    swerve.setRotationSpeed(rotSpeed);
  }
}
