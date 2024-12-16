package frc4488.robot.commands.eruption.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc4488.lib.logging.LeveledSmartDashboard;
import frc4488.lib.sensors.gyro.IGyro;
import frc4488.robot.subsystems.drive.SwerveDrive;

/** Eruption's command for rotating to an angle during autonomous. */
public class RotateToAngleAuto extends Command {
  private final SwerveDrive swerve;
  private static final double TURN_P = 0.1;
  private static final double TURN_I = 0.001;
  private static final double TURN_D = 0;
  private PIDController turnPID = new PIDController(TURN_P, TURN_I, TURN_D);
  private double desiredAngle;
  private IGyro gyro;
  private double currentAngle;
  private static final double ANGLE_RANGE = 4;

  public boolean pidAtSetpoint() {
    return turnPID.atSetpoint();
  }

  public RotateToAngleAuto(SwerveDrive swerve, IGyro gyro, double desiredAngle) {
    this.swerve = swerve;
    addRequirements(swerve.rotationRequirement);

    this.desiredAngle = desiredAngle;
    this.gyro = gyro;

    turnPID.enableContinuousInput(-180, 180);
    turnPID.setTolerance(ANGLE_RANGE);
  }

  @Override
  public void execute() {
    currentAngle = gyro.getYaw().getDegrees();
    turnPID.setSetpoint(desiredAngle);
    double rotSpeed = turnPID.calculate(currentAngle);
    swerve.setRotationSpeed(rotSpeed);

    LeveledSmartDashboard.INFO.putNumber("Current Angle", currentAngle);
    LeveledSmartDashboard.INFO.putNumber("Desired Angle", desiredAngle);
  }
}
