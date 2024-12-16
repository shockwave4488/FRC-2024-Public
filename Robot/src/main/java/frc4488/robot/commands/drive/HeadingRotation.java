package frc4488.robot.commands.drive;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc4488.lib.autonomous.AutoPIDControllerContainer;
import frc4488.lib.sensors.gyro.IGyro;
import frc4488.robot.constants.Constants;
import frc4488.robot.subsystems.drive.SwerveDrive;
import java.util.function.Supplier;

public class HeadingRotation extends Command {
  private final SwerveDrive swerve;
  private final IGyro gyro;
  private final double rotationMultiplier;
  private final boolean reverse;
  private final Supplier<Pair<Double, Double>> rotValues;
  private final PIDController headingPIDController;

  private static final double ALLOWED_HEADING_CHANGE = 0.4;

  private double currentTargetAngle;

  public HeadingRotation(
      SwerveDrive swerve,
      IGyro gyro,
      AutoPIDControllerContainer pid,
      double rotationMultiplier,
      Supplier<Pair<Double, Double>> rotValues,
      boolean reverse) {
    this.swerve = swerve;
    this.gyro = gyro;
    this.rotationMultiplier = rotationMultiplier;
    this.rotValues = rotValues;
    this.reverse = reverse;

    headingPIDController =
        new PIDController(
            pid.thetaPidController.getP(),
            pid.thetaPidController.getI(),
            pid.thetaPidController.getD());
    headingPIDController.enableContinuousInput(-Math.PI, Math.PI);

    currentTargetAngle = gyro.getYaw().getRadians();

    addRequirements(swerve.rotationRequirement);
  }

  @Override
  public void execute() {
    double desiredAngle = calcDesiredAngle(currentTargetAngle);
    double rotPower = headingPIDController.calculate(gyro.getYaw().getRadians(), desiredAngle);

    // The rotationMultiplier that's passed in is negative so the values for Math.min and max may
    // seem unintuitive.
    rotPower = Math.min(Math.max(rotationMultiplier, rotPower), -rotationMultiplier);
    swerve.setRotationSpeed(rotPower);
  }

  private double calcDesiredAngle(double defaultAngle) {
    Pair<Double, Double> rotSpeedValues = rotValues.get();
    double rotYValue = rotSpeedValues.getFirst();
    double rotXValue = rotSpeedValues.getSecond() * -1;

    if ((rotXValue == 0) && (rotYValue == 0)) {
      if (Math.abs(currentTargetAngle - gyro.getYaw().getRadians()) >= ALLOWED_HEADING_CHANGE) {
        return gyro.getYaw().getRadians();
      }
      return defaultAngle;
    }

    double theta = Math.atan2(rotYValue, rotXValue);

    theta -= (Math.PI / 2);
    if (theta < -Math.PI) {
      theta += Constants.TAU;
    }

    if (reverse) {
      // Rotate by 180 so the back of the robot points in the direction of the joystick
      theta += Math.PI;
      if (theta > Math.PI) {
        theta -= Constants.TAU;
      }
    }

    currentTargetAngle = theta;
    return theta;
  }
}
