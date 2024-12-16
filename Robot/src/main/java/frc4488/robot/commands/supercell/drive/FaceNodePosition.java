package frc4488.robot.commands.supercell.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import frc4488.lib.sensors.gyro.IGyro;
import frc4488.robot.commands.drive.RotateTowardsPosition;
import frc4488.robot.constants.Constants2023.Node;
import frc4488.robot.subsystems.drive.SwerveDrive;
import java.util.function.Supplier;

public class FaceNodePosition extends RotateTowardsPosition {
  public FaceNodePosition(
      SwerveDrive swerve, IGyro gyro, ProfiledPIDController thetaController, Supplier<Node> node) {
    super(swerve, gyro, thetaController, () -> node.get().getPosition());
  }
}
