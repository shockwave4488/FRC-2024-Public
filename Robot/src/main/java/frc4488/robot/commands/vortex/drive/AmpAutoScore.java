package frc4488.robot.commands.vortex.drive;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc4488.lib.commands.DoneCycleCommand;
import frc4488.lib.commands.LogCommand;
import frc4488.lib.controlsystems.DoneCycleMachine;
import frc4488.lib.sensors.vision.VisionCameras.AprilTagCamera;
import frc4488.robot.commands.drive.StandardDrive;
import frc4488.robot.commands.vortex.shooter.Shoot;
import frc4488.robot.constants.Constants2024;
import frc4488.robot.subsystems.drive.SwerveDrive;
import frc4488.robot.subsystems.vortex.Arm;
import frc4488.robot.subsystems.vortex.Arm.Position;
import frc4488.robot.subsystems.vortex.Intake;
import frc4488.robot.subsystems.vortex.Shooter;

public class AmpAutoScore {

  public static Command from(
      SwerveDrive swerve, AprilTagCamera camera, Arm arm, Shooter shooter, Intake intake) {
    return LogCommand.sequence(
            LogCommand.sequence(
                new DoneCycleCommand<>(arm.getMoveToCommand(Position.AMP), true)
                    // wait until the arm is stable
                    .withDoneCycles(
                        DoneCycleMachine.supplierWithMinCycles(() -> (arm.isStable(true)), 3)),
                Shoot.getTouchAprilTagCommand(
                    () -> Constants2024.FieldConstants.getInstance().ampTag,
                    0.05,
                    0,
                    new Rotation2d(Math.PI),
                    swerve)),
            LogCommand.parallel(
                Shoot.ampShot(shooter, arm, intake),
                new WaitCommand(0.8)
                    .andThen(
                        new StandardDrive(
                            swerve,
                            1,
                            () ->
                                new Pair<Double, Double>(
                                    0.0,
                                    (DriverStation.getAlliance().orElse(Alliance.Blue)
                                            == Alliance.Blue)
                                        ? -0.1
                                        : 0.1),
                            () -> false))))
        .withName("ampAutoScore");
  }
}
