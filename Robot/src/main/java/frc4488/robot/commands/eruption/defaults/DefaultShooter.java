package frc4488.robot.commands.eruption.defaults;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4488.lib.sensors.vision.VisionCameras.TargetCamera;
import frc4488.robot.commands.eruption.shooter.SpinFlywheel;
import frc4488.robot.subsystems.eruption.Shooter;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class DefaultShooter extends SequentialCommandGroup {
  private static final double COAST_RPM = 0;
  private static final boolean ESTIMATE = true; // inaccurate but could be better than nothing

  public DefaultShooter(
      Shooter shooter,
      TargetCamera limelight,
      BooleanSupplier flywheelBeambreakState,
      Supplier<Pose2d> pose) {
    addRequirements(shooter);

    addCommands(
        new InstantCommand(() -> shooter.setRPM(COAST_RPM)),
        new SpinFlywheel(shooter, limelight, flywheelBeambreakState, pose, ESTIMATE, false));
    // Even though the above appears that it will spin the shooter if we see the target, it won't,
    // it never actuals tells the shooter to spin because interpolateShooter is false.
  }
}
