package frc4488.robot.commands.eruption.defaults;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4488.robot.subsystems.eruption.Shooter;

public class DefaultDemoShooter extends SequentialCommandGroup {
  private static final double COAST_RPM = 0;

  public DefaultDemoShooter(Shooter shooter) {
    addRequirements(shooter);

    addCommands(new InstantCommand(() -> shooter.setRPM(COAST_RPM)));
  }
}
