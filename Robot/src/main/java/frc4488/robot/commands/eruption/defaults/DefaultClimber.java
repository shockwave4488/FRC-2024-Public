package frc4488.robot.commands.eruption.defaults;

import edu.wpi.first.wpilibj2.command.Command;
import frc4488.robot.subsystems.eruption.Climber;

public class DefaultClimber extends Command {
  private final Climber climber;

  public DefaultClimber(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setClimberPosition(0);
  }
}
