package frc4488.robot.commands.eruption.defaults;

import edu.wpi.first.wpilibj2.command.Command;
import frc4488.robot.subsystems.eruption.Intake;

public class DefaultIntakeRetracted extends Command {
  private final Intake intake;

  public DefaultIntakeRetracted(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.onStop(intake.isSStopped());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
