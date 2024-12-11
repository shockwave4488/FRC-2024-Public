package frc4488.robot.commands.eruption.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc4488.robot.subsystems.eruption.Intake;
import frc4488.robot.subsystems.eruption.Intake.RollerState;

public class ColorlessIntake extends Command {
  // Backup intake if color intake fails

  private final Intake intake;

  public ColorlessIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.intakeOut();
  }

  @Override
  public void execute() {
    intake.setTopRollerState(RollerState.ForwardFull);
    intake.setBottomRollerState(RollerState.ForwardFull);
  }
}
