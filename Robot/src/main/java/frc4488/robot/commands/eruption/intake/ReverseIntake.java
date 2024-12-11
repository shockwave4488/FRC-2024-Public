package frc4488.robot.commands.eruption.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc4488.robot.subsystems.eruption.Intake;
import frc4488.robot.subsystems.eruption.Intake.RollerState;

public class ReverseIntake extends Command {
  private final Intake intake;

  public ReverseIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.intakeOut();
    intake.setTopRollerState(RollerState.ReverseMedium);
    intake.setBottomRollerState(RollerState.ReverseMedium);
  }
}
