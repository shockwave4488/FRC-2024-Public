package frc4488.robot.commands.eruption.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc4488.robot.subsystems.eruption.Intake;
import frc4488.robot.subsystems.eruption.Intake.RollerState;

public class PurgeIntake extends Command {
  private final Intake intake;

  public PurgeIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.intakeOut();
    intake.setTopRollerState(RollerState.Off);
    intake.setBottomRollerState(RollerState.ForwardMedium);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
