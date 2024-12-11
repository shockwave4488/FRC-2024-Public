package frc4488.robot.commands.eruption.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc4488.robot.subsystems.eruption.Indexer;
import frc4488.robot.subsystems.eruption.Intake;
import frc4488.robot.subsystems.eruption.Intake.RollerState;
import frc4488.robot.subsystems.eruption.Shooter;

public class PurgeAllIntake extends Command {
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;

  public PurgeAllIntake(Intake intake, Indexer indexer, Shooter shooter) {
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    addRequirements(intake, indexer, shooter);
  }

  @Override
  public void execute() {
    intake.intakeOut();
    intake.setBottomRollerState(RollerState.Off);
    intake.setTopRollerState(RollerState.ReverseMedium);
    indexer.spinBackwards();
    shooter.stop();
  }
}
