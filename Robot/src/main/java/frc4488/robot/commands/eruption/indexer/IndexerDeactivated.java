package frc4488.robot.commands.eruption.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc4488.robot.subsystems.eruption.Indexer;

public class IndexerDeactivated extends Command {
  private final Indexer indexer;

  public IndexerDeactivated(Indexer indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
  }

  @Override
  public void execute() {
    indexer.spinHold();
  }
}
