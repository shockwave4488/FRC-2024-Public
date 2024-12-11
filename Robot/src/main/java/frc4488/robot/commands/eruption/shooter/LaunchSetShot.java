package frc4488.robot.commands.eruption.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc4488.robot.commands.eruption.indexer.IndexerAdvance2;
import frc4488.robot.subsystems.SmartPCM;
import frc4488.robot.subsystems.eruption.Indexer;
import frc4488.robot.subsystems.eruption.Shooter;

public class LaunchSetShot extends SequentialCommandGroup {
  public LaunchSetShot(
      Shooter shooter,
      Indexer conveyor,
      SmartPCM compressor,
      double setRPM,
      double desiredHoodPos) {

    addRequirements(shooter);
    addRequirements(conveyor);
    addRequirements(compressor);

    addCommands(
        new InstantCommand(() -> compressor.stopCompressor()),
        new InstantCommand(() -> shooter.setRPM(setRPM)),
        new InstantCommand(() -> shooter.setHoodPosition(desiredHoodPos)),
        new WaitUntilCommand(shooter::isReady),
        new IndexerAdvance2(conveyor));
  }
}
