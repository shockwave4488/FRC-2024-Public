package frc4488.robot.commands.eruption.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc4488.robot.commands.eruption.indexer.IndexerAdvanceForFenderShot;
import frc4488.robot.subsystems.SmartPCM;
import frc4488.robot.subsystems.eruption.Indexer;
import frc4488.robot.subsystems.eruption.Shooter;

public class LaunchFenderShot extends SequentialCommandGroup {
  public LaunchFenderShot(
      Shooter shooter,
      Indexer conveyor,
      SmartPCM compressor,
      double setRPM,
      double desiredHoodPos) {

    addRequirements(shooter, conveyor, compressor);

    addCommands(
        new InstantCommand(() -> compressor.stopCompressor()),
        new InstantCommand(() -> shooter.setRPM(setRPM)),
        new InstantCommand(() -> shooter.setHoodPosition(desiredHoodPos)),
        new WaitUntilCommand(shooter::isReady),
        new IndexerAdvanceForFenderShot(conveyor)
            .alongWith(new InstantCommand(() -> shooter.setHoodPosition(desiredHoodPos))),
        new InstantCommand(() -> shooter.stop()));
  }
}
