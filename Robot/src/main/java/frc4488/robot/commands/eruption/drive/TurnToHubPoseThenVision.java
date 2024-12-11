package frc4488.robot.commands.eruption.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc4488.lib.commands.DoneCycleCommand;
import frc4488.robot.commands.drive.RotateTowardsPosition;

public class TurnToHubPoseThenVision extends SequentialCommandGroup {
  public TurnToHubPoseThenVision(
      RotateTowardsPosition hubCentricSwerveDriveCommand,
      DoneCycleCommand<VisionAlignToTarget> visionAlignToTargetCommand) {
    super(
        new DoneCycleCommand<>(hubCentricSwerveDriveCommand, true)
            .withDoneCycles(cmd -> cmd.getComposedCommand().getYawDoneCycleMachine(5)),
        visionAlignToTargetCommand);
  }
}
