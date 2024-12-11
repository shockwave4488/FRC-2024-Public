package frc4488.robot.commands.eruption.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc4488.lib.commands.DoneCycleCommand;
import frc4488.lib.commands.LogCommand;
import frc4488.lib.controlsystems.DoneCycleMachine;
import frc4488.lib.sensors.gyro.IGyro;
import frc4488.lib.sensors.vision.VisionCameras.TargetCamera;
import frc4488.robot.commands.drive.HeadingRotation;
import frc4488.robot.commands.drive.RotateTowardsPosition;
import frc4488.robot.commands.eruption.drive.VisionAlignToTarget;
import frc4488.robot.commands.eruption.indexer.IndexerAdvance2;
import frc4488.robot.subsystems.SmartPCM;
import frc4488.robot.subsystems.drive.SwerveDrive;
import frc4488.robot.subsystems.eruption.Indexer;
import frc4488.robot.subsystems.eruption.Shooter;

public class CalculatedShot extends ParallelRaceGroup {
  public static final int MIN_HAS_TARGET_CYCLES = 8;

  public CalculatedShot(
      Shooter shooter,
      Indexer conveyor,
      SmartPCM compressor,
      SwerveDrive swerve,
      IGyro gyro,
      double rotationMultiplier,
      TargetCamera limelight,
      RotateTowardsPosition turnToHubCommand,
      HeadingRotation headingCommand,
      DoneCycleCommand<VisionAlignToTarget> alignToTarget) {

    DoneCycleCommand<?> spinFlywheel =
        new DoneCycleCommand<>(
                new SpinFlywheel(
                    shooter,
                    limelight,
                    conveyor.getIndexerStates()::getFlywheelBeamBreak,
                    swerve::getOdometry,
                    true,
                    true),
                false)
            .withDoneCycles(DoneCycleMachine.fromSupplier(shooter::hoodReady).withName("hoodReady"))
            .withDoneCycles(shooter.flywheelVelocityMachine);

    InstantCommand stopCompressor =
        new InstantCommand(() -> compressor.stopCompressor(), compressor);

    addCommands(
        LogCommand.arrayOf(
            this,
            LogCommand.parallel(spinFlywheel, stopCompressor),
            LogCommand.sequence(
                turnToHubCommand,
                headingCommand,
                LogCommand.race(
                    alignToTarget,
                    LogCommand.sequence(
                        new WaitUntilCommand(
                            () -> alignToTarget.isReady() && spinFlywheel.isReady()),
                        new IndexerAdvance2(conveyor))))));
  }
}
