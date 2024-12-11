package frc4488.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc4488.lib.autonomous.AutoPIDControllerContainer;
import frc4488.lib.commands.CommandComposer;
import frc4488.lib.controlsystems.SupplierCache;
import frc4488.robot.Robot;
import frc4488.robot.subsystems.drive.SwerveDrive;
import java.util.Arrays;
import java.util.function.Supplier;

public class SwerveDriveToPosition extends CommandComposer<SwerveControllerCommand> {
  private static Trajectory constructTrajectory(
      Pose2d goalPose, Pose2d curPose, TrajectoryConfig config, Translation2d[] interiorWaypoints) {
    return TrajectoryGenerator.generateTrajectory(
        curPose, Arrays.asList(interiorWaypoints), goalPose, config);
  }

  private static Rotation2d getFinalTrajectoryRotation(Trajectory trajectory) {
    return trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation();
  }

  private final SwerveDrive swerve;
  private final AutoPIDControllerContainer pidControllers;
  public final SupplierCache<Trajectory> trajectory;
  private final Supplier<Rotation2d> desiredRotation;
  private boolean finalRotationMode = false;

  public SwerveDriveToPosition(
      SwerveDrive swerve,
      AutoPIDControllerContainer pidControllers,
      Supplier<Trajectory> trajectory,
      Supplier<Rotation2d> desiredRotation) {
    super(swerve.driveRequirement, swerve.rotationRequirement, swerve.modifierRequirement);
    this.swerve = swerve;
    this.pidControllers = pidControllers;
    this.trajectory = new SupplierCache<>(trajectory, this::updateTrajectory);
    this.desiredRotation = desiredRotation;
  }

  public SwerveDriveToPosition(
      SwerveDrive swerve,
      AutoPIDControllerContainer pidControllers,
      Supplier<Trajectory> trajectory) {
    this(swerve, pidControllers, trajectory, () -> getFinalTrajectoryRotation(trajectory.get()));
    finalRotationMode = true;
  }

  public SwerveDriveToPosition(
      SwerveDrive swerve,
      AutoPIDControllerContainer pidControllers,
      Supplier<TrajectoryConfig> config,
      Supplier<Pose2d> goalPose,
      Supplier<Rotation2d> desiredRotation,
      Supplier<Translation2d[]> interiorWaypoints,
      boolean reversed) {
    this(
        swerve,
        pidControllers,
        () -> {
          TrajectoryConfig configInst = config.get();
          configInst.setReversed(reversed);
          Pose2d goalPoseValue = goalPose.get();
          Robot.getConsole().debugln("AutoDrive > Traj Start: " + swerve.getOdometry());
          Robot.getConsole().debugln("AutoDrive > Traj End: " + goalPoseValue);
          Trajectory trajectory =
              constructTrajectory(
                  goalPoseValue, swerve.getOdometry(), configInst, interiorWaypoints.get());
          Robot.getConsole().debugln("AutoDrive > Traj: " + trajectory);
          swerve
              .getField()
              .getObject("[Trajectory] SwerveDriveToPosition")
              .setTrajectory(trajectory);

          return trajectory;
        },
        desiredRotation);
  }

  public SwerveDriveToPosition(
      SwerveDrive swerve,
      AutoPIDControllerContainer pidControllers,
      Supplier<TrajectoryConfig> config,
      Supplier<Pose2d> goalPose,
      Supplier<Translation2d[]> interiorWaypoints,
      boolean reversed) {
    this(
        swerve,
        pidControllers,
        config,
        goalPose,
        () -> goalPose.get().getRotation(),
        interiorWaypoints,
        reversed);
    finalRotationMode = true;
  }

  private void updateTrajectory() {
    setComposedCommand(
        new SwerveControllerCommand(
            trajectory.current(),
            swerve::getOdometry,
            swerve.getKinematics(),
            pidControllers.xPidController,
            pidControllers.yPidController,
            pidControllers.thetaPidController,
            (finalRotationMode) ? () -> desiredRotation.get() : desiredRotation,
            swerve::assignModuleStates));
    composedCommand.initialize();
  }

  @Override
  public void initialize() {
    trajectory.update();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    swerve.stop();
  }
}
