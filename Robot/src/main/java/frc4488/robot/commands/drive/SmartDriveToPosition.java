package frc4488.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc4488.lib.commands.CommandComposer;
import frc4488.lib.commands.LogCommand;
import frc4488.lib.geometry.PoseUtil;
import frc4488.robot.Robot;
import frc4488.robot.subsystems.drive.SwerveDrive;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Similar to {@link SwerveDriveToPosition}, but uses the "smarter" Path Planner library. Also
 * allows for two-stage movement, where the first stage is fast and the second stage is slower
 * (enable with the <code>safely</code> flag)
 */
public class SmartDriveToPosition extends CommandComposer<Command> {

  public static Command create(
      SwerveDrive swerve, PathConstraints constraints, Supplier<Pose2d> end, boolean safely) {
    return Commands.defer(
        () -> new SmartDriveToPosition(swerve, constraints, end.get(), safely),
        Set.of(swerve.driveRequirement, swerve.rotationRequirement));
  }

  private static Command createPathPlannerPath(
      SwerveDrive swerve,
      PathConstraints constraints,
      Pose2d start,
      Pose2d end,
      double endVelocity) {
    // Determine points along the path
    Rotation2d angle = end.getTranslation().minus(start.getTranslation()).getAngle();
    List<Translation2d> waypoints =
        PathPlannerPath.bezierFromPoses(
            new Pose2d(start.getTranslation(), angle), new Pose2d(end.getTranslation(), angle));

    // Print out path for debugging
    swerve
        .getField()
        .getObject("[Trajectory] SmartDriveToPosition")
        .setPoses(
            waypoints.stream().map(waypoint -> new Pose2d(waypoint, end.getRotation())).toList());
    Robot.getConsole().debugln("Traj Start: " + start);
    Robot.getConsole().debugln("Traj End: " + end);

    // Convert points to a followable path
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints, constraints, new GoalEndState(endVelocity, end.getRotation(), true));
    path.preventFlipping = true;
    return AutoBuilder.followPath(path);
  }

  private static final double RECREATE_RADIUS = 0.5;
  private static final double SAFE_RADIUS = 1.0;
  private static final PathConstraints SAFE_CONSTRAINTS = new PathConstraints(0.5, 1, 1, 1);

  private final SwerveDrive swerve;
  private final PathConstraints constraints;
  private final Pose2d end;
  private final boolean safely;
  private Pose2d prevPose;

  private SmartDriveToPosition(
      SwerveDrive swerve, PathConstraints constraints, Pose2d end, boolean safely) {
    this.swerve = swerve;
    this.end = end;
    this.constraints = constraints;
    this.safely = safely;
    setPath();
  }

  @Override
  public void execute() {
    Pose2d pose = swerve.getOdometry();
    if (prevPose != null) {
      // If the robot has suddently moved (due to a vision update), recreate the path based on the
      // new location
      double dist = pose.getTranslation().getDistance(prevPose.getTranslation());
      if (dist > RECREATE_RADIUS) {
        setPath();
        getComposedCommand().initialize();
      }
    }
    prevPose = pose;

    super.execute();
  }

  private void setPath() {
    Pose2d start = swerve.getOdometry();
    double dist = start.getTranslation().getDistance(end.getTranslation());
    if (dist < SAFE_RADIUS || !safely) {
      // The robot is near the end or the path isn't safe
      setComposedCommand(createPathPlannerPath(swerve, SAFE_CONSTRAINTS, start, end, 0));
    } else {
      // Break up the path into a faster "unsafe" segment and a slower "safer" segment
      Pose2d intermediate = PoseUtil.interpolate(start, end, (dist - SAFE_RADIUS) / dist);
      setComposedCommand(
          LogCommand.sequence(
              createPathPlannerPath(
                  swerve, constraints, start, intermediate, constraints.getMaxVelocityMps()),
              createPathPlannerPath(swerve, SAFE_CONSTRAINTS, intermediate, end, 0)));
    }
  }
}
