package frc4488.robot.commands.drive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc4488.lib.logging.LeveledSmartDashboard;
import frc4488.lib.misc.Timed;
import frc4488.lib.sensors.vision.Limelight;
import frc4488.robot.subsystems.drive.SwerveDrive;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

public class VisionPoseUpdater extends Command {

  private static record PoseReference(Pose2d robotPose, List<Pose2d> refPoses, double timestamp) {}

  public static VisionPoseUpdater createForLimelights(
      SwerveDrive swerve, Limelight[] cameras, AprilTagFieldLayout layout) {
    return new VisionPoseUpdater(
        swerve,
        (poseHolder) -> {
          for (Limelight camera : cameras) {
            List<Integer> tags = camera.getAprilTagTargetIds();
            if (tags.isEmpty()) {
              continue;
            }
            // Convert the visible tags to the poses of those tags, to be shown on the field image
            List<Pose2d> tagPoses =
                tags.stream()
                    .map(tag -> layout.getTagPose(tag))
                    .filter(Optional::isPresent)
                    .map(pose -> pose.get().toPose2d())
                    .toList();
            // Combine the robot pose, visible tag poses, and timestamp together and add it to the
            // list of robot pose estimates
            Timed<Pose3d> robotPose = camera.getRobotPose();
            poseHolder.add(
                new PoseReference(robotPose.value().toPose2d(), tagPoses, robotPose.time()));
          }
        });
  }

  public static VisionPoseUpdater createForLimelight(
      SwerveDrive swerve, Limelight camera, AprilTagFieldLayout layout) {
    return createForLimelights(swerve, new Limelight[] {camera}, layout);
  }

  private final SwerveDrive swerve;
  private final Consumer<List<PoseReference>> poseSupplier;
  private final List<PoseReference> poseHolder;
  private final List<FieldObject2d> fieldObjects;
  private int ticksSinceUpdate;

  private VisionPoseUpdater(SwerveDrive swerve, Consumer<List<PoseReference>> poseSupplier) {
    this.swerve = swerve;
    this.poseSupplier = poseSupplier;
    this.poseHolder = new ArrayList<>();
    this.fieldObjects = new ArrayList<>();
  }

  @Override
  public void execute() {
    poseHolder.clear();
    poseSupplier.accept(poseHolder);
    poseHolder.forEach(ref -> swerve.consumeVisionEstimate(ref.robotPose(), ref.timestamp()));

    updateSmartDashboard();
    ticksSinceUpdate++;
    if (!poseHolder.isEmpty()) {
      ticksSinceUpdate = 0;
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  private void updateSmartDashboard() {
    List<Pose2d> refPoses = poseHolder.stream().flatMap(ref -> ref.refPoses().stream()).toList();
    for (int i = 0; i < refPoses.size(); i++) {
      FieldObject2d fieldObj = swerve.getField().getObject("Reference #" + i);
      if (fieldObjects.size() == i) {
        fieldObjects.add(fieldObj);
      }
      fieldObj.setPose(refPoses.get(i));
    }
    fieldObjects
        .subList(refPoses.size(), fieldObjects.size())
        .removeIf(
            fieldObj -> {
              // Move the object to off-screen, since closing it prevents it from re-appearing later
              fieldObj.setPose(44, 88, new Rotation2d());
              return true;
            });
    Pose2d robotPose = swerve.getOdometry();
    LeveledSmartDashboard.INFO.putNumber("TicksSinceVisionUpdate", ticksSinceUpdate);
    LeveledSmartDashboard.INFO.putNumber("RobotX", robotPose.getX());
    LeveledSmartDashboard.INFO.putNumber("RobotY", robotPose.getY());
    LeveledSmartDashboard.INFO.putNumber("RobotYaw (deg)", robotPose.getRotation().getDegrees());
    LeveledSmartDashboard.INFO.putNumber("GyroYaw (deg)", swerve.getGyroYaw().getDegrees());
  }
}
