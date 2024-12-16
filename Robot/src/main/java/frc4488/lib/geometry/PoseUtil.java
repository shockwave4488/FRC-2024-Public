package frc4488.lib.geometry;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc4488.lib.sensors.vision.VisionTargets.AprilTagTarget;
import frc4488.robot.constants.Constants;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class PoseUtil {
  private PoseUtil() {}

  public interface PoseSupplier extends Supplier<Pose2d> {
    static PoseSupplier fromParts(Supplier<Translation2d> position, Supplier<Rotation2d> rotation) {
      return () -> new Pose2d(position.get(), rotation.get());
    }

    default PoseSupplier withRotation(Supplier<Rotation2d> rotation) {
      return fromParts(() -> get().getTranslation(), rotation);
    }

    default PoseSupplier withTranslation(Supplier<Translation2d> position) {
      return fromParts(position, () -> get().getRotation());
    }

    default PoseSupplier map(Function<Pose2d, Pose2d> mapper) {
      return () -> mapper.apply(get());
    }

    default PoseSupplier withAdjustment(Supplier<Transform2d> adjustment) {
      return () -> get().transformBy(adjustment.get());
    }
  }

  public static Function<Pose2d, Pose2d> adjustmentApplier(Supplier<Transform2d> adjustment) {
    return pose -> pose.transformBy(adjustment.get());
  }

  public static class DualPose extends Pair<Pose2d, Pose2d> {
    public DualPose(Pose2d pose1, Pose2d pose2) {
      super(pose1, pose2);
    }

    /**
     * @return the angle where the first pose points toward the second pose
     */
    public Rotation2d faceAngle() {
      return getSecond().getTranslation().minus(getFirst().getTranslation()).getAngle();
    }

    /**
     * @return the angle where the second pose points toward the first pose
     */
    public Rotation2d faceReverseAngle() {
      return faceAngle().rotateBy(Rotation2d.fromRotations(0.5));
    }

    public <T> T getFromFirst(Function<Pose2d, T> poseFunction) {
      return poseFunction.apply(getFirst());
    }

    public <T> T getFromSecond(Function<Pose2d, T> poseFunction) {
      return poseFunction.apply(getSecond());
    }
  }

  public static Pose2d addTranslation(Pose2d current, Translation2d addition) {
    Rotation2d currentRotation = current.getRotation();
    return new Pose2d(current.getTranslation().plus(addition), currentRotation);
  }

  public static PoseSupplier aprilTagPoseSupplier(Supplier<AprilTag> tag) {
    return () -> tag.get().pose.toPose2d();
  }

  public static PoseSupplier aprilTagPoseSupplier(
      Supplier<AprilTagTarget> target,
      Supplier<Pose2d> robotPose,
      Transform3d robotCenterToCamera) {
    return aprilTagPoseSupplier(
        () -> getAprilTagFromTarget(target.get(), robotPose.get(), robotCenterToCamera));
  }

  public static AprilTag getAprilTagFromTarget(
      AprilTagTarget target, Pose2d robotPose, Transform3d robotCenterToCamera) {
    return target.toAprilTag(robotPose, robotCenterToCamera);
  }

  public static Rotation2d lerpRotationShortest(
      Rotation2d startValue, Rotation2d endValue, double t) {
    return startValue.interpolate(endValue, t);
  }

  public static Rotation2d lerpRotationLongest(
      Rotation2d startValue, Rotation2d endValue, double t) {
    double diff = endValue.minus(startValue).getRadians();
    return Rotation2d.fromRadians(
        startValue.getRadians()
            + (diff - Math.copySign(Constants.TAU, diff)) * MathUtil.clamp(t, 0, 1));
  }

  public static Rotation2d lerpRotationClockwise(
      Rotation2d startValue, Rotation2d endValue, double t) {
    return startValue.plus(
        new Rotation2d(
                MathUtil.inputModulus(
                    endValue.getRadians() - startValue.getRadians(), 0, Constants.TAU))
            .times(MathUtil.clamp(t, 0, 1)));
  }

  public static Rotation2d lerpRotationCounterclockwise(
      Rotation2d startValue, Rotation2d endValue, double t) {
    return startValue.plus(
        new Rotation2d(
                -MathUtil.inputModulus(
                    startValue.getRadians() - endValue.getRadians(), 0, Constants.TAU))
            .times(MathUtil.clamp(t, 0, 1)));
  }

  public static Optional<Rotation2d> avgRotations(Collection<Rotation2d> rotations) {
    if (rotations.isEmpty()) {
      return Optional.empty();
    }
    double xSum = 0;
    double ySum = 0;
    for (Rotation2d rotation : rotations) {
      xSum += rotation.getCos();
      ySum += rotation.getSin();
    }
    if (xSum == 0 && ySum == 0) {
      return Optional.empty();
    }
    return Optional.of(
        Rotation2d.fromRadians(Math.atan2(ySum / rotations.size(), xSum / rotations.size())));
  }

  public static Optional<Pose2d> avgPoses(
      Collection<Pose2d> poses, Supplier<Rotation2d> rotationFallback) {
    if (poses.isEmpty()) {
      return Optional.empty();
    }
    double xSum = 0;
    double ySum = 0;
    List<Rotation2d> rotations = new ArrayList<>();
    for (Pose2d pose : poses) {
      xSum += pose.getX();
      ySum += pose.getY();
      rotations.add(pose.getRotation());
    }
    Rotation2d thetaAvg = avgRotations(rotations).orElseGet(rotationFallback);
    return Optional.of(new Pose2d(xSum / poses.size(), ySum / poses.size(), thetaAvg));
  }

  /**
   * This is not the same thing as {@link Pose2d#interpolate(Pose2d, double)}. That method uses
   * {@link Twist2d}s to interpolate, while this interpolates the x, y, and rotation directly
   */
  public static Pose2d interpolate(Pose2d start, Pose2d end, double progress) {
    return new Pose2d(
        MathUtil.interpolate(start.getX(), end.getX(), progress),
        MathUtil.interpolate(start.getY(), end.getY(), progress),
        start.getRotation().interpolate(end.getRotation(), progress));
  }
}
