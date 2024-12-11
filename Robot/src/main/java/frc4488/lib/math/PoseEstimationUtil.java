package frc4488.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc4488.lib.sensors.vision.VisionCameras.AprilTagCamera;
import frc4488.lib.sensors.vision.VisionTargets.AprilTagTarget;
import frc4488.robot.subsystems.drive.SwerveDrive;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleUnaryOperator;

public final class PoseEstimationUtil {
  private PoseEstimationUtil() {}

  /**
   * Estimates camera range to a target using the target's elevation, and only 2D vision
   * information. This method can produce more stable results than SolvePnP when well tuned, if the
   * full 6D robot pose is not required. Note that this method requires the camera to have 0 roll
   * (not be skewed clockwise or CCW relative to the floor), and for there to exist a height
   * differential between goal and camera. The larger this differential, the more accurate the
   * distance estimate will be.
   */
  public static double calculateDistanceToTarget(
      double cameraHeightMeters,
      double targetHeightMeters,
      Rotation2d cameraPitch,
      Rotation2d targetPitch,
      Rotation2d targetYaw) {
    return (targetHeightMeters - cameraHeightMeters)
        / (cameraPitch.plus(targetPitch).getTan() * targetYaw.getCos());
  }

  /**
   * This method will calculate an estimated robot position based on a given translational distance
   * and angle from camera to a target whose position is provided, and the angle to the target. Only
   * call this method if you know your translational distance is correct.
   *
   * @param translationalDistance Translational distance from the target in meters
   * @see {@link #calculateDistanceToTarget(double, double, Rotation2d, Rotation2d, Rotation2d)}
   */
  public static Pose2d getRobotPoseFrom2d(
      Translation2d targetPos,
      double translationalDistance,
      Rotation2d currentAngle,
      Rotation2d angleToTarget,
      Transform2d robotCenterToCamera) {
    // The robot's quadrant on the field does not matter when doing the following calculations.
    Translation2d relativeTransformToCamera =
        new Translation2d(
            -angleToTarget.getCos() * translationalDistance,
            -angleToTarget.getSin() * translationalDistance);
    Translation2d estimatedCameraPos = targetPos.plus(relativeTransformToCamera);
    // Transform pose from camera to robot center
    return new Pose2d(estimatedCameraPos, currentAngle).transformBy(robotCenterToCamera.inverse());
  }

  /**
   * Returns the pose of the robot center from tranform information between the camera and a
   * reference point (such as a vision target).
   */
  public static Pose2d getRobotPoseFromTransform(
      Pose3d referencePose, Transform3d cameraToReference, Transform3d robotCenterToCamera) {
    return referencePose
        .transformBy(cameraToReference.inverse())
        .transformBy(robotCenterToCamera.inverse())
        .toPose2d();
  }

  /**
   * Returns the pose of a reference point (such as a vision target) from tranform information
   * between the camera and the reference point.
   */
  public static Pose3d getTargetPoseFromTransform(
      Pose2d robotPose, Transform3d cameraToTarget, Transform3d robotCenterToCamera) {
    return new Pose3d(robotPose).transformBy(robotCenterToCamera).transformBy(cameraToTarget);
  }

  public static Optional<? extends AprilTagTarget> getBestTarget(
      AprilTagCamera camera, Collection<Integer> ids) {
    Optional<? extends AprilTagTarget> bestTarget = camera.getBestAprilTagTarget();
    if (bestTarget.isEmpty() || ids.contains(bestTarget.get().getId())) {
      return bestTarget;
    }
    return camera.getAprilTagTargets().stream()
        .filter(target -> ids.contains(target.getId()))
        .sorted(
            (a, b) ->
                a.getCameraToTarget().getTranslation().getNorm()
                        < b.getCameraToTarget().getTranslation().getNorm()
                    ? -1
                    : 1)
        .findFirst();
  }

  /**
   * Adjust the real target so that aiming at the new target while moving will cause the shot to hit
   * the real target
   */
  public static Optional<Translation2d> accountForRobotVelocity(
      Translation2d target, SwerveDrive swerve, double shootingMps) {
    // See derivation in docs/accountForRobotVelocity_proof.png

    Translation2d robotPos = swerve.getOdometry().getTranslation();
    ChassisSpeeds chassisSpeeds = swerve.getFieldRelativeChassisSpeeds();
    Translation2d velocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    double velocityMag = velocity.getNorm();
    double dist = robotPos.getDistance(target);
    Rotation2d theta = target.minus(robotPos).getAngle().minus(velocity.getAngle());

    // Robot is moving very slowly; don't bother calculating. This also avoids possible issues when
    // velocity = 0
    if (Math.abs(velocityMag) < 0.01) {
      return Optional.of(target);
    }

    DoubleUnaryOperator f =
        (x) -> {
          return theta.getSin() * Math.cos(x) * shootingMps
              - theta.getCos() * Math.sin(x) * shootingMps
              + theta.getSin() * velocityMag;
        };

    double extreme1 = f.applyAsDouble(theta.getRadians() - Math.PI / 2);
    double extreme2 = f.applyAsDouble(theta.getRadians() + Math.PI / 2);

    double x = theta.getRadians() - Math.asin(2 * extreme2 / (extreme2 - extreme1) - 1);

    double offsetMag = dist * theta.getSin() * Math.tan(x - Math.PI / 2) + dist * theta.getCos();
    if (Double.isNaN(offsetMag)) { // No solutions exist; robot moving too fast in wrong direction
      return Optional.empty();
    }
    Translation2d offset = new Translation2d(-offsetMag, velocity.getAngle());
    Translation2d newTarget = target.plus(offset);
    Translation2d newTargetMinusRobot = newTarget.minus(robotPos);
    if (Math.abs(
            newTargetMinusRobot
                .div(newTargetMinusRobot.getNorm())
                .times(shootingMps)
                .plus(velocity)
                .getAngle()
                .minus(target.minus(robotPos).getAngle())
                .getRadians())
        > 0.001) {
      // Solution doesn't make sense; robot moving too fast in wrong direction
      return Optional.empty();
    }
    return Optional.of(newTarget);
  }

  /**
   * Adjust the real target so that aiming at the new target while moving will cause the shot to hit
   * the real target
   */
  public static Optional<Translation3d> accountForRobotVelocity(
      Translation3d target, SwerveDrive swerve, double shootingMps) {
    return accountForRobotVelocity(target.toTranslation2d(), swerve, shootingMps)
        .map(newTarget -> new Translation3d(newTarget.getX(), newTarget.getY(), target.getZ()));
  }

  /** <code>avg</code> must equal the average of the supplied <code>poses</code> */
  public static double calcPoseVariance(Pose2d avg, List<Pose2d> poses) {
    List<Double> distances = new ArrayList<>();
    List<Double> rotations = new ArrayList<>();
    for (Pose2d pose : poses) {
      distances.add(pose.getTranslation().getDistance(avg.getTranslation()));
      rotations.add(pose.getRotation().getRadians());
    }
    return MathUtil.calcVariance(0, distances)
        + MathUtil.calcVariance(avg.getRotation().getRadians(), rotations, true);
  }
}
