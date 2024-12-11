package frc4488.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc4488.lib.geometry.PoseUtil;
import frc4488.lib.misc.Timed;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;

/**
 * Combines modules, gyro, and vision to determine the location of the robot. This works by
 * averaging the vision information after using the modules and gyro to shift it forward in time
 */
public class PoseEstimator {

  private record EncoderEstimate(
      SwerveDriveKinematics kinematics,
      Pose2d poseMeters,
      Rotation2d gyroAngle,
      SwerveDriveWheelPositions wheelPositions)
      implements Interpolatable<EncoderEstimate> {
    @Override
    public EncoderEstimate interpolate(EncoderEstimate endValue, double t) {
      return (t < 1 ? this : endValue);
    }
  }

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;
  private final double historyLength;
  private final int historyLengthCap;
  private final int historyLengthMin;
  private final double maxPoseVariancePer;
  private final TimeInterpolatableBuffer<EncoderEstimate> encoderHistory;
  private final Deque<Timed<Pose2d>> visionHistory;
  private Rotation2d lastGyroAngle;
  private SwerveModulePosition[] lastModulePositions;
  private Rotation2d hybridOdometryGyroOffset;
  private SwerveDriveOdometry hybridOdometry;
  private Timed<Pose2d> cachedPose;

  /**
   * A longer history length will allow for more vision updates to be averaged, but requires
   * trusting the encoders for longer too <br>
   * <br>
   * Use history length cap to limit the number of vision updates, to prevent processing so many
   * that the robot starts to lag <br>
   * <br>
   * The max pose variance prevents data that is all over the place from causing problems. This is
   * per each vision estimate, so when there are enough vision estimates the average should be
   * stable and the updates will be accepted. To reduce processing, once the number of vision
   * estimates is over half the cap, the data is assumed to be good
   *
   * @param kinematics
   * @param gyroAngle
   * @param modulePositions
   * @param initialPoseMeters
   * @param historyLength Length of history over time (seconds)
   * @param historyLengthCap Max number of vision estimates for performance
   * @param historyLengthMin Min number of vision estimates to trust the avg
   * @param maxPoseVariancePer The max variance per pose to still trust vision
   */
  public PoseEstimator(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPoseMeters,
      double historyLength,
      int historyLengthCap,
      int historyLengthMin,
      double maxPoseVariancePer) {
    this.kinematics = kinematics;
    this.odometry =
        new SwerveDriveOdometry(kinematics, gyroAngle, modulePositions, initialPoseMeters);
    this.historyLength = historyLength;
    this.historyLengthCap = historyLengthCap;
    this.historyLengthMin = historyLengthMin;
    this.maxPoseVariancePer = maxPoseVariancePer;
    this.encoderHistory = TimeInterpolatableBuffer.createBuffer(historyLength);
    this.visionHistory = new ArrayDeque<>();
    this.hybridOdometry = null;
    this.cachedPose = new Timed<>(0, initialPoseMeters);
  }

  public void updateBase(Rotation2d gyroAngle, Timed<SwerveModulePosition[]> modulePositions) {
    SwerveDriveWheelPositions wheelPositions =
        new SwerveDriveWheelPositions(modulePositions.value());
    odometry.update(gyroAngle, wheelPositions);
    lastGyroAngle = gyroAngle;
    lastModulePositions = modulePositions.value();
    if (hybridOdometry != null) {
      hybridOdometry.update(gyroAngle.plus(hybridOdometryGyroOffset), wheelPositions);
    }
    encoderHistory.addSample(
        modulePositions.time(),
        new EncoderEstimate(kinematics, odometry.getPoseMeters(), gyroAngle, wheelPositions));
  }

  public void updateVision(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    visionHistory.add(new Timed<>(timestampSeconds, visionRobotPoseMeters));
    cleanHistory();
  }

  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
    odometry.resetPosition(gyroAngle, modulePositions, poseMeters);
    visionHistory.clear();
    lastGyroAngle = gyroAngle;
    lastModulePositions = modulePositions;
    hybridOdometryGyroOffset = null;
    hybridOdometry = null;
    cachedPose = new Timed<>(Timer.getFPGATimestamp(), poseMeters);
  }

  public void cleanHistory() {
    if (visionHistory.isEmpty()) {
      return;
    }
    if (visionHistory.peekLast().time() < Timer.getFPGATimestamp() - historyLength / 2) {
      // Latest vision information is over half of the history old; average is going to become bad
      visionHistory.clear();
      return;
    }
    double minTimestamp = Timer.getFPGATimestamp() - historyLength;
    while (!visionHistory.isEmpty() && visionHistory.peek().time() < minTimestamp
        || visionHistory.size() > historyLengthCap) {
      visionHistory.remove();
    }
  }

  public Pose2d recalculatePose() {
    cleanHistory();

    // Shift the visionHistory forward in time using encoder data
    Pose2d encoderPose = odometry.getPoseMeters();
    List<Pose2d> visionEstimates = new ArrayList<>();
    for (Timed<Pose2d> oldEstimate : visionHistory) {
      getEncoderPose(oldEstimate.time())
          .ifPresent(
              oldEncoderPose ->
                  visionEstimates.add(oldEstimate.value().plus(encoderPose.minus(oldEncoderPose))));
    }

    boolean visionReliable = false;
    Optional<Pose2d> visionPosesAvg = PoseUtil.avgPoses(visionEstimates, encoderPose::getRotation);
    if (visionEstimates.size() >= historyLengthMin && visionPosesAvg.isPresent()) {
      if (visionEstimates.size() >= historyLengthCap / 2) {
        // Assume vision is reliable when there are a lot of estimates for performance
        visionReliable = true;
      } else {
        visionReliable =
            (PoseEstimationUtil.calcPoseVariance(visionPosesAvg.get(), visionEstimates)
                    / visionEstimates.size()
                <= maxPoseVariancePer);
      }
    }

    Pose2d newPose;
    if (!visionReliable) {
      // No vision information available; create a new odometry to track the encoder-only position
      // relative to the last vision-based pose
      if (lastGyroAngle == null) {
        return cachedPose.value();
      }
      if (hybridOdometry == null) {
        hybridOdometryGyroOffset = cachedPose.value().getRotation().minus(lastGyroAngle);
        hybridOdometry =
            new SwerveDriveOdometry(
                kinematics,
                cachedPose.value().getRotation(),
                lastModulePositions,
                cachedPose.value());
      }
      newPose = hybridOdometry.getPoseMeters();
    } else {
      hybridOdometryGyroOffset = null;
      hybridOdometry = null;
      newPose = visionPosesAvg.get();
    }

    cachedPose = new Timed<>(Timer.getFPGATimestamp(), newPose);
    return cachedPose.value();
  }

  public Pose2d getCachedPose() {
    return cachedPose.value();
  }

  private Optional<Pose2d> getEncoderPose(double timestamp) {
    return encoderHistory.getSample(timestamp).map(EncoderEstimate::poseMeters);
  }
}
