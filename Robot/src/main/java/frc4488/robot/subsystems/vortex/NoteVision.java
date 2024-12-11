package frc4488.robot.subsystems.vortex;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc4488.lib.logging.LeveledSmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class NoteVision {
  private PhotonCamera camera;
  private PhotonPipelineResult result;
  private static final double CAMERA_HEIGHT_METERS = 0.44;
  private static final double TARGET_HEIGHT_METERS = 0.03;
  private static final double CAMERA_PITCH_RADIANS = -0.525;

  public NoteVision(String cameraName, int nnPipelineIdx) {
    camera = new PhotonCamera(cameraName);
    camera.setPipelineIndex(nnPipelineIdx);
    result = null;
  }

  public boolean hasTargets() {
    result = camera.getLatestResult();
    return result.hasTargets();
  }

  public double getDistanceToTarget() {
    return hasTargets()
        ? PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(result.getBestTarget().getPitch()))
        : -1;
  }

  public double getYawToTarget() {
    return hasTargets() ? result.getBestTarget().getYaw() : 0;
  }

  public double getPipelineLatency() {
    return result.getLatencyMillis() / 1000.0;
  }

  public Rotation2d nnGetAngleToTarget(Pose2d currentPose) {
    // Get the current pose (x, y, yaw)
    double robotYaw = 0;

    // Look if we have a target in sight
    if (hasTargets()) {
      robotYaw = currentPose.getRotation().getDegrees() - getYawToTarget();
      LeveledSmartDashboard.INFO.putNumber("nnGetNote Target Yaw", robotYaw);
      LeveledSmartDashboard.INFO.putNumber("nnGetNote Yaw to target", getYawToTarget());
    } else {
      robotYaw = currentPose.getRotation().getDegrees();
    }
    LeveledSmartDashboard.INFO.putBoolean("nnVision.hasTargets", hasTargets());
    return Rotation2d.fromDegrees(robotYaw);
  }
}
