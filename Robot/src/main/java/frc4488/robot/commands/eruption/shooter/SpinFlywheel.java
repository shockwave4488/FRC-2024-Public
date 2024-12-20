package frc4488.robot.commands.eruption.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc4488.lib.flowcontrol.EdgeTrigger;
import frc4488.lib.sensors.vision.VisionCamera.CameraPositionConstants;
import frc4488.lib.sensors.vision.VisionCameras.TargetCamera;
import frc4488.robot.constants.Constants2022.FieldConstants;
import frc4488.robot.constants.Constants2022.ShooterConstants;
import frc4488.robot.subsystems.eruption.Shooter;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SpinFlywheel extends Command {
  private final Shooter shooter;
  private final TargetCamera limelight;
  private final Supplier<Pose2d> pose;
  private final BooleanSupplier flywheelBeambreakState;
  private final CameraPositionConstants cameraConsts;

  private final boolean estimate;
  private final boolean interpolateShooter;
  private double rpmOffset;
  private double hoodOffset;
  private static final double FALLBACK_SPEED = ShooterConstants.BACK_OF_TARMAC_RPM;
  private static final double FALLBACK_HOOD_INPUT = ShooterConstants.BACK_OF_TARMAC_HOOD_INPUT;
  private double lastSpeed = FALLBACK_SPEED;
  private EdgeTrigger flywheelBB = new EdgeTrigger();

  public SpinFlywheel(
      Shooter shooter,
      TargetCamera limelight,
      BooleanSupplier flywheelBeambreakState,
      Supplier<Pose2d> pose,
      boolean estimate,
      boolean interpolateShooter) {
    this.shooter = shooter;
    this.limelight = limelight;
    this.flywheelBeambreakState = flywheelBeambreakState;
    this.pose = pose;
    cameraConsts = limelight.getCameraPositionConsts();
    this.estimate = estimate;
    this.interpolateShooter = interpolateShooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setHoodPosition(FALLBACK_HOOD_INPUT);
    /*
    if (limelight != null) {
      limelight.setLed(VisionLEDMode.kOn);
    }
    */
    flywheelBB.update(flywheelBeambreakState.getAsBoolean());
  }

  @Override
  public void execute() {
    spinToSpeed();
  }

  private void spinToSpeed() {
    double speed;
    rpmOffset = shooter.getRPMOffset();
    hoodOffset = shooter.getHoodOffset();

    speed = getSpeed();
    lastSpeed = speed;

    speed += rpmOffset;
    // SmartDashboard.putNumber("SpinFlywheel RPM (offset)", speed);

    if (interpolateShooter) {
      shooter.setRPM(speed);
    }
  }

  private double getSpeed() {
    double speed;
    double yOffset;

    if (limelight.hasTargets()) {
      yOffset = limelight.getBestTarget().get().getY().getDegrees();
    } else {
      if (estimate) {
        yOffset = groundToYOffset();
      } else {
        return lastSpeed;
      }
    }

    speed = shooter.getRPMFromYOffset(yOffset);

    double hoodLevel = shooter.getHoodLevelFromYOffset(yOffset);
    hoodLevel += hoodOffset;
    shooter.setHoodPosition(hoodLevel);
    if (flywheelBB.getFallingUpdate(flywheelBeambreakState.getAsBoolean())) {
      shooter.logShooterValues(yOffset, speed + rpmOffset, hoodLevel);
    }

    return speed;
  }

  private double groundToYOffset() {
    double ground_dist =
        pose.get().getTranslation().getDistance(FieldConstants.HUB_CENTER)
            - FieldConstants.HUB_RADIUS_METERS;
    double angleToLimelight =
        Math.atan((FieldConstants.TARGET_HEIGHT_METERS - cameraConsts.camHeight) / ground_dist);
    // Y-offset after being passed to a Rotation2d decreases as the target gets higher in the
    // image,
    // but that's not what our interpolation tables use. To retain the old output, the camera
    // pitch
    // (which is now negative when pointed up) must be added instead of subtracted.
    double yOffset = angleToLimelight + cameraConsts.camToNormalAngle.getDegrees();
    return Math.toDegrees(yOffset);
  }

  @Override
  public void end(boolean interrupted) {
    /*
    if (limelight != null) {
      limelight.setLed(VisionLEDMode.kOff);
    }
    */
  }
}
