package frc4488.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc4488.lib.misc.MatchUtil;

public class Constants2024 {

  public static class RobotConstants {
    public static final double X_SIZE = 0.81;
    public static final double Y_SIZE = 0.81;
    public static final double MAX_ARM_ACCELERATION = 3.25;
    public static final double MAX_ARM_VELOCITY = 3.5;

    public static class ArmConstants {
      public static final double THRESHOLD = 0.01; // radians
      public static final double THRESHOLD_SUBWOOFER = 0.04; // radians
    }

    public static class ShooterConstants {
      public static final double THRESHOLD = 5;
      public static final double THRESHOLD_SUBWOOFER = 60;
      public static final double SPEED = 12.219; // m/s
      public static final double WAIT_SPEED = 1; // m/s
    }

    public static class NeuralNetworkConstants {
      public static final int NN_PIPELINE_INDEX = 3; // Object Detection pipeline index
    }
  }

  public static class FieldConstants {

    public static final double FIELD_WIDTH = 8.211231;
    public static final double FIELD_LENGTH = 16.541052;
    public static final double SOURCE_SLOT_WIDTH = Units.inchesToMeters(25);
    public static final double SUBWOOFER_FRONT_SIZE = 0.95;
    public static final double SUBWOOFER_LEFT_ANGLE = 1.0472; // radians
    public static final double SUBWOOFER_RIGHT_ANGLE = -1.0472; // radians
    private static final AprilTagFieldLayout PRE_INITIALIZED_APRIL_TAGS =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private static FieldConstants instance;

    public static synchronized FieldConstants getInstance() {
      if (instance == null) {
        if (MatchUtil.isRealMatch().isEmpty() && DriverStation.isDisabled()) {
          throw new RuntimeException(
              """
              No FieldConstants instance available at the time of calling getInstance(). \
              Most likely the call should be moved to a place that will only be run \
              when the robot is enabled.""");
        } else {
          instance = new FieldConstants();
        }
      }
      return instance;
    }

    public static void eagerlyInitialize() {
      // Either enabled or connected to the FMS at this point
      MatchUtil.runOnRealMatchDetermination(
          () -> {
            synchronized (FieldConstants.class) {
              if (instance == null) instance = new FieldConstants();
            }
          });
    }

    public final int southSourceTag;
    public final int northSourceTag;
    public final int southSpeakerTag;
    public final int northSpeakerTag;
    public final int ampTag;
    public final int southStageTag;
    public final int centerStageTag;
    public final int northStageTag;
    public final AprilTagFieldLayout aprilTags = PRE_INITIALIZED_APRIL_TAGS;
    public final Translation3d speakerOpeningCenter;

    {
      if (DriverStation.getAlliance().orElseThrow() == Alliance.Red) {
        aprilTags.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        southSourceTag = 1;
        northSourceTag = 2;
        southSpeakerTag = 3;
        northSpeakerTag = 4;
        ampTag = 5;
        southStageTag = 11;
        centerStageTag = 13;
        northStageTag = 12;
      } else {
        southSourceTag = 10;
        northSourceTag = 9;
        southSpeakerTag = 8;
        northSpeakerTag = 7;
        ampTag = 6;
        southStageTag = 16;
        centerStageTag = 14;
        northStageTag = 15;
      }

      double y = 5.5493425;
      if (DriverStation.getAlliance().orElseThrow() == Alliance.Red) {
        y = FIELD_WIDTH - y;
      }
      speakerOpeningCenter = new Translation3d(0.2039205, y, 2.0506285);
    }
  }
}
