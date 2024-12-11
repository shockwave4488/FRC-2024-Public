package frc4488.lib.sensors.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc4488.lib.logging.LeveledSmartDashboard;
import frc4488.lib.logging.LogManager;
import frc4488.lib.math.EpsilonUtil;
import frc4488.lib.misc.Timed;
import frc4488.lib.misc.Util;
import frc4488.lib.sensors.vision.LimelightHelpers.CoordinateSpace;
import frc4488.lib.sensors.vision.LimelightHelpers.LimelightResults;
import frc4488.lib.sensors.vision.LimelightHelpers.LimelightTarget_Generic;
import frc4488.lib.sensors.vision.LimelightHelpers.PoseEstimate;
import frc4488.lib.sensors.vision.LimelightHelpers.RawFiducial;
import frc4488.lib.sensors.vision.VisionCameras.AprilTagCamera;
import frc4488.lib.sensors.vision.VisionCameras.ReflectiveTapeCamera;
import frc4488.lib.sensors.vision.VisionTargets.AprilTagLimelightTarget;
import frc4488.lib.sensors.vision.VisionTargets.LimelightTarget;
import frc4488.lib.sensors.vision.VisionTargets.RetroreflectiveLimelightTarget;
import frc4488.lib.sensors.vision.VisionTargets.VisionTarget;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.photonvision.common.hardware.VisionLEDMode;

public class Limelight extends CameraSubsystem implements ReflectiveTapeCamera, AprilTagCamera {
  static class TargetEntries {
    final DoubleSubscriber xSub, ySub, areaSub, skewSub, ambiguitySub;
    final IntegerSubscriber idSub;
    final DoubleArraySubscriber camtranSub;

    TargetEntries(NetworkTable table) {
      xSub = table.getDoubleTopic("tx").subscribe(0.0);
      ySub = table.getDoubleTopic("ty").subscribe(0.0);
      areaSub = table.getDoubleTopic("ta").subscribe(0.0);
      skewSub = table.getDoubleTopic("ts").subscribe(0.0);
      idSub = table.getIntegerTopic("tid").subscribe(-1);
      camtranSub =
          table
              .getDoubleArrayTopic("targetpose_cameraspace")
              .subscribe(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      // Not a real ambiguity topic
      ambiguitySub = table.getDoubleTopic("tpa").subscribe(0);
    }
  }

  private static final Map<VisionLEDMode, Integer> ledModeToEntryValue =
      Collections.unmodifiableMap(
          Map.of(
              VisionLEDMode.kDefault,
              0,
              VisionLEDMode.kOn,
              3,
              VisionLEDMode.kOff,
              1,
              VisionLEDMode.kBlink,
              2));

  /** Unit: Seconds */
  private static final double HEARTBEAT_TIMEOUT = 0.1;

  private static final double[] DISCONNECTED_BOTPOSE =
      new double[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  private static final LimelightResults DISCONNECTED_RESULTS = new LimelightResults();
  private static final PoseEstimate DISCONNECTED_POSE_ESTIMATE =
      new PoseEstimate(new Pose2d(0, 0, new Rotation2d()), 0, 0, 0, 0, 0, 0, new RawFiducial[0]);

  private static final int DEFAULT_PIPE = 0;
  private int snapshotCycles = 1;

  private final boolean jsonDisabled;

  private RetroreflectiveLimelightTarget bestReflectiveTarget;
  private AprilTagLimelightTarget bestAprilTagTarget;
  private LimelightHelpers.LimelightResults curJsonResult;
  private LimelightHelpers.PoseEstimate curPoseEstimate;

  private final TargetEntries targetEntries;
  private Supplier<double[]> botposeSupplier;
  private final DoubleSubscriber hasTargetSub, currentPipeSub;
  private final DoublePublisher ledControlPub, pipeControlPub, snapshotPub;

  private Optional<DigitalOutput> extraLedDio;

  private double lastHeartbeatUpdate;
  private boolean frozen;

  public Limelight(
      String name, CameraPositionConstants cameraConsts, boolean jsonDisabled, LogManager logger) {
    super(name, cameraConsts, logger);

    this.jsonDisabled = jsonDisabled;

    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    targetEntries = new TargetEntries(table);
    hasTargetSub = table.getDoubleTopic("tv").subscribe(0.0);
    currentPipeSub = table.getDoubleTopic("getpipe").subscribe(0.0);
    ledControlPub = table.getDoubleTopic("ledMode").publish();
    pipeControlPub = table.getDoubleTopic("pipeline").publish();
    snapshotPub = table.getDoubleTopic("snapshot").publish();

    botposeSupplier = () -> DISCONNECTED_BOTPOSE;

    extraLedDio = Optional.empty();
    setLed(VisionLEDMode.kOff);
    setPipeline(DEFAULT_PIPE);

    NetworkTableInstance.getDefault()
        .addListener(
            table.getIntegerTopic("hb").subscribe(0),
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> {
              lastHeartbeatUpdate = Timer.getFPGATimestamp();
            });
  }

  public Limelight(
      String name,
      CameraPositionConstants cameraConsts,
      boolean jsonDisabled,
      int secondLedDio,
      LogManager logger) {
    this(name, cameraConsts, jsonDisabled, logger);
    extraLedDio = Optional.of(new DigitalOutput(secondLedDio));
  }

  public void restart() {
    LimelightWebSocket socket =
        new LimelightWebSocket(name, logger.getLogFile(name + "/WebSocket"));
    socket.connect();
    socket.restart();
  }

  @Override
  public boolean hasTargets() {
    if (frozen) {
      return false;
    }
    return EpsilonUtil.epsilonEquals(hasTargetSub.get(), 1.0);
  }

  @Override
  public int getRunningPipeline() {
    return (int) currentPipeSub.get();
  }

  @Override
  public Optional<? extends LimelightTarget> getBestTarget() {
    // If the generic method is called, the subclass doesn't matter
    return getBestRetroreflectiveTarget();
  }

  @Override
  public List<? extends LimelightTarget> getTargets() {
    List<LimelightTarget> targetList = new ArrayList<>();
    targetList.addAll(getRetroreflectiveTargets());
    targetList.addAll(getAprilTagTargets());
    return targetList;
  }

  private <T extends VisionTarget, R extends LimelightTarget_Generic> List<T> getJSONTargets(
      Function<R, T> targetFromJson, Function<LimelightHelpers.Results, R[]> targetResults) {
    if (frozen) {
      return new ArrayList<>();
    }
    LimelightHelpers.Results jsonResults = getResults().targetingResults;
    List<T> targets = new ArrayList<>();
    for (R target : targetResults.apply(jsonResults)) {
      T constructedTarget = targetFromJson.apply(target);
      targets.add(constructedTarget);
    }
    return targets;
  }

  @Override
  public Optional<RetroreflectiveLimelightTarget> getBestRetroreflectiveTarget() {
    if (frozen) {
      return Optional.empty();
    }
    // Constructs a "retroreflective" target from the best target information.
    // There's no guarantee what kind of target it is. (Same goes for AprilTag targets)
    if (bestReflectiveTarget == null && hasTargets()) {
      bestReflectiveTarget = new RetroreflectiveLimelightTarget(targetEntries);
    }
    return Optional.ofNullable(bestReflectiveTarget);
  }

  @Override
  public List<RetroreflectiveLimelightTarget> getRetroreflectiveTargets() {
    if (frozen) {
      return new ArrayList<>();
    }
    return getJSONTargets(RetroreflectiveLimelightTarget::new, results -> results.targets_Retro);
  }

  @Override
  public Optional<AprilTagLimelightTarget> getBestAprilTagTarget() {
    if (frozen) {
      return Optional.empty();
    }
    if (bestAprilTagTarget == null && hasTargets()) {
      bestAprilTagTarget = new AprilTagLimelightTarget(targetEntries);
    }
    return Optional.ofNullable(bestAprilTagTarget);
  }

  @Override
  public List<AprilTagLimelightTarget> getAprilTagTargets() {
    if (frozen) {
      return new ArrayList<>();
    }
    return getJSONTargets(AprilTagLimelightTarget::new, results -> results.targets_Fiducials);
  }

  public List<Integer> getAprilTagTargetIds() {
    if (frozen) {
      return new ArrayList<>();
    }
    return Arrays.stream(getPoseEstimate().rawFiducials)
        .map(tag -> tag.id)
        .collect(Collectors.toList());
  }

  public Timed<Pose3d> getRobotPose() {
    double[] botpose = botposeSupplier.get();
    if (frozen) {
      botpose = DISCONNECTED_BOTPOSE;
    }
    return new Timed<>(
        Timer.getFPGATimestamp() - botpose[6] / 1000.0,
        LimelightHelpers.toGeometry3D(botpose, CoordinateSpace.Field, Pose3d::new));
  }

  public LimelightResults getResults() {
    if (jsonDisabled || frozen) {
      return DISCONNECTED_RESULTS;
    }
    return Util.lazyInitialize(
        results -> curJsonResult = results,
        () -> LimelightHelpers.getLatestResults(name),
        curJsonResult);
  }

  public PoseEstimate getPoseEstimate() {
    if (frozen) {
      return DISCONNECTED_POSE_ESTIMATE;
    }
    return Util.lazyInitialize(
        estimate -> curPoseEstimate = estimate,
        () ->
            DriverStation.getAlliance().orElseThrow() == Alliance.Red
                ? LimelightHelpers.getBotPoseEstimate_wpiRed(name)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue(name),
        curPoseEstimate);
  }

  public boolean isFrozen() {
    return frozen;
  }

  @Override
  public void setLed(VisionLEDMode controlMode) {
    ledControlPub.set(ledModeToEntryValue.get(controlMode));
    extraLedDio.ifPresent(out -> out.set(controlMode == VisionLEDMode.kOn));
  }

  @Override
  public void setPipeline(int pipeline) {
    pipeControlPub.set(pipeline);
  }

  @Override
  public void takeSnapshot() {
    setSnapshotState(0);
    setSnapshotState(1);
    snapshotCycles = 10;
  }

  /* Snapshots appear under the Input tab on the limelight connection (http://limelight.local:5801/).
   * Change the "Source Image" setting to "Snapshot" to view any snapshots collected. Warning: Snapshots do
   * not show 3D April Tag processing like the limelight Dashboard itself does.
   */
  public Command snapshotCommand() {
    return new InstantCommand(() -> setSnapshotState(0))
        .andThen(() -> setSnapshotState(1))
        .andThen(() -> snapshotCycles = 10)
        .withName("Take Snapshot");
  }

  private void setSnapshotState(int state) {
    snapshotPub.set(state);
  }

  private void resetSnapshot() {
    snapshotCycles--;
    if (snapshotCycles == 0) {
      setSnapshotState(0);
    }
    snapshotCycles = Math.max(snapshotCycles, -1);
  }

  @Override
  public void periodic() {
    bestReflectiveTarget = null;
    bestAprilTagTarget = null;
    curJsonResult = null;
    curPoseEstimate = null;

    resetSnapshot();

    frozen = (Timer.getFPGATimestamp() - lastHeartbeatUpdate > HEARTBEAT_TIMEOUT);
  }

  @Override
  public void updateSmartDashboard() {
    super.updateSmartDashboard();
    putDashboardPrints();
    LeveledSmartDashboard.INFO.putNumber(
        name + "-X", getBestTarget().map(target -> target.getX().getDegrees()).orElse(0.));
    LeveledSmartDashboard.INFO.putNumber(
        name + "-Y", getBestTarget().map(target -> target.getY().getDegrees()).orElse(0.));
    LeveledSmartDashboard.HIGH.putBoolean(name + "-HasTarget", hasTargets());
    LeveledSmartDashboard.INFO.putNumber(
        name + "-Area", getBestTarget().map(target -> target.getArea()).orElse(0.));
    LeveledSmartDashboard.INFO.putNumber(name + "-Pipe", getRunningPipeline());
    LeveledSmartDashboard.INFO.putBoolean(name + "-Frozen", frozen);
  }

  @Override
  public void onStart(boolean sStopped) {
    botposeSupplier =
        DriverStation.getAlliance().orElseThrow() == Alliance.Blue
            ? () -> LimelightHelpers.getBotPose_wpiBlue(name)
            : () -> LimelightHelpers.getBotPose_wpiRed(name);

    setLed(VisionLEDMode.kOff);
  }

  @Override
  public void onStop(boolean sStopped) {
    setLed(VisionLEDMode.kOff);
  }

  @Override
  public void setUpTrackables(LogManager logger) {
    super.setUpTrackables(logger);
    logger
        .getLogFile(name + "/TargetXY")
        .setDefaultFrequency(10)
        .addTracker("X", () -> getBestTarget().map(target -> target.getX().getDegrees()).orElse(0.))
        .addTracker("Y", () -> getBestTarget().map(target -> target.getY().getDegrees()).orElse(0.))
        .addTracker("Frozen", () -> frozen)
        .addTracker("Last HB", () -> lastHeartbeatUpdate);
  }
}
