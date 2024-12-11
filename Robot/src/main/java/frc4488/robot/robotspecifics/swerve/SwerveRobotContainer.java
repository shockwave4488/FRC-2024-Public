package frc4488.robot.robotspecifics.swerve;

import com.google.gson.JsonObject;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc4488.lib.commands.DoneCycleCommand;
import frc4488.lib.controlsystems.DoneCycleMachine;
import frc4488.lib.dashboard.DashboardServer;
import frc4488.lib.dashboard.gui.CameraWidget;
import frc4488.lib.dashboard.gui.FieldWidget;
import frc4488.lib.dashboard.gui.GroupWidget;
import frc4488.lib.dashboard.gui.GroupWidget.GroupWidgetDirection;
import frc4488.lib.logging.LogManager;
import frc4488.lib.misc.MatchUtil;
import frc4488.lib.operator.POVRange;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.lib.sensors.vision.Limelight;
import frc4488.lib.sensors.vision.VisionCamera.CameraPositionConstants;
import frc4488.robot.Robot;
import frc4488.robot.autonomous.modes.swerve.AutonomousChooser;
import frc4488.robot.commands.drive.LockedSwerveDrive;
import frc4488.robot.commands.drive.LockedSwerveDrive.LockedMode;
import frc4488.robot.commands.drive.RotateToAngle;
import frc4488.robot.commands.drive.SwerveModifierCommand;
import frc4488.robot.commands.drive.SwerveModifierCommand.SwerveModifier;
import frc4488.robot.commands.drive.VisionPoseUpdater;
import frc4488.robot.commands.eruption.drive.VisionAlignToTarget;
import frc4488.robot.commands.supercell.drive.DriveAndBalanceOnChargeStation;
import frc4488.robot.commands.supercell.drive.DriveAndBalanceOnChargeStation.ApproachBehavior;
import frc4488.robot.constants.Constants;
import frc4488.robot.constants.Constants.DriveTrainConstants;
import frc4488.robot.constants.Constants2024;
import frc4488.robot.robotspecifics.SwerveDriveRobotContainer;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SwerveRobotContainer extends SwerveDriveRobotContainer {
  private final Limelight limelight;
  private final AutonomousChooser autonomousChooser;

  /**
   * The robot container for our basic swerve drive robot, this is where all classes relevant to
   * this robot are created and where its default command(s) are set
   */
  public SwerveRobotContainer(PreferencesParser prefs, LogManager logger) {
    super(false, false, prefs, logger);

    JsonObject limelightPrefs = prefs.getJsonObject("LimelightConstants");
    limelight =
        new Limelight(
            limelightPrefs.get("Name").getAsString(),
            CameraPositionConstants.getFromJson(limelightPrefs),
            false,
            logger);

    autonomousChooser =
        new AutonomousChooser(
            swerve,
            limelight,
            gyro,
            autoPidControllers,
            () ->
                new TrajectoryConfig(
                    Constants.DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
                    Constants.DriveTrainConstants.SWERVE_DRIVE_MAX_ACCEL),
            prefs,
            logger);

    addSubsystems();
    configureButtonBindings();

    AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    MatchUtil.runOnRealMatchDetermination(
        () -> {
          VisionPoseUpdater.createForLimelight(
                  swerve, limelight, Constants2024.FieldConstants.getInstance().aprilTags)
              .schedule();
        });
  }

  private RotateToAngle rotateToAngle(
      Supplier<Rotation2d> desiredAngle, boolean updateContinuously) {
    ProfiledPIDController anglePidController =
        new ProfiledPIDController(
            prefs.getDouble("AutoTurnP"),
            0,
            prefs.getDouble("AutoTurnD"),
            new TrapezoidProfile.Constraints(Constants.TAU, Constants.TAU));
    anglePidController.enableContinuousInput(-Math.PI, Math.PI);
    return new RotateToAngle(swerve, gyro, anglePidController, desiredAngle, updateContinuously);
  }

  protected void addSubsystems() {
    super.addSubsystems();
    subsystems.add(limelight);
  }

  @Override
  protected RotationControls getRotationControls() {
    return RotationControls.ALWAYS_STANDARD;
  }

  protected void configureButtonBindings() {
    super.configureButtonBindings();

    driverJoystick
        .leftTrigger(0.75)
        .toggleOnTrue(
            new SwerveModifierCommand(swerve)
                .bindModifierToggle(buttonBox.button(1), SwerveModifier.forSpeed(0.25))
                .bindModifierToggle(buttonBox.button(2), SwerveModifier.forSpeed(0.1, 0.15))
                .bindModifier(
                    new POVRange(driverJoystick, POVRange.UP),
                    SwerveModifier.forCenterOffset(0.5, 0))
                .bindModifier(
                    new POVRange(driverJoystick, POVRange.DOWN),
                    SwerveModifier.forCenterOffset(-0.5, 0))
                .bindModifier(
                    new POVRange(driverJoystick, POVRange.RIGHT),
                    SwerveModifier.forCenterOffset(0, -0.5))
                .bindModifier(
                    new POVRange(driverJoystick, POVRange.LEFT),
                    SwerveModifier.forCenterOffset(0, 0.5)));

    driverJoystick
        .rightTrigger(0.75)
        .toggleOnTrue(new LockedSwerveDrive(swerve, LockedMode.XShape));

    DoneCycleCommand<VisionAlignToTarget> visionAlign =
        new DoneCycleCommand<>(
                new VisionAlignToTarget(
                    swerve,
                    limelight,
                    gyro,
                    DriveTrainConstants.SWERVE_ROTATION_MAX_SPEED,
                    () -> {
                      Pair<Double, Double> driveValues = getDriverLeftStickInput().get();
                      return driveValues.getFirst() == 0 && driveValues.getSecond() == 0;
                    }),
                true)
            .withDoneCycles(
                DoneCycleMachine.supplierWithMinCycles(limelight::hasTargets, 10)
                    .withName("Has targets"))
            .withDoneCycles(
                cmd ->
                    DoneCycleMachine.supplierWithMinCycles(
                            cmd::pidAtSetpoint, VisionAlignToTarget.MIN_AT_SETPOINT_CYCLES)
                        .withName("At setpoint"));

    driverJoystick.y().toggleOnTrue(visionAlign);

    driverJoystick
        .a()
        .whileTrue(
            DriveAndBalanceOnChargeStation.create(
                swerve, gyro, ApproachBehavior.fromVelocity(swerve, () -> 1.8)));
  }

  @Override
  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand();
  }

  @Override
  public void onDashboardInit(DashboardServer dashboard) {
    super.onDashboardInit(dashboard);

    GroupWidget root = dashboard.getWebsite().getWidgets();
    root.setDirection(GroupWidget.GroupWidgetDirection.VERTICAL);
    GroupWidget vision = new GroupWidget(GroupWidgetDirection.HORIZONTAL, true, true);
    root.addWidget(vision);
    vision.addWidget(CameraWidget.forLimelight());
    vision.addWidget(new CameraWidget("{60}:1182/stream.mjpg", true));
    root.addWidget(new FieldWidget("Field", FieldWidget.FieldYear.Y2023));
    autonomousChooser.setupDropdown(root);
  }
}
