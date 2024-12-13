package frc4488.robot.autonomous.modes.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc4488.lib.autonomous.AutoPIDControllerContainer;
import frc4488.lib.autonomous.PathPlannerUtil;
import frc4488.lib.commands.LogCommand;
import frc4488.lib.dashboard.gui.DropdownWidget;
import frc4488.lib.dashboard.gui.GroupWidget;
import frc4488.lib.logging.LogLevel;
import frc4488.lib.logging.LogManager;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.lib.sensors.gyro.IGyro;
import frc4488.lib.sensors.vision.Limelight;
import frc4488.robot.commands.other.InitPositionFromTag;
import frc4488.robot.constants.Constants;
import frc4488.robot.subsystems.drive.SwerveDrive;
import java.util.function.Supplier;

public class AutonomousChooser {

  private final SwerveDrive swerve;
  private final Limelight camera;
  private final IGyro gyro;
  private final PathConstraints pathConstraints;
  private final AutoPIDControllerContainer pid;
  private final LogManager logger;
  private final DropdownWidget<AutonomousMode> dropdown;

  public AutonomousChooser(
      SwerveDrive swerve,
      Limelight camera,
      IGyro gyro,
      AutoPIDControllerContainer pid,
      Supplier<TrajectoryConfig> config,
      PreferencesParser prefs,
      LogManager logger) {
    this.swerve = swerve;
    this.camera = camera;
    this.gyro = gyro;
    this.pid = pid;
    this.logger = logger;
    this.dropdown = new DropdownWidget<>("Auto Mode");

    for (AutonomousMode mode : AutonomousMode.values()) {
      dropdown.addOption(mode.getNiceName(), mode);
    }

    AutoBuilder.configureHolonomic(
        swerve::getOdometry,
        swerve::resetOdometry,
        swerve::getRobotRelativeChassisSpeeds,
        speeds -> swerve.assignModuleStates(speeds, false),
        new HolonomicPathFollowerConfig(
            new PIDConstants(
                prefs.getDouble("AutoPosPathP"),
                prefs.getDouble("AutoPosPathI"),
                prefs.getDouble("AutoPosPathD")),
            new PIDConstants(prefs.getDouble("AutoTurnP"), 0.0, 0.0),
            Constants.DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
            prefs.getDouble("DriveBaseRadius"),
            new ReplanningConfig()),
        // We aren't flipping the path here because our code maintains an origin based on whichever
        // alliance we are on. PathPlanner's origin is ALWAYS blue, which throws off our pose
        // estimation.
        () -> false,
        swerve.driveRequirement);

    pathConstraints =
        new PathConstraints(
            Constants.DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
            Constants.DriveTrainConstants.SWERVE_DRIVE_MAX_ACCEL,
            Constants.DriveTrainConstants.SWERVE_ROTATION_MAX_SPEED,
            Constants.DriveTrainConstants.SWERVE_ROTATION_MAX_ACCEL);

    registerCommands();
  }

  public enum AutonomousMode {
    PATH_PLANNER_CURVE("Path Planner Curve Test"),
    STRAIGHT("Straight");

    private final String name;

    private AutonomousMode(String name) {
      this.name = name;
    }

    public String getNiceName() {
      return name;
    }
  }

  private void registerCommands() {}

  // Example Auto
  public Command getPathPlannerCurveTestCommand() {
    // Assign a command that builds the desired .auto auto
    Command fullCurveTestAuto = PathPlannerUtil.buildAutoWithAlliance("CurvySpinnyPathyThing");
    // Add the rotation requirement for swerve
    fullCurveTestAuto.addRequirements(swerve.rotationRequirement);

    // Write the path to the log
    logger.getMainLog().println(LogLevel.INFO, "CurvySpinnyPathyThing");

    // Start each command sequence with a pose reset. This can be using april tags or according
    // to the first path segment of the auto.
    // Next, run the built auto command. Any Named Commands part of waypoints will run if recorded
    // as named commands
    // in the registerCommands() method above.
    return resetRobotPose(PathPlannerUtil.getAutoStartingPoseWithAlliance("CurvySpinnyPathyThing"))
        .andThen(fullCurveTestAuto);
  }

  // Example auto using paths
  public Command getStraightPath() {
    Command followPath = PathPlannerUtil.buildAutoWithAlliance("Straight");
    followPath.addRequirements(swerve.rotationRequirement);

    return resetRobotPose(PathPlannerUtil.getAutoStartingPoseWithAlliance("Straight"))
        .andThen(followPath);
  }

  private Command resetRobotPoseFromTag(Pose2d fallback) {
    return LogCommand.race(
        InitPositionFromTag.create(swerve, gyro, camera),
        new WaitCommand(0.5).andThen(resetRobotPose(fallback)));
  }

  // Default
  public Command getDoNothingCommand() {
    return resetRobotPoseFromTag(new Pose2d(new Translation2d(1, 1), new Rotation2d(0)));
  }

  public Command getCommand() {
    AutonomousMode mode = dropdown.getSelected();

    if (mode == null) {
      return getDoNothingCommand();
    }

    Command autoCommand =
        switch (mode) {
          case PATH_PLANNER_CURVE -> getPathPlannerCurveTestCommand();
          case STRAIGHT -> getStraightPath();
          default -> getDoNothingCommand();
        };
    autoCommand.setName(mode.getNiceName().replaceAll("\\s+", ""));
    return autoCommand;
  }

  public void setupDropdown(GroupWidget root) {
    dropdown.setSizeLocked(true);
    root.addWidget(dropdown);
  }

  private Command resetRobotPose(Pose2d pose) {
    return new InstantCommand(
        () -> {
          gyro.setYawAdjustment(new Rotation2d());
          gyro.setYawAdjustment(pose.getRotation().minus(gyro.getYaw()));
          swerve.resetOdometry(pose);
        });
  }
}
