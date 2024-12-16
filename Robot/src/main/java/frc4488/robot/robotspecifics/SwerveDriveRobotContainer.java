package frc4488.robot.robotspecifics;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc4488.lib.BaseRobotContainer;
import frc4488.lib.autonomous.AutoPIDControllerContainer;
import frc4488.lib.commands.CommandUtil;
import frc4488.lib.devices.Conductor;
import frc4488.lib.drive.SwerveParameters;
import frc4488.lib.logging.LogLevel;
import frc4488.lib.logging.LogManager;
import frc4488.lib.operator.CircularDeadzone;
import frc4488.lib.operator.Controller;
import frc4488.lib.operator.DynamicController;
import frc4488.lib.operator.I2DDeadzoneCalculator;
import frc4488.lib.operator.IDeadzoneCalculator;
import frc4488.lib.operator.SafeController;
import frc4488.lib.operator.SquareDeadzoneCalculator;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.lib.sensors.gyro.IGyro;
import frc4488.lib.sensors.gyro.navx.NavXGyro;
import frc4488.lib.sensors.gyro.pigeon.PigeonGyro;
import frc4488.robot.DemoLevel;
import frc4488.robot.commands.drive.HeadingRotation;
import frc4488.robot.commands.drive.StandardDrive;
import frc4488.robot.commands.drive.StandardRotation;
import frc4488.robot.commands.drive.SwerveModifierCommand.SwerveModifier;
import frc4488.robot.constants.Constants;
import frc4488.robot.constants.Constants.DriveTrainConstants;
import frc4488.robot.constants.Constants.OIConstants;
import frc4488.robot.subsystems.drive.ISwerveModule;
import frc4488.robot.subsystems.drive.SwerveDrive;
import frc4488.robot.subsystems.drive.SwerveModuleFalcons;
import frc4488.robot.subsystems.drive.SwerveModuleNeos;
import java.util.function.Supplier;

public abstract class SwerveDriveRobotContainer extends BaseRobotContainer {

  protected enum RotationControls {
    /**
     * Defaults to heading, switches to standard when the start button is pressed. This is the
     * default option.
     */
    DEFAULT_HEADING,
    /** Defaults to standard, switches to heading when the start button is pressed */
    DEFAULT_STANDARD,
    /** Same as DEFAULT_HEADING, but the start button does nothing */
    ALWAYS_HEADING,
    /** Same as DEFAULT_STANDARD, but the start button does nothing */
    ALWAYS_STANDARD,
    /** SwerveDriveRobotContainer will not setup rotation controls */
    NONE;
  }

  protected final IGyro gyro;
  protected final DemoLevel demoLevel;
  protected final Conductor conductor;
  protected final SwerveDrive swerve;
  protected final I2DDeadzoneCalculator circularDeadzone;
  protected final I2DDeadzoneCalculator bigCircularDeadzone;
  protected final IDeadzoneCalculator squareDeadzone;
  protected final Controller driverJoystick;
  protected final CommandGenericHID buttonBox;
  protected final ISwerveModule[] swerveModules;
  protected final AutoPIDControllerContainer autoPidControllers;
  protected final Supplier<TrajectoryConfig> trajConfig;

  public SwerveDriveRobotContainer(
      boolean falconSwerve, boolean pigeonGyro, PreferencesParser prefs, LogManager logger) {
    super(prefs, logger);

    if (pigeonGyro) {
      gyro = new PigeonGyro(prefs.getInt("PigeonGyroCanID"), prefs.getString("CanivoreBusName"));
    } else {
      gyro = new NavXGyro(SPI.Port.kMXP);
    }

    demoLevel = DemoLevel.values()[prefs.tryGetValue(prefs::getInt, "DemoLevel", 0)];
    logger.getMainLog().println(LogLevel.INFO, "Demo Level: " + demoLevel);

    circularDeadzone =
        new CircularDeadzone(OIConstants.DEFAULT_CONTROLLER_DEADZONE, value -> value * value);
    bigCircularDeadzone = new CircularDeadzone(OIConstants.BIG_CONTROLLER_DEADZONE, value -> value);
    squareDeadzone = new SquareDeadzoneCalculator(OIConstants.DEFAULT_CONTROLLER_DEADZONE);

    conductor = new Conductor();

    SwerveParameters[] swerveParameters =
        SwerveParameters.getAllFromJson(prefs.getJsonObject("SwerveParameters"));
    swerveModules = new ISwerveModule[swerveParameters.length];
    for (int i = 0; i < swerveModules.length; i++) {
      if (falconSwerve) {
        swerveModules[i] = new SwerveModuleFalcons(swerveParameters[i], conductor, logger, prefs);
      } else {
        swerveModules[i] = new SwerveModuleNeos(swerveParameters[i], logger, prefs);
      }
    }

    swerve =
        new SwerveDrive(
            gyro, Rotation2d.fromDegrees(prefs.getDouble("GyroAdjustment")), swerveModules);
    driverJoystick =
        SafeController.forDemoLevel(
            demoLevel, DynamicController.getXboxOrPlaystation(OIConstants.DRIVER_CONTROLLER_PORT));
    buttonBox = new CommandGenericHID(OIConstants.BUTTON_BOX_PORT);

    autoPidControllers =
        new AutoPIDControllerContainer(
            new PIDController(
                prefs.getDouble("AutoPosPathP"),
                prefs.getDouble("AutoPosPathI"),
                prefs.getDouble("AutoPosPathD")),
            new PIDController(
                prefs.getDouble("AutoPosPathP"),
                prefs.getDouble("AutoPosPathI"),
                prefs.getDouble("AutoPosPathD")),
            new ProfiledPIDController(
                prefs.getDouble("AutoTurnP"),
                prefs.getDouble("AutoTurnI"),
                prefs.getDouble("AutoTurnD"),
                new TrapezoidProfile.Constraints(Constants.TAU, Constants.TAU)));

    trajConfig =
        () ->
            new TrajectoryConfig(
                DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
                DriveTrainConstants.SWERVE_DRIVE_MAX_ACCEL);

    swerve.driveRequirement.setDefaultCommand(
        new StandardDrive(
            swerve,
            DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
            getDriverLeftStickInput(),
            driverJoystick.start()));
    swerve.modifierRequirement.setDefaultCommand(
        CommandUtil.indefiniteInstantCommand(swerve::clearModifier, swerve.modifierRequirement)
            .withName("ResetSwerveModifier"));
  }

  protected void addSubsystems() {
    subsystems.add(swerve);
  }

  @Override
  protected void configureButtonBindings() {
    super.configureButtonBindings();

    RotationControls rotControls = getRotationControls();
    if (rotControls != RotationControls.NONE) {
      driverJoystick
          .rightTrigger()
          .whileTrue(
              new StandardRotation(
                      swerve,
                      -DriveTrainConstants.SWERVE_ROTATION_MAX_SPEED,
                      () -> squareDeadzone.deadzone(driverJoystick.getRightX()))
                  .alongWith(
                      new StartEndCommand(
                          () -> swerve.setModifier(SwerveModifier.forCenterOffset(0.5, 0)),
                          swerve::clearModifier,
                          swerve.modifierRequirement)));

      Command headingRotationCommand = getNewHeadingSwerveDriveCommand();
      Command standardRotationCommand =
          new StandardRotation(
              swerve,
              -DriveTrainConstants.SWERVE_ROTATION_MAX_SPEED,
              () -> squareDeadzone.deadzone(driverJoystick.getRightX()));
      boolean defaultHeadingRotation = (rotControls == RotationControls.DEFAULT_HEADING);

      swerve.rotationRequirement.setDefaultCommand(
          defaultHeadingRotation ? headingRotationCommand : standardRotationCommand);
      if (rotControls != RotationControls.ALWAYS_HEADING
          && rotControls != RotationControls.ALWAYS_STANDARD) {
        driverJoystick
            .start()
            .toggleOnTrue(
                defaultHeadingRotation ? standardRotationCommand : headingRotationCommand);
      }
    }
  }

  protected RotationControls getRotationControls() {
    return RotationControls.DEFAULT_HEADING;
  }

  protected Supplier<Pair<Double, Double>> getDriverRightStickInput() {
    return () ->
        bigCircularDeadzone.deadzone(
            driverJoystick.getRightY() * -1, driverJoystick.getRightX() * -1);
  }

  protected Supplier<Pair<Double, Double>> getDriverLeftStickInput() {
    return () ->
        circularDeadzone.deadzone(driverJoystick.getLeftY() * -1, driverJoystick.getLeftX() * -1);
  }

  protected HeadingRotation getNewHeadingSwerveDriveCommand() {
    /*
    The multiplication of SWERVE_ROTATION_SPEED by -1 and the first two getY/getX values being switched and multiplied by -1 are intentional.
    Multiplying SWERVE_ROTATION_SPEED by -1 corrects the direction of rotation of our robot, and we switch getY/getX and multiply them by -1 because the controller input is
    90 degrees off compared to the values WPILib utilities expect (particularly ChassisSpeeds)
    */

    return new HeadingRotation(
        swerve,
        gyro,
        autoPidControllers,
        DriveTrainConstants.SWERVE_ROTATION_MAX_SPEED * -1,
        getDriverRightStickInput(),
        false);
  }
}
