package frc4488.robot.robotspecifics.eruption;

import com.google.gson.JsonObject;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc4488.lib.commands.DoneCycleCommand;
import frc4488.lib.controlsystems.DoneCycleMachine;
import frc4488.lib.dashboard.DashboardServer;
import frc4488.lib.dashboard.gui.CameraWidget;
import frc4488.lib.dashboard.gui.FieldWidget;
import frc4488.lib.dashboard.gui.GroupWidget;
import frc4488.lib.logging.LeveledSmartDashboard;
import frc4488.lib.logging.LogManager;
import frc4488.lib.misc.Util;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.lib.sensors.vision.Limelight;
import frc4488.lib.sensors.vision.VisionCamera.CameraPositionConstants;
import frc4488.robot.DemoLevel;
import frc4488.robot.Robot;
import frc4488.robot.autonomous.modes.eruption.AutonomousChooser;
import frc4488.robot.autonomous.modes.eruption.AutonomousTrajectories;
import frc4488.robot.commands.LEDs.SetLEDMode;
import frc4488.robot.commands.drive.RotateTowardsPosition;
import frc4488.robot.commands.drive.StandardRotation;
import frc4488.robot.commands.eruption.defaults.DefaultDemoShooter;
import frc4488.robot.commands.eruption.defaults.DefaultIndexerLoad;
import frc4488.robot.commands.eruption.defaults.DefaultIntakeRetracted;
import frc4488.robot.commands.eruption.defaults.DefaultShooter;
import frc4488.robot.commands.eruption.drive.TurnToHubPoseThenVision;
import frc4488.robot.commands.eruption.drive.VisionAlignToTarget;
import frc4488.robot.commands.eruption.intake.ColorIntake;
import frc4488.robot.commands.eruption.intake.ColorlessIntake;
import frc4488.robot.commands.eruption.intake.PurgeAllIntake;
import frc4488.robot.commands.eruption.intake.PurgeBack;
import frc4488.robot.commands.eruption.shooter.CalculatedShot;
import frc4488.robot.commands.eruption.shooter.LaunchFenderShot;
import frc4488.robot.commands.eruption.shooter.PurgeForward;
import frc4488.robot.commands.eruption.shooter.SpinFlywheel;
import frc4488.robot.commands.supercell.drive.DriveAndBalanceOnChargeStation;
import frc4488.robot.commands.supercell.drive.DriveAndBalanceOnChargeStation.ApproachBehavior;
import frc4488.robot.constants.Constants.DriveTrainConstants;
import frc4488.robot.constants.Constants2022.FieldConstants;
import frc4488.robot.constants.Constants2022.ShooterConstants;
import frc4488.robot.robotspecifics.SwerveDriveRobotContainer;
import frc4488.robot.subsystems.SmartPCM;
import frc4488.robot.subsystems.eruption.Indexer;
import frc4488.robot.subsystems.eruption.Intake;
import frc4488.robot.subsystems.eruption.Shooter;
import frc4488.robot.subsystems.leds.ArduinoLEDController;
import frc4488.robot.subsystems.leds.LEDController;
import frc4488.robot.subsystems.leds.LEDMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class EruptionRobotContainer extends SwerveDriveRobotContainer {
  private final Limelight shooterLimelight;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Indexer.StateSupplier indexerStates;
  private final Intake intake;
  private final LEDController ledController;
  private final SmartPCM smartPCM;
  private final AutonomousChooser autonomousChooser;

  /**
   * The robot container for our basic swerve drive robot, this is where all classes relevant to
   * this robot are created and where its default command(s) are set
   */
  public EruptionRobotContainer(PreferencesParser prefs, LogManager logger) {
    super(true, false, prefs, logger);

    JsonObject limelightPrefs = prefs.getJsonObject("LimelightConstants");
    shooterLimelight =
        new Limelight(
            limelightPrefs.get("Name").getAsString(),
            CameraPositionConstants.getFromJson(limelightPrefs),
            false,
            logger);

    shooter =
        new Shooter(
            prefs.getInt("ShooterMFlywheelID"),
            prefs.getInt("ShooterFFlywheelID"),
            prefs.getInt("Shooter_PCM_ID"),
            prefs.getInt("HoodServo1ID"),
            prefs.getInt("HoodServo2ID"),
            logger,
            prefs);
    indexer =
        new Indexer(
            prefs.getInt("ConveyorID"),
            prefs.getInt("EntranceBBID"),
            prefs.getInt("MiddleBBID"),
            prefs.getInt("FlywheelBBID"),
            prefs);
    indexerStates = indexer.getIndexerStates();
    intake =
        new Intake(
            prefs.getInt("IntakeTopRollerID"),
            prefs.getInt("IntakeBottomRollerID"),
            prefs.getInt("Intake_PCM_ID"),
            prefs.getInt("IntakePistonID"),
            prefs.getInt("IntakeColorSensorID"),
            prefs.getInt("IntakeBallSensorID"));
    smartPCM = new SmartPCM(prefs.getInt("Intake_PCM_ID"));
    ledController =
        new ArduinoLEDController(LEDMode.Eruption.seismic(), prefs.getIntArray("LedDIO"));

    /*
    Set initial pose to have the robot pointing at the HUB so the robot doesn't snap upon
    enabling This will be overridden in real matches because autonomousInit will run in Robot.java.
    */
    swerve.resetOdometry(new Pose2d(0, 4.15, new Rotation2d(0)));

    intake.setDefaultCommand(new DefaultIntakeRetracted(intake));
    smartPCM.setDefaultCommand(
        new StartEndCommand(() -> smartPCM.startCompressor(), () -> {}, smartPCM)
            .withName("CompressorCommand"));
    indexer.setDefaultCommand(new DefaultIndexerLoad(indexer));

    // Assigns buttons bindings, default drive, and default shooter
    if (demoLevel != DemoLevel.NONE) {
      swerve.rotationRequirement.setDefaultCommand(getNewHeadingSwerveDriveCommand());
      shooter.setDefaultCommand(new DefaultDemoShooter(shooter));
      configureDemoModeBindings();
    } else {
      swerve.rotationRequirement.setDefaultCommand(
          new TurnToHubPoseThenVision(
              getNewSwerveTurnToHubCommand(), getNewVisionAlignToTargetCommand(false, 10)));
      shooter.setDefaultCommand(
          new DefaultShooter(
              shooter, shooterLimelight, indexerStates::getFlywheelBeamBreak, swerve::getOdometry));
      configureCompetitionBindings();
    }

    autonomousChooser =
        new AutonomousChooser(
            new AutonomousTrajectories(swerve, logger),
            swerve,
            intake,
            shooter,
            indexer,
            smartPCM,
            gyro,
            shooterLimelight,
            logger,
            prefs);

    addSubsystems();
  }

  protected void addSubsystems() {
    super.addSubsystems();
    subsystems.add(shooterLimelight);
    subsystems.add(shooter);
    subsystems.add(indexer);
    subsystems.add(intake);
    subsystems.add(ledController);
    subsystems.add(smartPCM);
  }

  @Override
  protected RotationControls getRotationControls() {
    return RotationControls.NONE;
  }

  private void configureDemoModeBindings() {
    super.configureButtonBindings();

    driverJoystick
        .back()
        .toggleOnTrue(
            new StandardRotation(
                    swerve,
                    -DriveTrainConstants.SWERVE_ROTATION_MAX_SPEED,
                    () -> squareDeadzone.deadzone(driverJoystick.getRightX()))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    driverJoystick.rightBumper().whileTrue(new PurgeAllIntake(intake, indexer, shooter));
    driverJoystick.leftBumper().whileTrue(new PurgeBack(intake, indexer, shooter));
    driverJoystick.rightTrigger().whileTrue(new ColorlessIntake(intake));
    driverJoystick.leftTrigger().whileTrue(new PurgeForward(indexer, shooter));
    driverJoystick.a().whileTrue(new ColorIntake(intake, indexerStates, false));
    driverJoystick.b().whileTrue(new ColorIntake(intake, indexerStates, true));
    driverJoystick.x().toggleOnTrue(new SetLEDMode(ledController, LEDMode.Eruption.seismic()));

    // Activated if driver joystick value is outside of deadzone
    Trigger driverRightJoystick =
        new Trigger(
            () ->
                bigCircularDeadzone.isPastDeadzone(
                    driverJoystick.getRightY() * -1, driverJoystick.getRightX() * -1));
    driverRightJoystick.whileTrue(new RepeatCommand(getNewHeadingSwerveDriveCommand()));
  }

  protected void configureCompetitionBindings() {
    super.configureButtonBindings();

    driverJoystick
        .back()
        .toggleOnTrue(
            new StandardRotation(
                    swerve,
                    -DriveTrainConstants.SWERVE_ROTATION_MAX_SPEED,
                    () -> squareDeadzone.deadzone(driverJoystick.getRightX()))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    driverJoystick.x().onTrue(getNewSwerveTurnToHubCommand());
    driverJoystick.leftTrigger().whileTrue(new PurgeBack(intake, indexer, shooter));
    driverJoystick.rightTrigger().toggleOnTrue(new ColorlessIntake(intake));
    driverJoystick
        .rightBumper()
        .toggleOnTrue(
            getNewCalculatedShotCommand()
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    driverJoystick.leftBumper().whileTrue(new PurgeForward(indexer, shooter));

    driverJoystick
        .a()
        .whileTrue(
            DriveAndBalanceOnChargeStation.create(
                swerve, gyro, ApproachBehavior.fromVelocity(swerve, () -> 1.8)));

    driverJoystick
        .povUp()
        .onTrue(new InstantCommand(() -> shooter.setRPMOffset(shooter.getRPMOffset() + 10)));
    driverJoystick
        .povDown()
        .onTrue(new InstantCommand(() -> shooter.setRPMOffset(shooter.getRPMOffset() - 10)));

    // Button box/operator bindings
    buttonBox.button(1).toggleOnTrue(getNewVisionAlignToTargetCommand(false, 10));
    // Toggle so operator can stop if aligned to the wrong target
    buttonBox
        .button(3)
        .toggleOnTrue(
            getNewCalculatedShotCommand()
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    buttonBox.button(4).onTrue(shooterLimelight.snapshotCommand());
    buttonBox
        .button(5)
        .onTrue(
            new LaunchFenderShot(
                shooter,
                indexer,
                smartPCM,
                ShooterConstants.FENDER_RPM,
                ShooterConstants.FENDER_HOOD_INPUT));
    Command chargeShooter =
        new DoneCycleCommand<>(
                new SpinFlywheel(
                    shooter,
                    shooterLimelight,
                    indexerStates::getFlywheelBeamBreak,
                    swerve::getOdometry,
                    true,
                    true),
                false)
            .withDoneCycles(DoneCycleMachine.fromSupplier(shooter::hoodReady))
            .withDoneCycles(shooter.flywheelVelocityMachine);
    buttonBox.button(6).whileTrue(chargeShooter);
    buttonBox
        .button(7)
        .onTrue(new InstantCommand(() -> shooter.setRPMOffset(shooter.getRPMOffset() - 10)));
    buttonBox
        .button(8)
        .onTrue(new InstantCommand(() -> shooter.setRPMOffset(shooter.getRPMOffset() + 10)));
    buttonBox.button(10).whileTrue(new PurgeBack(intake, indexer, shooter));
    buttonBox.button(11).whileTrue(new PurgeForward(indexer, shooter));
    buttonBox.button(14).whileTrue(new ColorIntake(intake, indexerStates, false));
    buttonBox.button(15).whileTrue(new ColorlessIntake(intake));
    buttonBox.button(16).whileTrue(new ColorIntake(intake, indexerStates, true));

    if (LeveledSmartDashboard.INFO.isEnabled()) {
      LeveledSmartDashboard.INFO.putData(
          "Limelight Snapshot",
          new InstantCommand(() -> shooterLimelight.takeSnapshot()).withName("Take snapshot"));
    }

    // Activated if driver joystick value is outside of deadzone
    Trigger driverRightJoystick =
        new Trigger(
            () ->
                bigCircularDeadzone.isPastDeadzone(
                    driverJoystick.getRightY() * -1, driverJoystick.getRightX() * -1));
    driverRightJoystick.whileTrue(new RepeatCommand(getNewHeadingSwerveDriveCommand()));
  }

  private DoneCycleCommand<VisionAlignToTarget> getNewVisionAlignToTargetCommand(
      boolean stop, int hasTargetCycles) {
    return new DoneCycleCommand<>(
            new VisionAlignToTarget(
                swerve,
                shooterLimelight,
                gyro,
                DriveTrainConstants.SWERVE_ROTATION_MAX_SPEED,
                () -> {
                  Pair<Double, Double> driveValues = getDriverLeftStickInput().get();
                  return driveValues.getFirst() == 0 && driveValues.getSecond() == 0;
                }),
            stop)
        .withDoneCycles(
            DoneCycleMachine.supplierWithMinCycles(shooterLimelight::hasTargets, hasTargetCycles)
                .withName("Has targets"))
        .withDoneCycles(
            cmd ->
                DoneCycleMachine.supplierWithMinCycles(
                        cmd::pidAtSetpoint, VisionAlignToTarget.MIN_AT_SETPOINT_CYCLES)
                    .withName("At yaw setpoint"));
  }

  public RotateTowardsPosition getNewSwerveTurnToHubCommand() {
    return Util.returnAfterModifying(
        new RotateTowardsPosition(
            swerve, gyro, autoPidControllers.thetaPidController, () -> FieldConstants.HUB_CENTER),
        cmd -> cmd.setName("TurnToHub"));
  }

  private CalculatedShot getNewCalculatedShotCommand() {
    return new CalculatedShot(
        shooter,
        indexer,
        smartPCM,
        swerve,
        gyro,
        DriveTrainConstants.SWERVE_ROTATION_MAX_SPEED,
        shooterLimelight,
        getNewSwerveTurnToHubCommand(),
        getNewHeadingSwerveDriveCommand(),
        getNewVisionAlignToTargetCommand(false, CalculatedShot.MIN_HAS_TARGET_CYCLES));
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand();
  }

  @Override
  public void onDashboardInit(DashboardServer dashboard) {
    super.onDashboardInit(dashboard);

    dashboard.registerControlsAction("eruption" + (demoLevel == DemoLevel.NONE ? "" : "_demo"));

    GroupWidget root = dashboard.getWebsite().getWidgets();
    root.setDirection(GroupWidget.GroupWidgetDirection.VERTICAL);
    root.addWidget(CameraWidget.forLimelight());
    root.addWidget(new FieldWidget("Field", FieldWidget.FieldYear.Y2022));
    root.addWidget(autonomousChooser.getAutonomousModeChooser().setSizeLocked(true));
  }
}
