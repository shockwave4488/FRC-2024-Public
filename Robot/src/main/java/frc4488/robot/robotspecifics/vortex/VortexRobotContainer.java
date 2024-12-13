package frc4488.robot.robotspecifics.vortex;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc4488.lib.commands.DoneCycleCommand;
import frc4488.lib.commands.LogCommand;
import frc4488.lib.controlsystems.DigitalInputTrigger;
import frc4488.lib.controlsystems.DoneCycleMachine;
import frc4488.lib.dashboard.DashboardServer;
import frc4488.lib.dashboard.actions.MusicAction;
import frc4488.lib.dashboard.gui.ButtonWidget;
import frc4488.lib.dashboard.gui.CameraWidget;
import frc4488.lib.dashboard.gui.CheckSelectionsWidget;
import frc4488.lib.dashboard.gui.ConnectionIndicatorWidget;
import frc4488.lib.dashboard.gui.DisplayWidget;
import frc4488.lib.dashboard.gui.DropdownWidget;
import frc4488.lib.dashboard.gui.FieldWidget;
import frc4488.lib.dashboard.gui.GroupWidget;
import frc4488.lib.dashboard.gui.GroupWidget.GroupWidgetDirection;
import frc4488.lib.dashboard.gui.SliderWidget;
import frc4488.lib.dashboard.gui.SpacerWidget;
import frc4488.lib.dashboard.gui.SwitchWidget;
import frc4488.lib.dashboard.gui.TabsWidget;
import frc4488.lib.dashboard.gui.Widget;
import frc4488.lib.drive.SwerveParameters;
import frc4488.lib.drive.SwerveParameters.ModulePosition;
import frc4488.lib.logging.LogManager;
import frc4488.lib.math.EpsilonUtil;
import frc4488.lib.misc.MatchUtil;
import frc4488.lib.misc.Util;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.lib.sensors.vision.Limelight;
import frc4488.lib.sensors.vision.VisionCamera.CameraPositionConstants;
import frc4488.lib.wpiextensions.PriorityManager;
import frc4488.robot.DemoLevel;
import frc4488.robot.autonomous.modes.vortex.AutonomousChooser;
import frc4488.robot.autonomous.modes.vortex.AutonomousChooser.AutonomousMode;
import frc4488.robot.commands.drive.LockedSwerveDrive;
import frc4488.robot.commands.drive.RotateToAngle;
import frc4488.robot.commands.drive.StandardDrive;
import frc4488.robot.commands.drive.VisionPoseUpdater;
import frc4488.robot.commands.vortex.drive.AmpAutoScore;
import frc4488.robot.commands.vortex.shooter.Shoot;
import frc4488.robot.constants.Constants;
import frc4488.robot.constants.Constants.DriveTrainConstants;
import frc4488.robot.constants.Constants2024;
import frc4488.robot.constants.Constants2024.RobotConstants.ShooterConstants;
import frc4488.robot.robotspecifics.SwerveDriveRobotContainer;
import frc4488.robot.subsystems.leds.LEDMode;
import frc4488.robot.subsystems.leds.RaspberryPiLEDController;
import frc4488.robot.subsystems.vortex.Arm;
import frc4488.robot.subsystems.vortex.Climber;
import frc4488.robot.subsystems.vortex.Intake;
import frc4488.robot.subsystems.vortex.Intake.RollerState;
import frc4488.robot.subsystems.vortex.NoteVision;
import frc4488.robot.subsystems.vortex.Shooter;
import java.nio.charset.StandardCharsets;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.photonvision.common.hardware.VisionLEDMode;

public class VortexRobotContainer extends SwerveDriveRobotContainer {

  private final Climber climber;
  private final Intake intake;
  private final Arm arm;
  private final Shooter shooter;
  private final Limelight[] limelights; // Dragon, Right, Left (only has dragon on practice)
  private final RaspberryPiLEDController ledController;
  private final PriorityManager<LEDMode.Vortex.Priorities> ledPriorities;
  private AutonomousChooser autonomousChooser;
  public final SendableChooser<AutonomousMode> autoModeChooser = new SendableChooser<>();
  private final NoteVision nnVision;
  private Supplier<Double> armTrackOffset;
  private boolean fieldRelative = true;

  private static final double SUBWOOFER_PROXIMITY = 2.0; // 2 meters

  public VortexRobotContainer(PreferencesParser prefs, LogManager logger) {
    super(true, false, prefs, logger);

    JsonArray limelightPrefs = prefs.getJsonArray("LimelightConstants");
    limelights =
        limelightPrefs.asList().stream()
            .map(JsonElement::getAsJsonObject)
            .map(
                limelightPref ->
                    new Limelight(
                        limelightPref.get("Name").getAsString(),
                        CameraPositionConstants.getFromJson(limelightPref),
                        true,
                        logger))
            .toArray(Limelight[]::new);
    shooter = new Shooter(prefs);
    climber = new Climber(prefs);
    intake = new Intake(prefs);
    arm = new Arm(prefs);

    autonomousChooser =
        new AutonomousChooser(
            swerve,
            limelights[0],
            intake,
            shooter,
            arm,
            gyro,
            autoPidControllers,
            () ->
                new TrajectoryConfig(
                    DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
                    DriveTrainConstants.SWERVE_DRIVE_MAX_ACCEL),
            prefs,
            logger);

    intake.setDefaultCommand(intake.intakeCommand(RollerState.OFF));
    shooter.setDefaultCommand(new InstantCommand(() -> shooter.coastOut(), shooter));

    ledController = new RaspberryPiLEDController(LEDMode.Vortex.idle());
    ledPriorities =
        new PriorityManager.SubsystemPriorityManager<>(
            ledController, LEDMode.Vortex.Priorities.class);
    conductor.addEventListener(new LEDConductorEventListener(conductor, ledPriorities));
    ledPriorities.enableTempState(LEDMode.Vortex.Priorities.STARTUP);
    configureLEDAnimations();

    armTrackOffset = () -> 0.0;

    Constants2024.FieldConstants.eagerlyInitialize();

    addSubsystems();
    if (demoLevel == DemoLevel.NONE) {
      configureButtonBindings();
    } else {
      configureDemoModeBindings();
    }

    MatchUtil.runOnRealMatchDetermination(
        () -> {
          // whileFalse only triggers on a falling edge, so this is faked via returning true first
          new Trigger(Util.changeFirstReturn(RobotState::isAutonomous, true))
              .whileFalse(
                  VisionPoseUpdater.createForLimelights(
                      swerve, limelights, Constants2024.FieldConstants.getInstance().aprilTags));
        });

    nnVision =
        new NoteVision(
            prefs.getString("nnVisionCameraName"),
            Constants2024.RobotConstants.NeuralNetworkConstants.NN_PIPELINE_INDEX);

    // Shooter Prep is commented out because it often overrides the shoot commands on competition.
    // I would prefer it just be bound to a button instead of happening automatically.

    // MatchUtil.runOnRealMatchDetermination(
    //     () ->
    //         new Trigger(
    //                 () ->
    //                     intake.hasNote() /* if our intake contains a note
    //                 getOdometry gets the current position of the robot and relativeTo compares
    //                 with our subwoofer's april tag.  getTranslation extracts the translational
    //                 part of the pose (and drops the rotational part) and getNorm calculates the
    //                 distance. If that distance is less than the SUBWOOFER_PROXIMITY constant
    //                 then we will trigger the following command. */
    //                         && swerve
    //                                 .getOdometry()
    //                                 .relativeTo(
    //                                     /* Get the 2D location of the April tag that is in the
    // center of the subwoofer for our alliance.
    //                                     convert 3d to 2d
    //                                     If the robot holds a note and the location of the robot
    // is with a specified distance of the
    //                                     center subwoofer April tag then pre-move the arm to the
    // shooting position and partially spin
    //                                     up the shooter motors. */
    //                                     Constants2024.FieldConstants.getInstance()
    //                                         .aprilTags
    //                                         .getTagPose(
    //                                             Constants2024.FieldConstants.getInstance()
    //                                                 .northSpeakerTag) // get 3D pose of our
    //                                         // alliance's
    //                                         // center subwoofer tag
    //                                         .get() // extract from optional type
    //                                         .toPose2d())
    //                                 .getTranslation()
    //                                 .getNorm()
    //                             < SUBWOOFER_PROXIMITY)
    //             .whileTrue(
    //                 LogCommand.parallel(
    //                     arm.getMoveToCommand(Arm.Position.SPEAKER),
    //                     shooter.prepSubwooferCommand())));
  }

  private void configureLEDAnimations() {
    new Trigger(RobotState::isEnabled)
        .whileTrue(ledPriorities.toggleState(LEDMode.Vortex.Priorities.ENABLED));
    new Trigger(
            () ->
                !EpsilonUtil.epsilonEquals(
                    arm.getRequestedPosition(), Arm.Position.CROUCH.getRadians()))
        .whileTrue(ledPriorities.toggleState(LEDMode.Vortex.Priorities.ARM_ACTIVE));
    new Trigger(() -> !EpsilonUtil.epsilonEquals(climber.getRequestedPosition(), 0))
        .onTrue(
            ledPriorities
                .toggleState(LEDMode.Vortex.Priorities.CLIMBING)
                .raceWith(
                    new WaitUntilCommand(RobotState::isDisabled)
                        .andThen(new WaitUntilCommand(RobotState::isEnabled)))
                .ignoringDisable(true));
  }

  protected void addSubsystems() {
    super.addSubsystems();
    for (Limelight limelight : limelights) {
      subsystems.add(limelight);
    }
    subsystems.add(climber);
    subsystems.add(intake);
    subsystems.add(arm);
    subsystems.add(shooter);
    subsystems.add(ledController);
  }

  private Pair<Double, Double> getLinearContribution(
      Rotation2d lineAngle, Supplier<Pair<Double, Double>> positionSupplier) {
    Pair<Double, Double> position = positionSupplier.get();
    Rotation2d controllerAngle =
        Rotation2d.fromDegrees(
            (Math.toDegrees(Math.atan2(position.getSecond(), position.getFirst())) + 270) % 360);
    double controllerHypotenuse =
        Math.sqrt(Math.pow(position.getFirst(), 2) + Math.pow(position.getSecond(), 2));
    Rotation2d anglediff =
        Rotation2d.fromDegrees(
            ((controllerAngle.getDegrees() - lineAngle.getDegrees()) + 360) % 360);
    double lineDistance = Math.cos(anglediff.getRadians()) * controllerHypotenuse;
    return new Pair<Double, Double>(
        Math.sin(lineAngle.getRadians()) * lineDistance,
        Math.cos(lineAngle.getRadians()) * lineDistance);
  }

  protected void configureButtonBindings() {
    super.configureButtonBindings();

    // Auto move to specific locations on field
    driverJoystick
        .rightBumper()
        .whileTrue(
            Shoot.getTouchAprilTagCommand(
                () -> Constants2024.FieldConstants.getInstance().ampTag,
                1,
                0,
                new Rotation2d(),
                swerve));
    // rotate to scoring location angles
    driverJoystick
        .x()
        .whileTrue(
            rotateToAngle(() -> new Rotation2d(Constants2024.FieldConstants.SUBWOOFER_LEFT_ANGLE)));
    driverJoystick.y().whileTrue(rotateToAngle(() -> new Rotation2d(0))); // subwoofer center
    driverJoystick
        .b()
        .whileTrue(
            rotateToAngle(
                () -> new Rotation2d(Constants2024.FieldConstants.SUBWOOFER_RIGHT_ANGLE)));
    driverJoystick // amp
        .a()
        .whileTrue(
            rotateToAngle(
                () ->
                    (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
                        ? new Rotation2d(Math.PI / -2)
                        : new Rotation2d(Math.PI / 2)));

    // Do we want to add additional intelligence to handle the case where a note is picked up but
    // the driver is still
    // holding the button (in which case the NN will cause the robot's rotation to alter toward
    // another note - if one
    // is present)?
    driverJoystick
        .leftTrigger()
        .whileTrue(
            LogCommand.parallel(
                    getNewHeadingSwerveDriveCommand(),
                    new StandardDrive(
                        swerve,
                        DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
                        getDriverLeftStickInput(),
                        () -> fieldRelative))
                .until(() -> nnVision.hasTargets())
                .andThen(
                    new DoneCycleCommand<>(
                            LogCommand.parallel(
                                rotateToAngle(
                                    () -> nnVision.nnGetAngleToTarget(swerve.getOdometry()), true),
                                new StandardDrive(
                                    swerve,
                                    Constants.DriveTrainConstants.SWERVE_DRIVE_MAX_SPEED,
                                    () ->
                                        fieldRelative
                                            ? new Pair<>(driverJoystick.getLeftY() * -1, 0.0)
                                            : getLinearContribution(
                                                nnVision.nnGetAngleToTarget(swerve.getOdometry()),
                                                getDriverLeftStickInput()),
                                    () -> fieldRelative)),
                            true)
                        .withDoneCycles(
                            DoneCycleMachine.supplierWithMinCycles(
                                () -> !nnVision.hasTargets(), 5)))
                .repeatedly());

    // Climber
    buttonBox
        .button(8)
        .whileTrue(
            new InstantCommand(() -> climber.setRatchet(false))
                .andThen(climber.getMoveDownCommand().withTimeout(1))
                .andThen(climber.getMoveToCommand(Climber.Position.UP)));
    buttonBox
        .button(7)
        .whileTrue(
            new InstantCommand(() -> climber.setRatchet(true))
                .alongWith(climber.getMoveToCommand(Climber.Position.DOWN)));
    // Intake
    buttonBox
        .button(5)
        .whileTrue(
            intake
                .intakeCommand(RollerState.FORWARD)
                .deadlineWith(shooter.reverseCommand())
                .withName("IntakeIn"));
    buttonBox.button(4).whileTrue(intakeWithBB());
    buttonBox
        .button(9)
        .whileTrue(
            intakeWithBB().asProxy().andThen(Shoot.fixNotePosition(intake, shooter).asProxy()));
    buttonBox.button(13).whileTrue(Shoot.fixNotePosition(intake, shooter));
    buttonBox.button(3).whileTrue(intake.intakeCommand(RollerState.REVERSE).withName("IntakeOut"));
    // Shooter
    buttonBox
        .button(15)
        .whileTrue(
            LogCommand.parallel(
                    Shoot.subwooferShot(shooter, arm, intake),
                    new LockedSwerveDrive(swerve, LockedSwerveDrive.LockedMode.XShape))
                .withName("SubwooferShot"));
    buttonBox.button(14).whileTrue(Shoot.ampShot(shooter, arm, intake).withName("AmpShot"));
    buttonBox
        .button(16)
        .whileTrue(
            Shoot.podiumShot(
                    arm,
                    intake,
                    shooter,
                    swerve,
                    gyro,
                    new ProfiledPIDController(
                        prefs.getDouble("AutoTurnP"),
                        0,
                        0,
                        new TrapezoidProfile.Constraints(Constants.TAU, Constants.TAU)),
                    () -> armTrackOffset.get())
                .withName("PodiumShot"));
    buttonBox
        .button(1)
        .whileTrue(
            LogCommand.parallel(
                    arm.getMoveToCommand(Arm.Position.SPEAKER), shooter.prepSubwooferCommand())
                .withName("ShooterPrep"));
    // Arm
    buttonBox.button(12).onTrue(arm.getMoveToCommand(Arm.Position.AMP));
    buttonBox.button(11).onTrue(arm.getMoveToCommand(Arm.Position.SPEAKER));
    buttonBox.button(2).onTrue(arm.getMoveToCommand(Arm.Position.INTAKE));

    driverJoystick
        .leftBumper()
        .whileTrue(AmpAutoScore.from(swerve, limelights[0], arm, shooter, intake));

    // Coast button (on robot)
    new DigitalInputTrigger("CoastButtonID", prefs)
        .get()
        .negate()
        .and(RobotState::isDisabled)
        .whileTrue(
            Commands.startEnd(
                    () -> {
                      arm.setCoastMode(true);
                      climber.setCoastMode(true);
                      intake.setCoastMode(true);
                    },
                    () -> {
                      arm.setCoastMode(false);
                      climber.setCoastMode(false);
                      intake.setCoastMode(false);
                    })
                .ignoringDisable(true));

    // Shot tracking
    buttonBox
        .button(6)
        .whileTrue(
            Shoot.trackArmAndDrive(
                arm,
                intake,
                shooter,
                swerve,
                gyro,
                autoPidControllers.thetaPidController,
                ShooterConstants.SPEED,
                () -> armTrackOffset.get()));
    buttonBox.button(10).whileTrue(Shoot.shot(shooter, arm, intake));

    // Subsystem stops
    driverJoystick
        .povDown()
        .onTrue(
            new InstantCommand(() -> subsystems.forEach(subsystem -> subsystem.setSStopped(true)))
                .ignoringDisable(true));
    driverJoystick
        .povUp()
        .onTrue(
            new InstantCommand(() -> subsystems.forEach(subsystem -> subsystem.setSStopped(false)))
                .ignoringDisable(true));
  }

  private void configureDemoModeBindings() {
    super.configureButtonBindings();

    // Climber
    if (demoLevel == DemoLevel.NEW_DRIVER) {
      driverJoystick
          .povUp()
          .onTrue(
              new InstantCommand(
                  () -> ledPriorities.enableState(LEDMode.Vortex.Priorities.CLIMBING)));
      driverJoystick
          .povDown()
          .onTrue(
              new InstantCommand(
                  () -> ledPriorities.disableState(LEDMode.Vortex.Priorities.CLIMBING)));
    } else {
      driverJoystick
          .povUp()
          .whileTrue(
              new InstantCommand(() -> climber.setRatchet(false))
                  .andThen(climber.getMoveDownCommand().withTimeout(1))
                  .andThen(climber.getMoveToCommand(Climber.Position.UP)));
      driverJoystick
          .povDown()
          .whileTrue(
              new InstantCommand(() -> climber.setRatchet(true))
                  .alongWith(climber.getMoveToCommand(Climber.Position.DOWN)));
    }
    // Intake
    driverJoystick.leftBumper().whileTrue(intakeWithBB());
    driverJoystick
        .leftTrigger()
        .whileTrue(
            intakeWithBB().asProxy().andThen(Shoot.fixNotePosition(intake, shooter).asProxy()));
    driverJoystick.b().whileTrue(intake.intakeCommand(RollerState.REVERSE).withName("IntakeOut"));
    // Shooter
    driverJoystick
        .a()
        .whileTrue(
            LogCommand.parallel(
                    Shoot.subwooferShot(shooter, arm, intake),
                    new LockedSwerveDrive(swerve, LockedSwerveDrive.LockedMode.XShape))
                .withName("SubwooferShot"));
    driverJoystick.x().whileTrue(Shoot.ampShot(shooter, arm, intake).withName("AmpShot"));
    // Coast button (on robot)
    new DigitalInputTrigger("CoastButtonID", prefs)
        .get()
        .negate()
        .and(RobotState::isDisabled)
        .whileTrue(
            Commands.startEnd(
                    () -> {
                      arm.setCoastMode(true);
                      climber.setCoastMode(true);
                      intake.setCoastMode(true);
                    },
                    () -> {
                      arm.setCoastMode(false);
                      climber.setCoastMode(false);
                      intake.setCoastMode(false);
                    })
                .ignoringDisable(true));

    // Shot tracking
    driverJoystick
        .rightBumper()
        .whileTrue(
            Shoot.trackArmAndDrive(
                arm,
                intake,
                shooter,
                swerve,
                gyro,
                autoPidControllers.thetaPidController,
                ShooterConstants.SPEED,
                () -> armTrackOffset.get()));
    driverJoystick.rightTrigger().whileTrue(Shoot.shot(shooter, arm, intake));
  }

  private Command intakeWithBB() {
    return intake
        .loadWithSensor()
        .deadlineWith(shooter.reverseCommand(), arm.getMoveToCommand(Arm.Position.INTAKE))
        .withName("IntakeInWithSensor")
        .andThen(
            new ScheduleCommand(
                /*Commands.parallel(
                Commands.startEnd(
                        () -> driverJoystick.rumble(0.3),
                        () -> driverJoystick.rumble(0))
                    .withTimeout(1),*/
                Commands.startEnd(
                        () ->
                            forEachSecondaryLimelight(
                                limelight -> limelight.setLed(VisionLEDMode.kBlink)),
                        () ->
                            forEachSecondaryLimelight(
                                limelight -> limelight.setLed(VisionLEDMode.kOff)))
                    .withTimeout(0.5)));
  }

  private RotateToAngle rotateToAngle(Supplier<Rotation2d> desiredAngle) {
    return rotateToAngle(desiredAngle, false);
  }

  private RotateToAngle rotateToAngle(
      Supplier<Rotation2d> desiredAngle, boolean updateContinuously) {
    return new RotateToAngle(
        swerve, gyro, autoPidControllers.thetaPidController, desiredAngle, updateContinuously);
  }

  private void forEachSecondaryLimelight(Consumer<Limelight> consumer) {
    for (int i = 1; i < limelights.length; i++) {
      consumer.accept(limelights[i]);
    }
  }

  @Override
  public void onDashboardInit(DashboardServer dashboard) {
    super.onDashboardInit(dashboard);

    dashboard.registerControlsAction("vortex");

    GroupWidget root = dashboard.getWebsite().getWidgets();
    TabsWidget tabs = new TabsWidget();
    root.addWidget(tabs);

    tabs.addTab("Main", createMainDashboardTab(dashboard));
    tabs.addTab("Recovery", createRecoveryDashboardTab(dashboard));

    dashboard.registerActionHandler("music", new MusicAction(conductor));
  }

  private Widget createMainDashboardTab(DashboardServer dashboard) {
    GroupWidget main = new GroupWidget(GroupWidget.GroupWidgetDirection.HORIZONTAL, true, true);

    GroupWidget leftGroup = new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, true, true);
    GroupWidget rightGroup =
        new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, false, true);
    main.addWidget(leftGroup);
    main.addWidget(rightGroup);

    // TODO: Make the Limelight class know the url and pass in the limelight instance here instead
    leftGroup.addWidget(CameraWidget.forLimelight(10));

    GroupWidget cameraIndicators =
        new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, false, false);
    cameraIndicators.setSizeLocked(true);
    leftGroup.addWidget(cameraIndicators);
    cameraIndicators.addWidget(
        ConnectionIndicatorWidget.forNTEntry("Dragon LL", "/limelight/hb")
            .withRestartButton(() -> limelights[0].restart()));
    cameraIndicators.addWidget(
        ConnectionIndicatorWidget.forNTEntry("Right LL", "/limelight-right/hb")
            .withRestartButton(() -> limelights[1].restart()));
    cameraIndicators.addWidget(
        ConnectionIndicatorWidget.forNTEntry("Left LL", "/limelight-left/hb")
            .withRestartButton(() -> limelights[2].restart()));
    cameraIndicators.addWidget(
        ConnectionIndicatorWidget.forIP("Intake PI", "{13}:1182|1183/stream.mjpg"));

    GroupWidget robotIndicatorsAndIntakeCamera;
    String intakeCameraIP = prefs.getString("IntakeCameraIP");

    if (intakeCameraIP.isEmpty()) {
      robotIndicatorsAndIntakeCamera = leftGroup;
    } else {
      robotIndicatorsAndIntakeCamera =
          new GroupWidget(GroupWidget.GroupWidgetDirection.HORIZONTAL, true, true);
      leftGroup.addWidget(robotIndicatorsAndIntakeCamera);
    }

    GroupWidget robotIndicators =
        new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, false, false);
    robotIndicators.setSizeLocked(true);
    robotIndicatorsAndIntakeCamera.addWidget(robotIndicators);
    for (SwerveParameters.ModulePosition pos : SwerveParameters.ModulePosition.values()) {
      robotIndicators.addWidget(
          ConnectionIndicatorWidget.forBoolean(
              pos.getExpandedName() + " Encoder", swerve.getModule(pos)::isEncoderConnected));
    }

    if (!intakeCameraIP.isEmpty()) {
      robotIndicatorsAndIntakeCamera.addWidget(new CameraWidget(intakeCameraIP, true));
    }

    rightGroup.addWidget(
        new FieldWidget("Field", FieldWidget.FieldYear.Y2024)); // Might want to make our own widget

    GroupWidget options = new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, false, false);
    rightGroup.addWidget(options);

    options.addWidget(
        new ButtonWidget(
            "Reset Gyro",
            () -> {
              gyro.reset();
              gyro.setYawAdjustment(Rotation2d.fromDegrees(0));
              swerve.resetOdometry(
                  new Pose2d(swerve.getOdometry().getTranslation(), gyro.getYaw()));
            }));

    options.addWidget(
        new ButtonWidget(
            "Take LL Snapshot",
            () -> {
              for (Limelight limelight : limelights) {
                limelight.snapshotCommand().schedule();
              }
            }));

    for (AutonomousMode mode : AutonomousMode.values()) {
      autoModeChooser.addOption(mode.getNiceName(), mode);
    }

    DropdownWidget<AutonomousMode> autoChooser =
        new DropdownWidget<>(
            "Auto mode", autonomousChooser.getAutoModeChooserKey(), autoModeChooser);
    options.addWidget(autoChooser);

    options.addWidget(new CheckSelectionsWidget("Check Selections", dashboard, autoChooser));

    SliderWidget armTrackOffset = new SliderWidget("Arm Track/Podium Offset (deg)", 0, -20, 20);
    this.armTrackOffset = () -> Math.toRadians(armTrackOffset.getValue());
    options.addWidget(armTrackOffset);

    options.addWidget(
        new ButtonWidget(
            "Move Shooter to Horizontal",
            () -> arm.getMoveToCommand(Arm.Position.HORIZONTAL_SHOOTER).schedule()));

    ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
    competitionTab.add("Auto mode", autoModeChooser).withSize(2, 1).withPosition(0, 1);

    return main;
  }

  @SuppressWarnings("null") // The SwitchWidget's value cannot be null
  private Widget createRecoveryDashboardTab(DashboardServer dashboard) {
    GroupWidget recovery = new GroupWidget(GroupWidgetDirection.VERTICAL, false, false);

    GroupWidget sStop = new GroupWidget(GroupWidget.GroupWidgetDirection.HORIZONTAL, true, false);
    recovery.addWidget(sStop);

    GroupWidget subsystems =
        new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, false, false);
    sStop.addWidget(subsystems);
    subsystems.addWidget(new DisplayWidget("Subsystems", DisplayWidget.Size.COMPACT));
    subsystems.addWidget(
        new SwitchWidget("S-Stop Arm", false)
            .followEnabled(arm::isSStopped)
            .addListener(arm::setSStopped));
    subsystems.addWidget(
        new SwitchWidget("S-Stop Climber", false)
            .followEnabled(climber::isSStopped)
            .addListener(climber::setSStopped));
    subsystems.addWidget(
        new SwitchWidget("S-Stop Intake", false)
            .followEnabled(intake::isSStopped)
            .addListener(intake::setSStopped));
    subsystems.addWidget(
        new SwitchWidget("S-Stop Shooter", false)
            .followEnabled(shooter::isSStopped)
            .addListener(shooter::setSStopped));

    GroupWidget drive = new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, false, false);
    sStop.addWidget(drive);
    drive.addWidget(new DisplayWidget("Drive", DisplayWidget.Size.COMPACT));
    drive.addWidget(
        new SwitchWidget("S-Stop Drive", false)
            .followEnabled(swerve::isSStopped)
            .addListener(swerve::setSStopped));
    drive.addWidget(
        new SwitchWidget("S-Stop Front Left", false)
            .followEnabled(() -> swerve.isSpecificallySStopped(ModulePosition.FRONT_LEFT))
            .addListener(value -> swerve.setSStopped(ModulePosition.FRONT_LEFT, value)));
    drive.addWidget(
        new SwitchWidget("S-Stop Front Right", false)
            .followEnabled(() -> swerve.isSpecificallySStopped(ModulePosition.FRONT_RIGHT))
            .addListener(value -> swerve.setSStopped(ModulePosition.FRONT_RIGHT, value)));
    drive.addWidget(
        new SwitchWidget("S-Stop Back Left", false)
            .followEnabled(() -> swerve.isSpecificallySStopped(ModulePosition.BACK_LEFT))
            .addListener(value -> swerve.setSStopped(ModulePosition.BACK_LEFT, value)));
    drive.addWidget(
        new SwitchWidget("S-Stop Back Right", false)
            .followEnabled(() -> swerve.isSpecificallySStopped(ModulePosition.BACK_RIGHT))
            .addListener(value -> swerve.setSStopped(ModulePosition.BACK_RIGHT, value)));

    recovery.addWidget(new ButtonWidget("Zero Swerve", swerve::zeroSensors));
    recovery.addWidget(
        new ButtonWidget(
            "Restart All Limelights",
            () -> {
              for (Limelight limelight : limelights) {
                limelight.restart();
              }
            }));
    recovery.addWidget(
        new SwitchWidget("Rainbow", false)
            .addListener(
                enabled -> ledPriorities.setState(LEDMode.Vortex.Priorities.RAINBOW, enabled)));

    recovery.addWidget(
        new DisplayWidget(
            new String(
                dashboard.getWebsite().getFile("controls/vortex.txt"), StandardCharsets.UTF_8),
            DisplayWidget.Size.TINY));

    recovery.addWidget(new SpacerWidget());

    return recovery;
  }

  @Override
  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand(autoModeChooser.getSelected());
  }
}
