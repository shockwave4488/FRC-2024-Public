package frc4488.robot.robotspecifics.supercell;

import com.google.gson.JsonObject;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc4488.lib.commands.CommandUtil;
import frc4488.lib.commands.LogCommand;
import frc4488.lib.dashboard.DashboardServer;
import frc4488.lib.dashboard.gui.ButtonWidget;
import frc4488.lib.dashboard.gui.CameraWidget;
import frc4488.lib.dashboard.gui.DropdownWidget;
import frc4488.lib.dashboard.gui.FieldWidget;
import frc4488.lib.dashboard.gui.GroupWidget;
import frc4488.lib.logging.LogManager;
import frc4488.lib.misc.MatchUtil;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.lib.sensors.vision.Limelight;
import frc4488.lib.sensors.vision.VisionCamera.CameraPositionConstants;
import frc4488.robot.Robot;
import frc4488.robot.autonomous.modes.supercell.AutonomousChooser;
import frc4488.robot.commands.LEDs.SetLEDMode;
import frc4488.robot.commands.drive.LockedSwerveDrive;
import frc4488.robot.commands.drive.LockedSwerveDrive.LockedMode;
import frc4488.robot.commands.drive.StandardRotation;
import frc4488.robot.commands.drive.SwerveModifierCommand.SwerveModifier;
import frc4488.robot.commands.supercell.AutoScoreCommandBuilder;
import frc4488.robot.commands.supercell.arm.MoveArmWithPID;
import frc4488.robot.commands.supercell.drive.BalanceOnChargeStation;
import frc4488.robot.commands.supercell.drive.DriveAndBalanceOnChargeStation;
import frc4488.robot.commands.supercell.drive.DriveAndBalanceOnChargeStation.ApproachBehavior;
import frc4488.robot.commands.supercell.intake.HoldCone;
import frc4488.robot.commands.supercell.intake.IntakeCommand;
import frc4488.robot.constants.Constants.DriveTrainConstants;
import frc4488.robot.constants.Constants2023.FieldConstants;
import frc4488.robot.constants.Constants2023.GamePiece;
import frc4488.robot.constants.Constants2023.RobotConstants.ArmConstants.ArmSetpoint;
import frc4488.robot.robotspecifics.SwerveDriveRobotContainer;
import frc4488.robot.subsystems.SmartPCM;
import frc4488.robot.subsystems.leds.ArduinoLEDController;
import frc4488.robot.subsystems.leds.LEDController;
import frc4488.robot.subsystems.leds.LEDMode;
import frc4488.robot.subsystems.supercell.Arm;
import frc4488.robot.subsystems.supercell.Intake;
import frc4488.robot.subsystems.supercell.Intake.Speed;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SupercellRobotContainer extends SwerveDriveRobotContainer {
  private final Intake intake;
  private final Arm arm;
  private final Limelight limelight;
  private final SmartPCM smartPCM;
  private final LEDController ledController;
  private final AutonomousChooser autonomousChooser;
  private final SelectedConstants selectedConstants;

  /**
   * The robot container for our basic swerve drive robot, this is where all classes relevant to
   * this robot are created and where its default command(s) are set
   */
  public SupercellRobotContainer(PreferencesParser prefs, LogManager logger) {
    super(true, false, prefs, logger);

    smartPCM = new SmartPCM(prefs.getInt("PCM_ID"));
    ledController =
        new ArduinoLEDController(LEDMode.Supercell.seismic(), prefs.getIntArray("LedDIO"));

    selectedConstants = new SelectedConstants();

    intake = new Intake(prefs.getInt("IntakeRollerID"), prefs.getInt("IntakeTimeOfFlightID"));
    arm = new Arm(prefs, logger);

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
            arm,
            intake,
            limelight,
            gyro,
            autoPidControllers,
            trajConfig,
            selectedConstants,
            prefs,
            logger);

    smartPCM.setDefaultCommand(
        CommandUtil.indefiniteInstantCommand(smartPCM::startCompressor, smartPCM)
            .withName("CompressorCommand"));
    arm.setDefaultCommand(
        new MoveArmWithPID(arm, arm.getMinimumSetpoint()).withName("ArmDefaultCommand"));
    intake.setDefaultCommand(
        new IntakeCommand(intake, Speed.STOPPED).withName("IntakeDefaultCommand"));

    ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");

    if (!MatchUtil.isSimulated()) {
      UsbCamera usbCamera = CameraServer.startAutomaticCapture();
      usbCamera.setResolution(320, 180);
      if (RobotBase.isReal()) {
        usbCamera.setFPS(15);
      }
      competitionTab.add("Front camera", usbCamera).withSize(3, 3).withPosition(6, 2);
    }

    ShuffleboardLayout gyroResetGrid =
        competitionTab
            .getLayout("Gyro reset", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 1))
            .withSize(3, 1)
            .withPosition(0, 3);
    GenericEntry resetYawEntry = gyroResetGrid.add("Yaw (degrees)", 0).getEntry("double");
    gyroResetGrid.add(
        "Reset gyro",
        new InstantCommand(
                () -> {
                  gyro.reset();
                  gyro.setYawAdjustment(Rotation2d.fromDegrees(resetYawEntry.getDouble(0)));
                  swerve.resetOdometry(
                      new Pose2d(swerve.getOdometry().getTranslation(), gyro.getYaw()));
                })
            .ignoringDisable(true)
            .withName("ResetGyro"));

    FieldConstants.eagerlyInitialize();

    addSubsystems();
    configureButtonBindings();
  }

  protected void addSubsystems() {
    super.addSubsystems();
    subsystems.add(limelight);
    subsystems.add(smartPCM);
    subsystems.add(intake);
    subsystems.add(arm);
    subsystems.add(ledController);
  }

  protected void configureButtonBindings() {
    super.configureButtonBindings();

    driverJoystick.rightTrigger().toggleOnTrue(new LockedSwerveDrive(swerve, LockedMode.XShape));

    driverJoystick
        .leftTrigger()
        .whileTrue(
            new StartEndCommand(
                () -> swerve.setModifier(SwerveModifier.forSpeed(0.5)),
                swerve::clearModifier,
                swerve.modifierRequirement));

    driverJoystick
        .rightBumper()
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

    driverJoystick
        .a()
        .whileTrue(
            DriveAndBalanceOnChargeStation.create(
                swerve,
                gyro,
                ApproachBehavior.fromVelocity(swerve, () -> 1.8)
                    .withDelayTime(swerve, gyro, 2, 1.2)
                    .withRotation(
                        swerve, gyro, autoPidControllers.thetaPidController, new Rotation2d())));
    driverJoystick.b().whileTrue(BalanceOnChargeStation.create(swerve, gyro));

    // Arm Bindings
    buttonBox.button(1).toggleOnTrue(new MoveArmWithPID(arm, ArmSetpoint.PIECE_PICKUP));
    buttonBox.button(16).onTrue(new InstantCommand(() -> arm.raiseOffset()));
    buttonBox.button(15).onTrue(new InstantCommand(() -> arm.lowerOffset()));
    // Intake Bindings
    final Trigger CONE_PURGE_BUTTON = buttonBox.button(13);
    final Trigger CUBE_PURGE_BUTTON = buttonBox.button(12);
    final Trigger CONE_INTAKE_BUTTON = buttonBox.button(9);
    final Trigger CUBE_INTAKE_BUTTON = buttonBox.button(8);
    CONE_INTAKE_BUTTON.whileTrue(intakeConeWithHold(intake, prefs));
    CONE_INTAKE_BUTTON.whileTrue(new SetLEDMode(ledController, LEDMode.Supercell.solid(0xFF6600)));
    CONE_PURGE_BUTTON.whileTrue(IntakeCommand.out(intake, GamePiece.Cone));
    CUBE_INTAKE_BUTTON.whileTrue(IntakeCommand.in(intake, GamePiece.Cube).toDoneCycleCommand());
    CUBE_INTAKE_BUTTON.whileTrue(new SetLEDMode(ledController, LEDMode.Supercell.solid(0x9600E1)));
    CUBE_PURGE_BUTTON.whileTrue(IntakeCommand.out(intake, GamePiece.Cube));
    driverJoystick.y().whileTrue(IntakeCommand.launchCube(intake));
    buttonBox.button(14).whileTrue(IntakeCommand.launchCube(intake));

    // bindManualScoringCommands();
    bindAutoScoringCommands(
        driverJoystick.leftBumper().or(CONE_PURGE_BUTTON).or(CUBE_PURGE_BUTTON));

    // buttonBox
    //     .button(2)
    //     .whileTrue(
    //         LogCommand.endWhen(
    //             AlignToSubstation.from(
    //                 swerve,
    //                 m_gyro,
    //                 limelight,
    //                 arm,
    //                 autoPidControllers,
    //                 selectedConstants.substationSideChooser::getSelected),
    //             driverJoystick.rightBumper()));
    buttonBox.button(2).toggleOnTrue(new MoveArmWithPID(arm, ArmSetpoint.SUBSTATION));
  }

  @Override
  public void onDashboardInit(DashboardServer dashboard) {
    super.onDashboardInit(dashboard);

    dashboard.registerControlsAction("supercell");

    GroupWidget root = dashboard.getWebsite().getWidgets();

    GroupWidget leftGroup = new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, true, true);
    GroupWidget rightGroup =
        new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, false, true);
    root.addWidget(leftGroup);
    root.addWidget(rightGroup);

    leftGroup.addWidget(CameraWidget.forLimelight());
    leftGroup.addWidget(new CameraWidget("{2}:1181", true));

    rightGroup.addWidget(new FieldWidget("Field", FieldWidget.FieldYear.Y2023));

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
        new DropdownWidget<>(
            "Auto mode", "/Shuffleboard/Competition/Auto mode", selectedConstants.autoModeChooser));
    options.addWidget(
        new DropdownWidget<>(
            "Game piece 1",
            "/Shuffleboard/Competition/Game piece 1",
            selectedConstants.gamePieceChoosers.get(1)));
    options.addWidget(
        new DropdownWidget<>(
            "Game piece 2",
            "/Shuffleboard/Competition/Game piece 2",
            selectedConstants.gamePieceChoosers.get(2)));
    options.addWidget(
        new DropdownWidget<>(
            "Game piece 3",
            "/Shuffleboard/Competition/Game piece 3",
            selectedConstants.gamePieceChoosers.get(3)));
    options.addWidget(
        new DropdownWidget<>(
            "Game piece 4",
            "/Shuffleboard/Competition/Game piece 4",
            selectedConstants.gamePieceChoosers.get(4)));
    options.addWidget(
        new DropdownWidget<>(
            "Start game piece",
            "/Shuffleboard/Competition/Start game piece",
            selectedConstants.startPieceChooser));
    options.addWidget(
        new DropdownWidget<>(
            "Substation side",
            "/Shuffleboard/Competition/Substation side",
            selectedConstants.substationSideChooser));
  }

  public static Command intakeConeWithHold(Intake intake, PreferencesParser prefs) {
    return LogCommand.sequence(
        IntakeCommand.in(intake, GamePiece.Cone).toDoneCycleCommand(),
        new ScheduleCommand(HoldCone.basedOnPrefs(intake, prefs)));
  }

  @SuppressWarnings("unused")
  private void bindManualScoringCommands() {
    buttonBox.button(3).whileTrue(new MoveArmWithPID(arm, ArmSetpoint.LOW_SCORE));
    buttonBox.button(4).whileTrue(new MoveArmWithPID(arm, ArmSetpoint.MID_SCORE));
    buttonBox.button(5).whileTrue(new MoveArmWithPID(arm, ArmSetpoint.HIGH_SCORE));
  }

  private void bindAutoScoringCommands(BooleanSupplier shouldCancel) {
    buttonBox.button(5).toggleOnTrue(new MoveArmWithPID(arm, ArmSetpoint.LOW_SCORE));
    AutoScoreCommandBuilder.bindToButtonBox(
        swerve,
        gyro,
        intake,
        autoPidControllers,
        limelight,
        builder -> builder.withArm(arm),
        buttonBox,
        shouldCancel,
        new int[][] {new int[] {11, 7, 4}, new int[] {10, 6, 3}, new int[] {0, 0, 0}});
  }

  public Command getAutonomousCommand() {
    return LogCommand.sequence(
        new ScheduleCommand(HoldCone.basedOnPrefs(intake, prefs))
            .unless(() -> selectedConstants.startPieceChooser.getSelected() != GamePiece.Cone),
        autonomousChooser.getCommand(selectedConstants.autoModeChooser.getSelected()));
  }
}
