package frc4488.robot.robotspecifics.mock;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc4488.lib.dashboard.DashboardServer;
import frc4488.lib.dashboard.gui.ButtonWidget;
import frc4488.lib.dashboard.gui.DisplayWidget;
import frc4488.lib.dashboard.gui.FieldWidget;
import frc4488.lib.dashboard.gui.GroupWidget;
import frc4488.lib.dashboard.gui.GroupWidget.GroupWidgetDirection;
import frc4488.lib.dashboard.gui.SpacerWidget;
import frc4488.lib.dashboard.gui.SwitchWidget;
import frc4488.lib.dashboard.gui.TabsWidget;
import frc4488.lib.dashboard.gui.Widget;
import frc4488.lib.drive.SwerveParameters.ModulePosition;
import frc4488.lib.logging.LogManager;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.robot.robotspecifics.SwerveDriveRobotContainer;
import frc4488.robot.subsystems.mock.Elevator;
import frc4488.robot.subsystems.mock.Intake;
import frc4488.robot.subsystems.mock.Intake.RollerState;

public class MockRobotContainer extends SwerveDriveRobotContainer {

  private final Elevator elevator;
  private final Intake intake;

  public MockRobotContainer(PreferencesParser prefs, LogManager logger) {
    super(true, true, prefs, logger);
    elevator = new Elevator(prefs);
    intake = new Intake(prefs);

    addSubsystems();
    configureButtonBindings();

    intake.setDefaultCommand(intake.intakeCommand(RollerState.OFF));
  }

  protected void addSubsystems() {
    super.addSubsystems();
    subsystems.add(elevator);
    subsystems.add(intake);
  }

  protected void configureButtonBindings() {
    super.configureButtonBindings();

    // Elevator
    buttonBox.button(16).onTrue(elevator.goToHeightCommand(Elevator.Position.TOP));
    buttonBox.button(14).onTrue(elevator.goToHeightCommand(Elevator.Position.BOTTOM));

    // Intake
    buttonBox.button(3).whileTrue(intake.intakeCommand(RollerState.FORWARD));
    buttonBox.button(4).whileTrue(intake.loadWithSensor());
    buttonBox.button(5).whileTrue(intake.intakeCommand(RollerState.REVERSE));
  }

  @Override
  public void onDashboardInit(DashboardServer dashboard) {
    super.onDashboardInit(dashboard);

    GroupWidget root = dashboard.getWebsite().getWidgets();
    TabsWidget tabs = new TabsWidget();
    root.addWidget(tabs);

    tabs.addTab("Main", createMainDashboardTab(dashboard));
    tabs.addTab("Recovery", createRecoveryDashboardTab(dashboard));
  }

  private Widget createMainDashboardTab(DashboardServer dashboard) {
    GroupWidget main = new GroupWidget(GroupWidget.GroupWidgetDirection.HORIZONTAL, true, true);

    GroupWidget leftGroup = new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, true, true);
    GroupWidget rightGroup =
        new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, false, true);
    main.addWidget(leftGroup);
    main.addWidget(rightGroup);

    rightGroup.addWidget(new FieldWidget("Field", FieldWidget.FieldYear.Y2024));

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

    recovery.addWidget(new SpacerWidget());

    return recovery;
  }

  @Override
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
