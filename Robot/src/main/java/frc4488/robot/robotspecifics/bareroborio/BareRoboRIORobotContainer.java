package frc4488.robot.robotspecifics.bareroborio;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc4488.lib.BaseRobotContainer;
import frc4488.lib.dashboard.DashboardServer;
import frc4488.lib.dashboard.gui.ButtonWidget;
import frc4488.lib.dashboard.gui.CameraSignalWidget;
import frc4488.lib.dashboard.gui.CameraSignalWidget.ColorRange;
import frc4488.lib.dashboard.gui.CameraWidget;
import frc4488.lib.dashboard.gui.ConnectionIndicatorWidget;
import frc4488.lib.dashboard.gui.DisplayWidget;
import frc4488.lib.dashboard.gui.DropdownWidget;
import frc4488.lib.dashboard.gui.FieldWidget;
import frc4488.lib.dashboard.gui.GraphWidget;
import frc4488.lib.dashboard.gui.GroupWidget;
import frc4488.lib.dashboard.gui.SliderWidget;
import frc4488.lib.dashboard.gui.SwitchWidget;
import frc4488.lib.dashboard.gui.TabsWidget;
import frc4488.lib.logging.LogManager;
import frc4488.lib.preferences.PreferencesParser;
import java.util.List;

public class BareRoboRIORobotContainer extends BaseRobotContainer {

  public BareRoboRIORobotContainer(PreferencesParser prefs, LogManager logger) {
    super(prefs, logger);
  }

  public Command getAutonomousCommand() {
    return null;
  }

  protected void addSubsystems() {}

  @SuppressWarnings("null")
  @Override
  public void onDashboardInit(DashboardServer dashboard) {
    super.onDashboardInit(dashboard);

    // Dummy field for testing dashboard widgets
    Field2d field = new Field2d();

    GroupWidget root = dashboard.getWebsite().getWidgets();
    root.setDirection(GroupWidget.GroupWidgetDirection.VERTICAL);

    GroupWidget header = new GroupWidget(GroupWidget.GroupWidgetDirection.HORIZONTAL, false, true);
    TabsWidget tabs = new TabsWidget();
    GroupWidget body = new GroupWidget(GroupWidget.GroupWidgetDirection.HORIZONTAL, true, true);
    header.setSizeLocked(true);
    root.addWidget(header);
    root.addWidget(tabs);
    tabs.addTab("Main", body);

    header.addWidget(new DisplayWidget("Commands: {/LiveWindow/Ungrouped/CommandLogger/Names}"));

    GroupWidget leftGroup = new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, true, true);
    GroupWidget rightGroup =
        new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, false, true);
    body.addWidget(leftGroup);
    body.addWidget(rightGroup);

    leftGroup.addWidget(CameraWidget.forLimelight());
    leftGroup.addWidget(new CameraWidget("{2}:1181", true));
    leftGroup.addWidget(new GraphWidget("Time", "/SmartDashboard/Time"));

    CameraSignalWidget signals =
        new CameraSignalWidget(
            "Signals",
            0,
            0.9,
            0.01,
            new ColorRange("Red", 360 - 30, 30, 0.2, 1, 0.06, 1),
            new ColorRange("Yellow", 30, 90, 0.2, 1, 0.06, 1),
            new ColorRange("Green", 130, 190, 0.2, 1, 0.06, 1),
            new ColorRange("Blue", 210, 270, 0.4, 1, 0.06, 1));
    signals
        .getSignalTrigger("Red")
        .and(RobotState::isAutonomous)
        .onTrue(new InstantCommand(() -> dashboard.println("Detected red")));
    signals
        .getSignalTrigger("Yellow")
        .and(RobotState::isAutonomous)
        .onTrue(new InstantCommand(() -> dashboard.println("Detected yellow")));
    signals
        .getSignalTrigger("Green")
        .and(RobotState::isAutonomous)
        .onTrue(new InstantCommand(() -> dashboard.println("Detected green")));
    signals
        .getSignalTrigger("Blue")
        .and(RobotState::isAutonomous)
        .onTrue(new InstantCommand(() -> dashboard.println("Detected blue")));
    leftGroup.addWidget(signals);

    rightGroup.addWidget(new FieldWidget("Field", FieldWidget.FieldYear.Y2023));
    field.getObject("Test2").setPose(5, 2.5, Rotation2d.fromDegrees(-135));
    field
        .getObject("[Trajectory] Traj")
        .setTrajectory(
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(45)),
                List.of(),
                new Pose2d(2.5, 5, Rotation2d.fromDegrees(90)),
                new TrajectoryConfig(5, 5)));

    GroupWidget options = new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, false, false);
    rightGroup.addWidget(options);

    DropdownWidget<String> dropdown = new DropdownWidget<>("SendableChooser[1]");
    dropdown.addOption("test", "test_value");
    dropdown.addOption("test2", "test2_value");
    options.addWidget(dropdown);

    options.addWidget(
        new ButtonWidget(
            "println(hi)",
            () -> {
              dashboard.println("hi");
              System.out.println("hi");
            }));

    SwitchWidget switch1 = new SwitchWidget("Switch1", false);
    SwitchWidget switch2 = new SwitchWidget("Switch2", true);
    switch1.addListener(switch2::setEnabled);
    options.addWidget(switch1);
    options.addWidget(switch2);

    SliderWidget slider = new SliderWidget("Slider", 1, 0.5, 2);
    options.addWidget(slider);

    options.addWidget(ConnectionIndicatorWidget.forIP("USB Camera", "{2}:1181"));

    GroupWidget tab2 = new GroupWidget(GroupWidget.GroupWidgetDirection.VERTICAL, false, false);
    tab2.addWidget(new SwitchWidget("Test", false));
    tabs.addTab("Tab 2", tab2);

    TabsWidget nestedTabs = new TabsWidget();
    nestedTabs.addTab("a", new SwitchWidget("A", true));
    nestedTabs.addTab("b", new SwitchWidget("B", true));
    tabs.addTab("Tab 3", nestedTabs);

    SmartDashboard.putData("Field", field);
    new Thread(
            () -> {
              while (true) {
                try {
                  Thread.sleep(20);
                } catch (InterruptedException e) {
                  return;
                }
                SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());
                SmartDashboard.putRaw("Raw", new byte[] {1, 2, 3});
                field
                    .getObject("Test")
                    .setPoses(
                        new Pose2d(5, 5, Rotation2d.fromRotations(0.25)),
                        new Pose2d(10, 5, Rotation2d.fromRotations(Timer.getFPGATimestamp() / 2)));
                System.out.println(
                    dropdown.getSelected()
                        + " - "
                        + switch1.isEnabled()
                        + " - "
                        + switch2.isEnabled()
                        + " - "
                        + slider.getValue());
              }
            })
        .start();
  }
}
