package frc4488.robot.robotspecifics.invalid;

import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc4488.lib.BaseRobotContainer;
import frc4488.lib.dashboard.DashboardServer;
import frc4488.lib.dashboard.gui.ButtonWidget;
import frc4488.lib.dashboard.gui.DropdownWidget;
import frc4488.lib.dashboard.gui.GroupWidget;
import frc4488.lib.logging.LogLevel;
import frc4488.lib.logging.LogManager;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.robot.RobotSelector;
import java.io.IOException;
import java.nio.file.Files;

public class InvalidRobotContainer extends BaseRobotContainer {

  public InvalidRobotContainer(PreferencesParser prefs, LogManager logger) {
    super(prefs, logger);
  }

  public Command getAutonomousCommand() {
    return null;
  }

  protected void addSubsystems() {}

  @Override
  public void onDashboardInit(DashboardServer dashboard) {
    super.onDashboardInit(dashboard);

    GroupWidget root = dashboard.getWebsite().getWidgets();
    root.setDirection(GroupWidget.GroupWidgetDirection.VERTICAL);
    root.setStretched(false);
    root.setBordered(false);

    DropdownWidget<String> dropdown = new DropdownWidget<>("Robot");
    for (String robot : RobotSelector.ROBOTS.keySet()) {
      dropdown.addOption(robot, "robotspecifics/" + robot);
    }
    root.addWidget(dropdown);
    root.addWidget(
        new ButtonWidget(
            "Set Robot",
            () -> {
              dashboard.println(
                  "Setting '"
                      + (RobotBase.isReal() ? "" : "simulation/")
                      + "configDir.txt' to '"
                      + dropdown.getSelected()
                      + "'");
              try {
                Files.writeString(PreferencesParser.CONFIG_DIR_TXT, dropdown.getSelected());
                exit(); // Restart with new robot
              } catch (IOException e) {
                logger
                    .getMainLog()
                    .println(LogLevel.ERROR_CONSOLE, "Error while setting configDir.txt", e);
              }
            }));
  }

  @SuppressFBWarnings(value = "DM_EXIT", justification = "Used to restart the robot")
  // @SuppressFBWarnings doesn't work on lambdas (https://github.com/spotbugs/spotbugs/issues/724)
  private void exit() {
    System.exit(0);
  }
}
