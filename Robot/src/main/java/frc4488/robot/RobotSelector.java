package frc4488.robot;

import frc4488.lib.IRobotContainer;
import frc4488.lib.logging.LeveledSmartDashboard;
import frc4488.lib.logging.LogLevel;
import frc4488.lib.logging.LogManager;
import frc4488.lib.preferences.PreferenceDoesNotExistException;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.robot.robotspecifics.bareroborio.BareRoboRIORobotContainer;
import frc4488.robot.robotspecifics.eruption.EruptionRobotContainer;
import frc4488.robot.robotspecifics.invalid.InvalidRobotContainer;
import frc4488.robot.robotspecifics.mock.MockRobotContainer;
import frc4488.robot.robotspecifics.spider.SpiderBotContainer;
import frc4488.robot.robotspecifics.supercell.SupercellRobotContainer;
import frc4488.robot.robotspecifics.swerve.SwerveRobotContainer;
import frc4488.robot.robotspecifics.vortex.VortexRobotContainer;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiFunction;

public class RobotSelector {
  public static final Map<String, BiFunction<PreferencesParser, LogManager, IRobotContainer>>
      ROBOTS;

  static {
    Map<String, BiFunction<PreferencesParser, LogManager, IRobotContainer>> robots =
        new HashMap<>();
    robots.put("Eruption", EruptionRobotContainer::new);
    robots.put("Spider", SpiderBotContainer::new);
    robots.put("Swerve", SwerveRobotContainer::new);
    robots.put("BareRoboRIO", BareRoboRIORobotContainer::new);
    robots.put("Supercell", SupercellRobotContainer::new);
    robots.put("Vortex", VortexRobotContainer::new);
    robots.put("Mock", MockRobotContainer::new);
    ROBOTS = Collections.unmodifiableMap(robots);
  }

  private final PreferencesParser prefs;
  private final LogManager logger;
  private final String robotName;

  public RobotSelector(PreferencesParser prefs, LogManager logger) {
    this.prefs = prefs;
    this.logger = logger;

    String name;
    try {
      name = prefs.getString("RobotName");
    } catch (PreferenceDoesNotExistException e) {
      name = "Invalid";
    }
    robotName = name;
    LeveledSmartDashboard.HIGH.putString("Selected Robot", robotName);
  }

  public IRobotContainer getRobot() {
    BiFunction<PreferencesParser, LogManager, IRobotContainer> robot = ROBOTS.get(robotName);
    if (robot == null) {
      logger.getMainLog().println(LogLevel.ERROR_CONSOLE, "Invalid RobotName value in prefs!");
      return new InvalidRobotContainer(prefs, logger);
    }
    return robot.apply(prefs, logger);
  }
}
