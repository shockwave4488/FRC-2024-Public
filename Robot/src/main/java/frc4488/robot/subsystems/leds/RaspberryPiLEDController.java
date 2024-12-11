package frc4488.robot.subsystems.leds;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.RobotBase;
import frc4488.lib.commands.CommandUtil;
import frc4488.lib.dashboard.DashboardInitializer;
import frc4488.lib.dashboard.DashboardServer;
import frc4488.robot.Robot;
import java.io.File;
import java.net.URL;
import java.net.URLClassLoader;
import java.util.EnumSet;

public class RaspberryPiLEDController extends LEDController implements DashboardInitializer {

  // MAKE SURE TO UPDATE THE RASPBERRY PI'S TableKeys TOO
  public static class TableKeys {
    public static final String MODE = "mode";
    public static final String DONE = "done";
    public static final String MUSIC_SHIFT = "music/shift";
    public static final String MUSIC_SPEED = "music/speed";
    public static final String ARM_PERCENT = "Arm Percent";
    public static final String CLIMBER_PERCENT = "Climber Percent";
    public static final String INTAKE_PERCENT = "Intake Percent";
    public static final String SHOOTER_PERCENT = "Shooter Percent";
  }

  public static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("LEDs");
  private static final File raspberryPiCode = new File("../LEDs/raspberry_pi");
  private static final File raspberryPiJar = new File(raspberryPiCode, "app/build/libs/app.jar");

  public static double getSubsystemPercent(
      double value, double requestedValue, double min, double max) {
    if (RobotBase.isSimulation()) {
      value = requestedValue;
    }
    return (value - min) / (max - min);
  }

  @SuppressWarnings("resource") // URLClassLoader needs to remain loaded
  private static void simLeds() throws Exception {
    Robot.getConsole().println("Building leds ...");
    Process ledBuild = Runtime.getRuntime().exec("gradlew.bat build", null, raspberryPiCode);
    while (ledBuild.isAlive()) {
      ledBuild.getInputStream().transferTo(Robot.getConsole().getOutputStream());
      ledBuild.getErrorStream().transferTo(Robot.getConsole().getErrorStream());
    }
    Robot.getConsole().println("Built leds!");

    URLClassLoader loader = new URLClassLoader(new URL[] {raspberryPiJar.toURI().toURL()});
    loader
        .loadClass("shockwave.leds.LEDs")
        .getMethod("main", String[].class)
        .invoke(null, (Object) new String[] {"simulation"});
  }

  private final GenericPublisher mode;

  public RaspberryPiLEDController(LEDMode defaultMode) {
    super(defaultMode);

    mode = TABLE.getTopic(TableKeys.MODE).genericPublish(NetworkTableType.kInteger.getValueStr());

    NetworkTableInstance.getDefault()
        .addListener(
            TABLE.getTopic(TableKeys.DONE).genericSubscribe(),
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> Robot.runOnMainThread(() -> CommandUtil.revertToDefault(this)));

    onModeChange(defaultMode);
  }

  @Override
  protected void onModeChange(LEDMode mode) {
    this.mode.setInteger(((long) mode.id() << 32) | mode.arg());
  }

  @Override
  public void onDashboardInit(DashboardServer dashboard) {
    if (RobotBase.isSimulation()) {
      dashboard.registerActionHandler(
          "leds",
          () -> {
            try {
              simLeds();
            } catch (Exception e) {
              Robot.getConsole().errorln("Failed to setup LED simulation!", e);
            }
          });
    }
  }
}
