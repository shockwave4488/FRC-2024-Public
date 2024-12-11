package frc4488.lib.dashboard;

import frc4488.lib.logging.Console;
import frc4488.robot.Robot;
import java.util.Optional;

public interface DashboardInitializer {

  public default Optional<DashboardServer> getDashboard() {
    Console console = Robot.getConsole();
    if (console instanceof DashboardServer dashboard) {
      return Optional.of(dashboard);
    }
    return Optional.empty();
  }

  public default void onDashboardInit(DashboardServer dashboard) {}
}
