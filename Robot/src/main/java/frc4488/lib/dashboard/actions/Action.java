package frc4488.lib.dashboard.actions;

import frc4488.lib.dashboard.DashboardServer;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

public interface Action {
  public static Action create(String usage, BiConsumer<DashboardServer, String[]> callback) {
    return new Action() {
      @Override
      public String getUsage() {
        return usage;
      }

      @Override
      public void onCall(DashboardServer server, String[] args) {
        callback.accept(server, args);
      }
    };
  }

  public static Action create(String usage, Consumer<String[]> callback) {
    return create(usage, (dashboard, args) -> callback.accept(args));
  }

  public String getUsage();

  public void onCall(DashboardServer server, String[] args);
}
