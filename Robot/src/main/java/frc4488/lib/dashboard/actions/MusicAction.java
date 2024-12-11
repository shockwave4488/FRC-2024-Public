package frc4488.lib.dashboard.actions;

import frc4488.lib.dashboard.DashboardServer;
import frc4488.lib.devices.Conductor;

public class MusicAction implements Action {

  private final Conductor conductor;

  public MusicAction(Conductor conductor) {
    this.conductor = conductor;
  }

  @Override
  public String getUsage() {
    return "play <song> | pause | resume | stop";
  }

  @Override
  public void onCall(DashboardServer server, String[] args) {
    if (args.length == 0) {
      server.errorln("Usage: music " + getUsage() + "\n" + getSongsList());
      return;
    }

    switch (args[0]) {
      case "play" -> {
        if (args.length == 1) {
          server.errorln("Usage: music play <song>\n" + getSongsList());
          return;
        }
        StringBuilder songBuilder = new StringBuilder(args[1]);
        for (int i = 2; i < args.length; i++) {
          songBuilder.append(' ');
          songBuilder.append(args[i]);
        }
        String song = songBuilder.toString();
        if (conductor.play(song)) {
          server.println("Playing: " + song);
        } else {
          server.errorln("Unknown song: " + song + "\n" + getSongsList());
        }
      }
      case "pause" -> {
        if (args.length > 1) {
          server.errorln("Usage: music pause");
          return;
        }
        if (conductor.pause()) {
          server.println("Song paused");
        } else {
          server.errorln("No song was playing");
        }
      }
      case "resume" -> {
        if (args.length > 1) {
          server.errorln("Usage: music resume");
          return;
        }
        if (conductor.resume()) {
          server.println("Song resumed");
        } else {
          server.errorln("No song to resume");
        }
      }
      case "stop" -> {
        if (args.length > 1) {
          server.errorln("Usage: music stop");
          return;
        }
        if (conductor.stop()) {
          server.println("Song stopped");
        } else {
          server.errorln("No song to stop");
        }
      }
      default -> {
        server.errorln("Usage: music " + getUsage() + "\n" + getSongsList());
      }
    }
  }

  private String getSongsList() {
    StringBuilder list = new StringBuilder("Options:");
    for (String song : conductor.getSongs()) {
      list.append("\n - ");
      list.append(song);
    }
    return list.toString();
  }
}
