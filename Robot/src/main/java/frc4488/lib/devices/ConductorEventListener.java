package frc4488.lib.devices;

public interface ConductorEventListener {
  public default void onPlay(String song) {}

  public default void onPause() {}

  public default void onResume() {}

  public default void onStop() {}
}
