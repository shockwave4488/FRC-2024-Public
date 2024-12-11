package frc4488.robot.robotspecifics.vortex;

import frc4488.lib.devices.Conductor;
import frc4488.lib.devices.ConductorEventListener;
import frc4488.lib.wpiextensions.PriorityManager;
import frc4488.robot.subsystems.leds.LEDMode;

public class LEDConductorEventListener implements ConductorEventListener {

  private final Conductor conductor;
  private final PriorityManager<LEDMode.Vortex.Priorities> ledController;
  private int songId;
  private long startTime;
  private int songTime;

  public LEDConductorEventListener(
      Conductor conductor, PriorityManager<LEDMode.Vortex.Priorities> ledController) {
    this.conductor = conductor;
    this.ledController = ledController;
  }

  @Override
  public void onPlay(String song) {
    songId = conductor.getSongs().indexOf(song);
    startTime = System.currentTimeMillis();
    songTime = 0;
    LEDMode.Vortex.Priorities.MUSIC.setMode(LEDMode.Vortex.music(songId, 0));
    ledController.disableState(
        LEDMode.Vortex.Priorities.MUSIC); // Restart the mode if it is running
    ledController.enableState(LEDMode.Vortex.Priorities.MUSIC);
  }

  @Override
  public void onPause() {
    songTime += System.currentTimeMillis() - startTime;
    ledController.disableState(LEDMode.Vortex.Priorities.MUSIC);
  }

  @Override
  public void onResume() {
    startTime = System.currentTimeMillis();
    LEDMode.Vortex.Priorities.MUSIC.setMode(LEDMode.Vortex.music(songId, songTime));
    ledController.enableState(LEDMode.Vortex.Priorities.MUSIC);
  }

  @Override
  public void onStop() {
    ledController.disableState(LEDMode.Vortex.Priorities.MUSIC);
  }
}
