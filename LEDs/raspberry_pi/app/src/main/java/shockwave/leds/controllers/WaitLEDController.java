package shockwave.leds.controllers;

/** Pauses a SequenceLEDController for a specified amount of time (based on WaitCommand) */
public class WaitLEDController implements LEDController {

  private final int millis;
  private long startTime;
  private long lastFrameTime;

  public WaitLEDController(int millis) {
    this.millis = millis;
    this.startTime = -1;
    this.lastFrameTime = -1;
  }

  @Override
  public void render() {
    lastFrameTime = System.currentTimeMillis();
    if (startTime == -1) {
      startTime = lastFrameTime;
    }
  }

  @Override
  public boolean isFinished() {
    return startTime + millis <= lastFrameTime;
  }
}
