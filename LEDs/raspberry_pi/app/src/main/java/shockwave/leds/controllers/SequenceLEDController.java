package shockwave.leds.controllers;

/**
 * Renders LEDControllers after each other, changing when the current controller's <code>
 * isFinished()</code> returns true (based on SequentialCommandGroup)
 */
public class SequenceLEDController implements LEDController {

  private final LEDController[] controllers;
  private int i;

  public SequenceLEDController(LEDController... controllers) {
    this.controllers = controllers;
  }

  @Override
  public void render() {
    if (i >= controllers.length) {
      return;
    }
    LEDController controller = controllers[i];
    controller.render();
    if (controller.isFinished()) {
      i++;
    }
  }

  @Override
  public boolean isFinished() {
    return i >= controllers.length;
  }
}
