package shockwave.leds.controllers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Renders LEDControllers in parallel (based on ParallelCommandGroup) */
public class ParallelLEDController implements LEDController {

  private final List<LEDController> controllers;

  public ParallelLEDController(LEDController... controllers) {
    this.controllers = new ArrayList<>(Arrays.asList(controllers));
  }

  @Override
  public void render() {
    controllers.removeIf(
        controller -> {
          controller.render();
          return controller.isFinished();
        });
  }

  @Override
  public boolean isFinished() {
    return controllers.isEmpty();
  }
}
