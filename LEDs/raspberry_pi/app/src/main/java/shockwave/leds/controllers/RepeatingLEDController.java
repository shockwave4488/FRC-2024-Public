package shockwave.leds.controllers;

import java.util.function.Supplier;

/**
 * Repeatedly renders an LEDController, re-creating it every time it completes (based on
 * RepeatCommand)
 */
public class RepeatingLEDController implements LEDController {

  private final Supplier<LEDController> supplier;
  private LEDController controller;

  public RepeatingLEDController(Supplier<LEDController> supplier) {
    this.supplier = supplier;
  }

  @Override
  public void render() {
    if (controller == null) {
      controller = supplier.get();
    }
    controller.render();
    if (controller.isFinished()) {
      controller = null;
    }
  }
}
