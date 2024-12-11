package shockwave.leds.modes;

import shockwave.leds.LEDs;
import shockwave.leds.canvas.LEDCanvas;
import shockwave.leds.canvas.LEDCloner;
import shockwave.leds.canvas.LEDReverser;
import shockwave.leds.controllers.RainbowLEDController;

public class ClimbingMode implements Mode {

  private static final int RAINBOW_CYCLE_TIME = 500;

  @Override
  public void set(LEDs leds, int arg) {
    LEDCanvas frame = new LEDCloner(leds.getFrameLeftStrip(), leds.getFrameRightStrip());
    LEDCanvas arm = new LEDReverser(new LEDCloner(leds.getArmLeftStrip(), leds.getArmRightStrip()));

    leds.getThread().addController(new RainbowLEDController(arm, RAINBOW_CYCLE_TIME));
    leds.getThread().addController(new RainbowLEDController(frame, RAINBOW_CYCLE_TIME));
    leds.getThread()
        .addController(new RainbowLEDController(leds.getDragonStrip(), RAINBOW_CYCLE_TIME));
  }
}
