package shockwave.leds.modes;

import shockwave.leds.Colors;
import shockwave.leds.LEDs;
import shockwave.leds.canvas.LEDStrip;

public class BlankMode implements Mode {

  @Override
  public void set(LEDs leds, int arg) {
    for (LEDStrip strip : leds.getStrips()) {
      strip.fillRGB(Colors.BLACK);
    }
  }
}
