package shockwave.leds.modes;

import shockwave.leds.LEDs;

public interface Mode {
  public void set(LEDs leds, int arg);

  public default boolean onNewArg(LEDs leds, int arg) {
    return false;
  }
}
