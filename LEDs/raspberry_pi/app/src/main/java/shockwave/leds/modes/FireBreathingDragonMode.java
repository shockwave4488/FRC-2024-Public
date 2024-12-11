package shockwave.leds.modes;

import java.util.concurrent.atomic.AtomicBoolean;
import shockwave.leds.Colors;
import shockwave.leds.LEDs;
import shockwave.leds.canvas.LEDCanvas;
import shockwave.leds.canvas.LEDConcat;
import shockwave.leds.canvas.LEDManager;
import shockwave.leds.canvas.LEDReverser;
import shockwave.leds.canvas.LEDStrip;
import shockwave.leds.controllers.DragonLEDController;
import shockwave.leds.controllers.FireIndicatorLEDController;
import shockwave.leds.controllers.FireLEDController;
import shockwave.leds.controllers.RandomPulseLEDController;
import shockwave.leds.controllers.SeismicLEDController;

public class FireBreathingDragonMode implements Mode {

  @Override
  public void set(LEDs leds, int arg) {
    LEDManager armLeftManager = new LEDManager(leds.getArmLeftStrip());
    LEDCanvas leftBottom =
        new LEDConcat(
            leds.getFrameLeftStrip(), armLeftManager.newSection().takeRangeLen(0, 5).build());
    LEDCanvas leftTop = armLeftManager.newSectionFromRemaining(false);

    LEDCanvas right = new LEDConcat(leds.getRightEqCompressed(), leds.getArmRightStrip());

    // Synchronize the fire
    AtomicBoolean startFire = new AtomicBoolean();
    long seed = System.currentTimeMillis();

    if (Math.random() < 0.5) {
      leds.getThread().addController(new RandomPulseLEDController(right, Colors.ORANGE));
    } else {
      leds.getThread().addController(new SeismicLEDController(right));
    }
    leds.getThread().addController(new DragonLEDController(leds.getDragonStrip(), false));
    leds.getThread()
        .addController(new FireIndicatorLEDController(LEDStrip.nullStrip(1), startFire::setPlain));
    leds.getThread()
        .addController(
            new FireLEDController(new LEDReverser(leftBottom), startFire::getPlain, seed));
    leds.getThread().addController(new FireLEDController(leftTop, startFire::getPlain, seed));
  }
}
