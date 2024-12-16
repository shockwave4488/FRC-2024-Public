package shockwave.leds.modes;

import shockwave.leds.Colors;
import shockwave.leds.LEDs;
import shockwave.leds.canvas.LEDCanvas;
import shockwave.leds.canvas.LEDCloner;
import shockwave.leds.canvas.LEDReverser;
import shockwave.leds.controllers.DragonLEDController;
import shockwave.leds.controllers.FadeLEDController;
import shockwave.leds.controllers.ParallelLEDController;
import shockwave.leds.controllers.SequenceLEDController;
import shockwave.leds.controllers.SwipeLEDController;
import shockwave.leds.controllers.WaitLEDController;

public class StartupMode implements Mode {

  private static final int SWIPE_TIME = 1000;
  private static final int FADE_IN_TIME = 1000;
  private static final int FADE_HOLD_TIME = 1000;
  private static final int FADE_OUT_TIME = 1000;
  private static final int FADE_IN_2_TIME = 100;

  @Override
  public void set(LEDs leds, int arg) {
    LEDCanvas frame = new LEDCloner(leds.getLeftEqCompressed(), leds.getRightEqCompressed());
    LEDCanvas arm = new LEDReverser(new LEDCloner(leds.getArmLeftStrip(), leds.getArmRightStrip()));

    leds.getThread().addController(new SwipeLEDController(frame, Colors.GOLD, SWIPE_TIME));
    leds.getThread().addController(new SwipeLEDController(arm, Colors.GOLD, SWIPE_TIME));
    leds.getThread()
        .addController(
            new SequenceLEDController(
                new WaitLEDController(SWIPE_TIME),
                new FadeLEDController(
                    leds.getDragonStrip(),
                    Colors.BLACK,
                    Colors.RED,
                    FADE_IN_TIME,
                    FadeLEDController.EASE_IN_OUT_2),
                new WaitLEDController(FADE_HOLD_TIME),
                new ParallelLEDController(
                    new FadeLEDController(
                        leds.getDragonStrip(), Colors.RED, Colors.BLACK, FADE_OUT_TIME),
                    new FadeLEDController(frame, Colors.GOLD, Colors.BLACK, FADE_OUT_TIME),
                    new FadeLEDController(arm, Colors.GOLD, Colors.BLACK, FADE_OUT_TIME)),
                new FadeLEDController(
                    leds.getDragonStrip(),
                    Colors.BLACK,
                    DragonLEDController.MAIN_NORMAL,
                    FADE_IN_2_TIME),
                leds::done));
  }
}
