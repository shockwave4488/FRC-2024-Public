package shockwave.leds.modes;

import shockwave.leds.Colors;
import shockwave.leds.LEDs;
import shockwave.leds.canvas.LEDCanvas;
import shockwave.leds.canvas.LEDCloner;
import shockwave.leds.canvas.LEDReverser;
import shockwave.leds.controllers.DragonLEDController;
import shockwave.leds.controllers.EQController;
import shockwave.leds.controllers.FadeLEDController;
import shockwave.leds.controllers.MovingGradientLEDController;

public class EnabledMode implements Mode {

  private static final int ARM_GRADIENT_REPEATS = 3;
  private static final double ARM_MIN_FREQ = 1;
  private static final double ARM_MAX_FREQ = 4;
  private static final int DRAGON_FADE_MILLIS = 100;

  private DragonLEDController dragonController;
  private boolean angry;

  @Override
  public void set(LEDs leds, int arg) {
    angry = (arg != 0);

    LEDCanvas arm = new LEDReverser(new LEDCloner(leds.getArmLeftStrip(), leds.getArmRightStrip()));

    leds.getLeftEq().setColorRange(Colors.YELLOW, Colors.RED);
    leds.getRightEq().setColorRange(Colors.YELLOW, Colors.RED);
    leds.getThread().addController(new EQController(leds.getLeftEq(), () -> getEQLevel(leds)));
    leds.getThread().addController(new EQController(leds.getRightEq(), () -> getEQLevel(leds)));
    leds.getThread()
        .addController(
            new MovingGradientLEDController(
                arm, Colors.RED, 0, ARM_GRADIENT_REPEATS, () -> getArmMillis(leds)));
    leds.getThread()
        .addController(dragonController = new DragonLEDController(leds.getDragonStrip(), angry));
  }

  private double getEQLevel(LEDs leds) {
    return Math.max(leds.getArmPercent(), leds.getClimberPercent());
  }

  private int getArmMillis(LEDs leds) {
    if (leds.getClimberPercent() > 0.001) {
      return (int) (1000 / ARM_MAX_FREQ);
    }
    double intakePercent = leds.getIntakePercent();
    double shooterPercent = leds.getShooterPercent();
    double speed =
        (Math.abs(intakePercent) < Math.abs(shooterPercent) ? shooterPercent : intakePercent);
    return (int)
        (1000 / (speed * (ARM_MAX_FREQ - ARM_MIN_FREQ) + (speed >= 0 ? 1 : -1) * ARM_MIN_FREQ));
  }

  @Override
  public boolean onNewArg(LEDs leds, int arg) {
    boolean angry = (arg != 0);
    if (this.angry == angry) {
      return true;
    }
    this.angry = angry;

    dragonController.setAngry(angry);

    int startColor = DragonLEDController.MAIN_NORMAL;
    int endColor = DragonLEDController.MAIN_ANGRY;
    if (!angry) {
      int temp = startColor;
      startColor = endColor;
      endColor = temp;
    }
    // Gets added after the DragonController, so this "overrides" it temporarily
    leds.getThread()
        .addController(
            new FadeLEDController(leds.getDragonStrip(), startColor, endColor, DRAGON_FADE_MILLIS));

    return true;
  }
}
