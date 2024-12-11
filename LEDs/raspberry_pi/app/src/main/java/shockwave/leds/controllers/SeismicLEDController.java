package shockwave.leds.controllers;

import shockwave.leds.canvas.CheckedLEDCanvas;
import shockwave.leds.canvas.LEDCanvas;

/** Ported from the Arduino version, which is ported from the legacy Arduino version */
public class SeismicLEDController extends RepeatingLEDController {

  private static final int YELLOW = 0xFFC800;
  private static final int RED = 0xFF0000;
  private static final int DIM_RED = 0x320000;
  private static final int ORANGE = 0xFF1E00;

  private static class FadeIn implements LEDController {
    private static final float ITERATION_AMOUNT =
        0.0625f * 5; // Previous animation ran at ~300fps, so 300/60 = 5x the speed

    private final LEDCanvas canvas;
    private float progress;

    public FadeIn(LEDCanvas canvas) {
      this.canvas = canvas;
    }

    @Override
    public void render() {
      float i = progress;
      int ledCount = canvas.getNumLeds();

      // 1.1, 1.2, & 1.3 are from the old Arduino code and make some colors move faster than others
      canvas.setRGB((int) i, YELLOW);
      canvas.setRGB(
          (int) (Math.pow(i, 1.1) - (Math.pow((ledCount / 2), 1.1) - ledCount / 2)), ORANGE);
      canvas.setRGB((int) (Math.pow(i, 1.2) - (Math.pow((ledCount / 2), 1.2) - ledCount / 2)), RED);
      canvas.setRGB(
          (int) (Math.pow(i, 1.3) - (Math.pow((ledCount / 2), 1.3) - ledCount / 2)), DIM_RED);
      canvas.setRGB((int) (ledCount + 1 - i), YELLOW);
      canvas.setRGB(
          (int)
              (ledCount + 1 - (Math.pow(i, 1.1) - (Math.pow((ledCount / 2), 1.1) - ledCount / 2))),
          ORANGE);
      canvas.setRGB(
          (int)
              (ledCount + 1 - (Math.pow(i, 1.2) - (Math.pow((ledCount / 2), 1.2) - ledCount / 2))),
          RED);
      canvas.setRGB(
          (int)
              (ledCount + 1 - (Math.pow(i, 1.3) - (Math.pow((ledCount / 2), 1.3) - ledCount / 2))),
          DIM_RED);

      progress +=
          ITERATION_AMOUNT * canvas.getNumLeds() / 48.0f; // Previous animation based on 48 leds
    }

    @Override
    public boolean isFinished() {
      return progress > canvas.getNumLeds() / 2;
    }
  }

  private static class FadeOut implements LEDController {
    private final LEDCanvas canvas;
    private int progress;

    public FadeOut(LEDCanvas canvas) {
      this.canvas = canvas;
      this.progress = canvas.getNumLeds() / 2;
    }

    @Override
    public void render() {
      int i = progress;

      chameleon2(i);
      canvas.setRGB(i + 8, DIM_RED);
      chameleon2(canvas.getNumLeds() + 1 - i);
      canvas.setRGB(canvas.getNumLeds() + 1 - i - 8, DIM_RED);

      progress--;
    }

    // For each 1/6: 0xFA0000, 0x966400, 0xFF6400, 0xFF6400, 0x966400, 0xFA0000
    private void chameleon2(int index) {
      int ledCount = canvas.getNumLeds();
      if (index < (ledCount / 6)) {
        canvas.setRGB(index, 0xFA0000);
      } else if (index < (ledCount / 3)) {
        canvas.setRGB(index, 0x966400);
      } else if (index < (ledCount * 2 / 3)) {
        canvas.setRGB(index, 0xFF6400);
      } else if (index < (ledCount * 5 / 6)) {
        canvas.setRGB(index, 0x966400);
      } else if (index < ledCount) {
        canvas.setRGB(index, 0xFA0000);
      }
    }

    @Override
    public boolean isFinished() {
      return progress < -8;
    }
  }

  public SeismicLEDController(LEDCanvas canvas) {
    this(
        new CheckedLEDCanvas(
            canvas)); // Original legacy code wasn't designed to avoid out of range indicies
  }

  private SeismicLEDController(CheckedLEDCanvas canvas) {
    super(
        () ->
            new SequenceLEDController(
                new FadeIn(canvas), new FadeOut(canvas), new WaitLEDController(500)));
  }
}
