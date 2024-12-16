package shockwave.leds.controllers;

import java.io.IOException;
import java.util.stream.IntStream;
import shockwave.leds.EQ;
import shockwave.leds.audio.AudioProcessor;
import shockwave.leds.canvas.LEDCanvas;

public class MusicLEDController implements LEDController {

  private static final float PITCH_SCALAR = 0.005f;
  private static final float PITCH_MAX = 0.75f;
  private static final float SOLID_VOLUME_SCALAR = 0.5f;
  private static final float EQ_VOLUME_SCALAR = 0.01f;

  private final LEDCanvas solidColorCanvas;
  private final EQ eq;
  private final AudioProcessor processor;

  public MusicLEDController(LEDCanvas solidColorCanvas, EQ eq, AudioProcessor processor) {
    this.solidColorCanvas = solidColorCanvas;
    this.eq = eq;
    this.processor = processor;
  }

  @Override
  public void render() {
    float pitch = Math.max(0, Math.min(PITCH_MAX, processor.getPitch() * PITCH_SCALAR));
    float volume = Math.max(0, Math.min(1, processor.getVolume() * SOLID_VOLUME_SCALAR));
    solidColorCanvas.fillHSV(pitch, 1, volume);

    float[] spectrum = processor.getSpectrum();
    int eqColumnSize = spectrum.length / eq.getWidth();
    for (int column = 0; column < eq.getWidth(); column++) {
      double columnVolume =
          IntStream.range(column * eqColumnSize, (column + 1) * eqColumnSize)
              .mapToDouble(i -> spectrum[i] * i)
              .average()
              .orElse(0);
      eq.setLevel(column, columnVolume * EQ_VOLUME_SCALAR);
    }
  }

  @Override
  public boolean isFinished() {
    boolean finished = processor.isFinished();
    if (finished) {
      try {
        processor.close();
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
    return finished;
  }
}
