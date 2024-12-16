package shockwave.leds.modes;

import java.io.BufferedInputStream;
import java.io.IOException;
import java.io.InputStream;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.UnsupportedAudioFileException;
import shockwave.leds.EQ;
import shockwave.leds.LEDs;
import shockwave.leds.audio.AudioProcessor;
import shockwave.leds.canvas.LEDCanvas;
import shockwave.leds.canvas.LEDCloner;
import shockwave.leds.canvas.LEDFill;
import shockwave.leds.controllers.MusicLEDController;
import shockwave.leds.controllers.SequenceLEDController;

public class MusicMode implements Mode {

  public static final long SHIFT =
      0; // More negative delays the LEDs, more positive skips forward a bit
  public static final double SPEED = 0.985;
  private static final int NUM_SAMPLES_PER_PIECE = 128;
  private static final int NUM_SAMPLES_TOTAL = 1024;
  private static final int AMPLITUDE_EXP = 10;

  @Override
  public void set(LEDs leds, int arg) {
    try {
      int songId = (arg >> 24);
      long skipMillis = (arg & 0xFF_FFFF) + leds.getMusicShift();

      InputStream song = MusicMode.class.getResourceAsStream("/songs/song" + songId + ".wav");
      AudioInputStream audio = AudioSystem.getAudioInputStream(new BufferedInputStream(song));
      AudioProcessor processor =
          new AudioProcessor(
              audio,
              skipMillis,
              leds.getMusicSpeed(),
              NUM_SAMPLES_PER_PIECE,
              NUM_SAMPLES_TOTAL,
              AMPLITUDE_EXP);

      LEDCanvas solidColorCanvas =
          new LEDCloner(
              new LEDFill(leds.getArmLeftStrip()),
              new LEDFill(leds.getArmRightStrip()),
              new LEDFill(leds.getDragonStrip()));
      EQ eq = new EQ(6, 6, new LEDCloner(leds.getFrameLeftStrip(), leds.getFrameRightStrip()));

      leds.getThread()
          .addController(
              new SequenceLEDController(
                  new MusicLEDController(solidColorCanvas, eq, processor), leds::done));
    } catch (IOException | UnsupportedAudioFileException e) {
      e.printStackTrace();
    }
  }
}
