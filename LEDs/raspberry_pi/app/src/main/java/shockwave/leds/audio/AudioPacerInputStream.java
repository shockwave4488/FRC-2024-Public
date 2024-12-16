package shockwave.leds.audio;

import java.io.IOException;
import java.io.InputStream;
import javax.sound.sampled.AudioInputStream;

/**
 * Delays the audio stream so that the data becomes available in real time rather than as fast as
 * possible
 */
public class AudioPacerInputStream extends InputStream {

  private final AudioInputStream audio;
  private final double bytesPerMillis;
  private final long startTime;
  private long totalBytesRead;

  public AudioPacerInputStream(AudioInputStream audio, long skipMillis, double speed)
      throws IOException {
    if (audio.getFormat().getSampleSizeInBits() % 8 != 0) {
      throw new IllegalArgumentException("Only multiples of 8 bit sample sizes supported!");
    }

    this.audio = audio;
    int frameSize = audio.getFormat().getSampleSizeInBits() / 8 * audio.getFormat().getChannels();
    this.bytesPerMillis = frameSize * audio.getFormat().getSampleRate() / 1000.0 * speed;
    this.startTime = System.currentTimeMillis() + (skipMillis < 0 ? -skipMillis : 0);
    this.totalBytesRead = 0;

    if (skipMillis > 0) {
      audio.skip((long) (skipMillis * bytesPerMillis));
    }
  }

  @Override
  public int read() throws IOException {
    if (available() == 0) {
      return -1;
    }
    return audio.read();
  }

  @Override
  public int read(byte[] buf, int off, int len) throws IOException {
    int bytesRead = audio.read(buf, off, Math.min(len, available()));
    if (bytesRead != -1) {
      totalBytesRead += bytesRead;
    }
    return bytesRead;
  }

  @Override
  public long skip(long n) throws IOException {
    int bytesSkipped = Math.min((int) n, available());
    totalBytesRead += bytesSkipped;
    return bytesSkipped;
  }

  @Override
  public int available() throws IOException {
    long time = Math.max(0, System.currentTimeMillis() - startTime);
    return Math.min((int) ((long) (time * bytesPerMillis) - totalBytesRead), audio.available());
  }

  @Override
  public void close() throws IOException {
    audio.close();
  }
}
