package shockwave.leds.audio;

import java.io.Closeable;
import java.io.IOException;
import javax.sound.sampled.AudioInputStream;
import shockwave.leds.audio.analysis.FFT;

public class AudioProcessor implements Closeable {

  /**
   * The maximum number of bytes behind the processor can get before it skips back to the current
   * time. If this is getting consistently hit, the numSamplesPerPiece and numSamplesTotal
   * parameters should be changed so that the Raspberry PI can keep up
   */
  private static final int MAX_LAG = 50000;

  private final AudioInputStream audio;
  private final Thread thread;
  private volatile boolean interrupted;

  private final float[] spectrum;
  private volatile float volume;
  private volatile float pitch;

  /**
   * @param audio The audio stream
   * @param skipMillis The number of milliseconds to skip at the beginning of the audio
   * @param speed Changes the speed of the song (1 is normal)
   * @param numSamplesPerPiece The number of samples that should be read on each iteration
   * @param numSamplesTotal The total number of samples considered when performing the FFT
   * @param amplitudeExp Makes the LEDs more strongly respond to spikes in a specific frequency
   */
  public AudioProcessor(
      AudioInputStream audio,
      long skipMillis,
      double speed,
      int numSamplesPerPiece,
      int numSamplesTotal,
      int amplitudeExp) {
    if (numSamplesTotal % numSamplesPerPiece != 0) {
      throw new IllegalArgumentException(
          "numSamplesTotal must be an integer multiple of numSamplesPerPiece!");
    }

    this.audio = audio;
    this.spectrum = new float[numSamplesTotal / 2 + 1];

    thread =
        new Thread(
            () -> {
              try {
                AudioPacerInputStream pacedAudio =
                    new AudioPacerInputStream(audio, skipMillis, speed);
                AudioDecoder decoder = new AudioDecoder(pacedAudio, audio.getFormat());
                FFT fft = new FFT(numSamplesTotal, audio.getFormat().getSampleRate());
                float[] samples = new float[numSamplesTotal];
                float[] samplesPiece = new float[numSamplesPerPiece];

                while (decoder.readCombinedSamples(samplesPiece) > 0
                    && !Thread.interrupted()
                    && !interrupted) {
                  if (pacedAudio.available() > MAX_LAG) {
                    System.out.println("Dropping audio!");
                    pacedAudio.skip(pacedAudio.available());
                  }

                  System.arraycopy(
                      samples, numSamplesPerPiece, samples, 0, samples.length - numSamplesPerPiece);
                  System.arraycopy(
                      samplesPiece,
                      0,
                      samples,
                      samples.length - numSamplesPerPiece,
                      numSamplesPerPiece);

                  fft.forward(samples);
                  float[] spectrum = fft.getSpectrum();
                  synchronized (this.spectrum) {
                    System.arraycopy(spectrum, 0, this.spectrum, 0, this.spectrum.length);
                  }

                  float sum = 0;
                  float weightedSum = 0;
                  float weightedArea = 0;
                  for (int i = 0; i < spectrum.length; i++) {
                    sum += spectrum[i];
                    weightedSum += i * Math.pow(spectrum[i], amplitudeExp);
                    weightedArea += Math.pow(spectrum[i], amplitudeExp);
                  }
                  volume = sum / spectrum.length;
                  pitch = weightedSum / weightedArea;
                }
              } catch (Exception e) {
                e.printStackTrace();
              }
            },
            "AudioProcessor");
    thread.start();
  }

  public float[] getSpectrum() {
    synchronized (spectrum) {
      float[] output = new float[spectrum.length];
      System.arraycopy(spectrum, 0, output, 0, spectrum.length);
      return output;
    }
  }

  public float getVolume() {
    return volume;
  }

  public float getPitch() {
    return pitch;
  }

  public boolean isFinished() {
    return !thread.isAlive();
  }

  @Override
  public void close() throws IOException {
    interrupted = true;
    thread.interrupt();
    audio.close();
  }
}
