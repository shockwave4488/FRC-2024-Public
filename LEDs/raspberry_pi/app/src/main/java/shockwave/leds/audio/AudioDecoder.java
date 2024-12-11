package shockwave.leds.audio;

import java.io.IOException;
import java.io.InputStream;
import java.io.InterruptedIOException;
import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;

public class AudioDecoder {

  public static void checkSupported(AudioFormat format, boolean allowUnspecified)
      throws IllegalArgumentException {
    if (format.getEncoding() != AudioFormat.Encoding.PCM_UNSIGNED
        && format.getEncoding() != AudioFormat.Encoding.PCM_SIGNED) {
      throw new IllegalArgumentException("Only PCM_UNSIGNED and PCM_SIGNED supported!");
    }
    if (format.getSampleSizeInBits() != AudioSystem.NOT_SPECIFIED) {
      if (format.getSampleSizeInBits() % 8 != 0) {
        throw new IllegalArgumentException("Only multiples of 8 bit sample sizes supported!");
      }
      if (format.getSampleSizeInBits() > 32) {
        throw new IllegalArgumentException("Max sample size is 32 bits!");
      }
    }

    if (!allowUnspecified) {
      if (format.getSampleRate() == AudioSystem.NOT_SPECIFIED
          || format.getSampleSizeInBits() == AudioSystem.NOT_SPECIFIED
          || format.getChannels() == AudioSystem.NOT_SPECIFIED) {
        throw new IllegalArgumentException("Cannot have anything unspecified!");
      }
    }
  }

  public static boolean isSupported(AudioFormat format, boolean allowUnspecified) {
    try {
      checkSupported(format, allowUnspecified);
      return true;
    } catch (IllegalArgumentException e) {
      return false;
    }
  }

  private final InputStream in;
  private final AudioFormat format;
  private final long signedShift;
  private final long max;

  public AudioDecoder(InputStream in, AudioFormat format) {
    checkSupported(format, false);
    this.in = in;
    this.format = format;
    this.signedShift =
        (format.getEncoding() == AudioFormat.Encoding.PCM_UNSIGNED
            ? -1L << (format.getSampleSizeInBits() - 1)
            : 0);
    this.max = (1L << format.getSampleSizeInBits() - 1);
  }

  public AudioDecoder(AudioInputStream stream) {
    this(stream, stream.getFormat());
  }

  public int readSamples(double[] samples) {
    if (samples.length % format.getChannels() != 0) {
      throw new IllegalArgumentException(
          "The samples array length must be a multiple of the channel count! (Did you mean to use #readCombinedSamples(double[])?)");
    }

    byte[] raw = new byte[format.getSampleSizeInBits() / 8 * samples.length];
    try {
      if (in.readNBytes(raw, 0, raw.length) == 0) {
        return 0;
      }
    } catch (InterruptedIOException e) {
      return 0;
    } catch (IOException e) {
      e.printStackTrace();
      return 0;
    }

    for (int i = 0, rawI = 0; i < samples.length; i++, rawI += format.getSampleSizeInBits() / 8) {
      samples[i] = (double) (readRawInt(raw, rawI) + signedShift) / max;
    }

    return samples.length;
  }

  public int readCombinedSamples(double[] samples) {
    double[] separated = new double[samples.length * format.getChannels()];
    if (readSamples(separated) == 0) {
      return 0;
    }

    for (int sampleI = 0; sampleI < samples.length; sampleI++) {
      double sample = 0;
      for (int channelI = 0; channelI < format.getChannels(); channelI++) {
        sample += separated[sampleI * format.getChannels() + channelI];
      }
      samples[sampleI] = sample / format.getChannels();
    }

    return samples.length;
  }

  public int readSamples(float[] samples) {
    double[] doubleSamples = new double[samples.length];
    if (readSamples(doubleSamples) == 0) {
      return 0;
    }
    for (int i = 0; i < samples.length; i++) {
      samples[i] = (float) doubleSamples[i];
    }
    return samples.length;
  }

  public int readCombinedSamples(float[] samples) {
    double[] doubleSamples = new double[samples.length];
    if (readCombinedSamples(doubleSamples) == 0) {
      return 0;
    }
    for (int i = 0; i < samples.length; i++) {
      samples[i] = (float) doubleSamples[i];
    }
    return samples.length;
  }

  private long readRawInt(byte[] raw, int rawIndex) {
    // The bytes at position "rawIndex" in "raw" need to be combined into a single possibly unsigned
    // integer

    long output = 0; // Uses a long as ints are always signed
    for (int i = 0; i < format.getSampleSizeInBits() / 8; i++) {
      byte b = raw[i + rawIndex];
      long l = b;
      if (l < 0) { // Convert from a signed byte (-128 to 127) to an unsigned byte (0 to 255)
        l += 256;
      }
      if (format
          .isBigEndian()) { // Appends the new byte to the result, with ordering dependant on
                            // endianness
        output = (output << 8) + l; // Add the byte to the right of the current result
      } else {
        output |= (l << (i * 8)); // Add the byte to the left of the current result
      }
    }
    if (format.getEncoding() == AudioFormat.Encoding.PCM_SIGNED) {
      // Since the output should be signed in this case, it can be safely casted to an int
      // This also automatically converts the unsigned output into a signed output IF the sample
      // size is 4 bytes
      // Otherwise, this needs to check if the sign bit is 1, and if so, fill in the rest of the
      // output with 1s to make the output negative
      int intOutput = (int) output;
      if (format.getSampleSizeInBits() != 32
          && (output & (1 << (format.getSampleSizeInBits() - 1))) != 0) {
        intOutput |= 0xFFFFFFFF << format.getSampleSizeInBits(); // Negative; extend sign
      }
      // When casting from an int to a long, the sign is automatically extended
      output = (long) intOutput;
    }
    return output;
  }
}
