package shockwave.leds.canvas;

import com.fazecast.jSerialComm.SerialPort;
import java.awt.Color;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;

public class LEDStrip implements LEDCanvas {

  public static LEDStrip nullStrip(int numLeds) {
    return new LEDStrip(numLeds, LEDEffect.RGB, OutputStream.nullOutputStream());
  }

  /** Keep in mind Arduinos can't start receiving information immediately */
  public static LEDStrip forSerialPort(String portName, int numLeds, LEDEffect effect)
      throws IOException {
    SerialPort port = SerialPort.getCommPort(portName);

    port.setBaudRate(1000000);
    port.setComPortTimeouts(SerialPort.TIMEOUT_READ_BLOCKING, Integer.MAX_VALUE, Integer.MAX_VALUE);
    if (!port.openPort()) {
      throw new IOException(
          "Failed to open port (" + port.getSystemPortName() + "): " + port.getLastErrorCode());
    }
    return new LEDStrip(numLeds, effect, port.getOutputStream());
  }

  public static interface LEDEffect {
    public static final LEDEffect RGB = (data, ledIndex) -> {};
    public static final LEDEffect GRB =
        (data, ledIndex) -> {
          int r = data[ledIndex];
          int g = data[ledIndex + 1];
          int b = data[ledIndex + 2];
          data[ledIndex] = g;
          data[ledIndex + 1] = r;
          data[ledIndex + 2] = b;
        };

    /**
     * RGB color codes are meant to be squared before rendering due to how vision works<br>
     * It seems the LEDs may not be applying this correction, so use this to apply it manually
     */
    public static final LEDEffect SQUARE =
        (data, ledIndex) -> {
          data[ledIndex] *= data[ledIndex] / 256.0;
          data[ledIndex + 1] *= data[ledIndex + 1] / 256.0;
          data[ledIndex + 2] *= data[ledIndex + 2] / 256.0;
        };

    public static LEDEffect colorCorrect(double rScalar, double gScalar, double bScalar) {
      return (data, ledIndex) -> {
        data[ledIndex] *= rScalar;
        data[ledIndex + 1] *= gScalar;
        data[ledIndex + 2] *= bScalar;
      };
    }

    public void modifyData(int[] data, int ledIndex);

    public default LEDEffect andThen(LEDEffect next) {
      return (data, ledIndex) -> {
        modifyData(data, ledIndex);
        next.modifyData(data, ledIndex);
      };
    }
  }

  private final int numLeds;
  private final LEDEffect effect;
  private final OutputStream output;
  private final int[] data;

  public LEDStrip(int numLeds, LEDEffect effect, OutputStream output) {
    this.numLeds = numLeds;
    this.effect = effect;
    this.output = output;
    this.data = new int[numLeds * 3];
  }

  public void show() throws IOException {
    int[] data = new int[this.data.length];
    System.arraycopy(this.data, 0, data, 0, data.length);
    for (int ledIndex = 0; ledIndex < data.length; ledIndex += 3) {
      effect.modifyData(data, ledIndex);
    }

    // Format (for every byte of data):
    // [toggle (1), zero (1), unreserved (2), data (4)] [toggle (1), unreserved (3), data (4)]
    // toggle: switches every byte of data (to sync)
    // zero: is false when sending the first byte of data, then changes to true (to sync)
    // data: the actual LED colors
    ByteArrayOutputStream buf = new ByteArrayOutputStream(data.length * 2);
    for (int i = 0; i < data.length; i++) {
      int toggleBit = ((i & 0b1) << 7);
      int zeroBit = ((i == 0 ? 0 : 1) << 6);
      int upperData = ((data[i] >> 4) & 0xF);
      int lowerData = (data[i] & 0xF);
      buf.write(toggleBit | zeroBit | upperData);
      buf.write(toggleBit | lowerData);
    }
    output.write(buf.toByteArray());
    output.flush();
  }

  @Override
  public int getNumLeds() {
    return numLeds;
  }

  @Override
  public void setRGB(int ledIndex, int r, int g, int b) {
    ledIndex *= 3;
    data[ledIndex] = r;
    data[ledIndex + 1] = g;
    data[ledIndex + 2] = b;
  }

  @Override
  public Color getColor(int ledIndex) {
    ledIndex *= 3;
    return new Color(data[ledIndex], data[ledIndex + 1], data[ledIndex + 2]);
  }
}
