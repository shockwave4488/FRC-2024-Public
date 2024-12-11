package shockwave.leds.canvas;

import java.awt.Color;
import java.util.Map;
import java.util.TreeMap;

public class LEDConcat implements LEDCanvas {

  private final TreeMap<Integer, LEDCanvas> leds;
  private final int numLeds;

  public LEDConcat(LEDCanvas... leds) {
    this.leds = new TreeMap<>();
    int offset = 0;
    for (LEDCanvas strip : leds) {
      this.leds.put(offset, strip);
      offset += strip.getNumLeds();
    }
    this.numLeds = offset;
  }

  @Override
  public int getNumLeds() {
    return numLeds;
  }

  @Override
  public void setRGB(int ledIndex, int r, int g, int b) {
    Map.Entry<Integer, LEDCanvas> entry = leds.floorEntry(ledIndex);
    entry.getValue().setRGB(ledIndex - entry.getKey(), r, g, b);
  }

  @Override
  public Color getColor(int ledIndex) {
    Map.Entry<Integer, LEDCanvas> entry = leds.floorEntry(ledIndex);
    return entry.getValue().getColor(ledIndex - entry.getKey());
  }
}
