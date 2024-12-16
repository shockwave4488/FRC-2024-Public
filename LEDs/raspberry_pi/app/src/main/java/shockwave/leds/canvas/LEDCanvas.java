package shockwave.leds.canvas;

import java.awt.Color;

public interface LEDCanvas {
  public int getNumLeds();

  public void setRGB(int ledIndex, int r, int g, int b);

  public Color getColor(int ledIndex);

  public default void setRGB(int ledIndex, int rgb) {
    setRGB(ledIndex, (rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF);
  }

  public default void setHSV(int ledIndex, float h, float s, float v) {
    setRGB(ledIndex, Color.HSBtoRGB(h, s, v));
  }

  public default void fade(int ledIndex, double factor) {
    Color color = getColor(ledIndex);
    setRGB(
        ledIndex,
        (int) (color.getRed() * factor),
        (int) (color.getGreen() * factor),
        (int) (color.getBlue() * factor));
  }

  public default void rangeRGB(int start, int len, int r, int g, int b) {
    for (int ledIndex = start; ledIndex < start + len; ledIndex++) {
      setRGB(ledIndex, r, g, b);
    }
  }

  public default void rangeRGB(int start, int len, int rgb) {
    rangeRGB(start, len, (rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF);
  }

  public default void rangeHSV(int start, int len, float h, float s, float v) {
    rangeRGB(start, len, Color.HSBtoRGB(h, s, v));
  }

  public default void rangeFade(int start, int len, double factor) {
    for (int ledIndex = start; ledIndex < start + len; ledIndex++) {
      fade(ledIndex, factor);
    }
  }

  public default void fillRGB(int r, int g, int b) {
    rangeRGB(0, getNumLeds(), r, g, b);
  }

  public default void fillRGB(int rgb) {
    rangeRGB(0, getNumLeds(), rgb);
  }

  public default void fillHSV(float h, float s, float v) {
    rangeHSV(0, getNumLeds(), h, s, v);
  }

  public default void fillFade(double factor) {
    rangeFade(0, getNumLeds(), factor);
  }
}
