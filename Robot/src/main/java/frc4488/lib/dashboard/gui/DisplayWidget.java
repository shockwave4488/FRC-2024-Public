package frc4488.lib.dashboard.gui;

import java.io.IOException;
import org.msgpack.core.MessagePacker;

public class DisplayWidget extends Widget {

  public enum Size {
    /** Large text (header style) */
    DEFAULT,
    /** Large text that's centered vertically and doesn't include extra padding (a large label) */
    COMPACT,
    /** Normal sized text (1em) without padding that's centered vertically (a small label) */
    TINY
  }

  private final String value;
  private final Size size;

  /**
   * Display a string
   *
   * @param value The value to display, using {key} to reference a network table entry
   * @param size The size of the text
   */
  public DisplayWidget(String value, Size size) {
    super("display");
    this.value = value;
    this.size = size;
  }

  /**
   * Display a string
   *
   * @param value The value to display, using {key} to reference a network table entry
   */
  public DisplayWidget(String value) {
    this(value, Size.DEFAULT);
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {
    packer.packString(value);
    packer.packByte((byte) size.ordinal());
  }
}
