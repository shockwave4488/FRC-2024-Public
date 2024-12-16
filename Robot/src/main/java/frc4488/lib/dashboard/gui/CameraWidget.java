package frc4488.lib.dashboard.gui;

import java.io.IOException;
import org.msgpack.core.MessagePacker;

/**
 *
 *
 * <pre>
 * IP Layout:
 * - Dynamic IP: {##}:port for 10.44.88.##:port, 172.22.11.2:port, localhost:port
 * - WiFi vs Tether Port: {11}:AAAA|BBBB for 10.44.88.11:AAAA and 172.22.11.2:BBBB
 * - Paths: 10.44.88.60:1182/stream.mjpg for 10.44.88.60:1182/stream.mjpg
 * </pre>
 */
public class CameraWidget extends Widget {

  public static CameraWidget forLimelight(int ipEnding) {
    return new CameraWidget(
        "{" + ipEnding + "}:5800|1180",
        true,
        "{" + ipEnding + "}:5801|5800",
        "limelight.local:5801");
  }

  public static CameraWidget forLimelight() {
    return forLimelight(11);
  }

  private final String ip;
  private final boolean autoRestart;
  private final String link;
  private final String altLink;

  /**
   * If <code>autoRestart</code> is <code>false</code>, <code>link</code> cannot be specified as
   * both react to a click
   *
   * @param ip Refer to {@link CameraWidget}
   * @param autoRestart Causes the stream to be recreated once every second in case of a disconnect
   * @param link A link to open in a new tab when clicked (same format as ip)
   * @param altLink A link to open on Alt+Click (must already have a normal link)
   */
  public CameraWidget(String ip, boolean autoRestart, String link, String altLink) {
    super("camera");
    this.ip = ip;
    this.autoRestart = autoRestart;
    this.link = link;
    this.altLink = altLink;

    if (!autoRestart && link != null) {
      throw new IllegalArgumentException("There can't be a link when autoRestart is false!");
    }
    if (link == null && altLink != null) {
      throw new IllegalArgumentException("There can't be an altLink when link is null!");
    }
  }

  /**
   * @param ip Refer to {@link CameraWidget}
   * @param autoRestart Causes the stream to be recreated once every second in case of a disconnect
   */
  public CameraWidget(String ip, boolean autoRestart) {
    this(ip, autoRestart, null, null);
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {
    packer.packString(ip);
    packer.packBoolean(autoRestart);
    if (link == null) {
      packer.packByte((byte) 0);
    } else {
      if (altLink == null) {
        packer.packByte((byte) 1);
        packer.packString(link);
      } else {
        packer.packByte((byte) 2);
        packer.packString(link);
        packer.packString(altLink);
      }
    }
  }
}
