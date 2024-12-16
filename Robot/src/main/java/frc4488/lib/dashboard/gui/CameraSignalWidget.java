package frc4488.lib.dashboard.gui;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import org.msgpack.core.MessagePacker;

public class CameraSignalWidget extends Widget {

  /**
   * @param hueMin The minimum hue [0-360] - will wrap around if hueMin > hueMax
   * @param hueMax The maximum hue [0-360]
   * @param satMin The minimum saturation [0-1]
   * @param satMax The maximum saturation [0-1]
   * @param valueMin The minimum value [0-1]
   * @param valueMax The maximum value [0-1]
   */
  public static record ColorRange(
      Object id,
      int hueMin,
      int hueMax,
      double satMin,
      double satMax,
      double valueMin,
      double valueMax) {
    private void sendData(MessagePacker packer) throws IOException {
      packer.packString(id.toString());
      packer.packInt(hueMin);
      packer.packInt(hueMax);
      packer.packDouble(satMin);
      packer.packDouble(satMax);
      packer.packDouble(valueMin);
      packer.packDouble(valueMax);
    }
  }

  private static record ColorRangeOnNT(ColorRange color, GenericSubscriber sub) {}

  private final String name;
  private final int cameraIndex;
  private final double crop;
  private final double matchingPixelPercent;
  private final ColorRange[] colors;
  private final Map<Object, ColorRangeOnNT> colorsOnNT;

  /**
   * Detects signals from the driver station's camera by counting pixels within the hsv range
   *
   * @param name Name of this signal
   * @param cameraIndex The camera to use (usually 0)
   * @param crop The percentage amount to decrease the width & height [0-1]
   * @param matchingPixelPercent The percentage of pixels required to activate the signal [0-1]
   * @param colors The colors to match
   */
  public CameraSignalWidget(
      String name,
      int cameraIndex,
      double crop,
      double matchingPixelPercent,
      ColorRange... colors) {
    super("camera_signal");
    this.name = name;
    this.cameraIndex = cameraIndex;
    this.crop = crop;
    this.matchingPixelPercent = matchingPixelPercent;
    this.colors = colors;
    this.colorsOnNT = new HashMap<>();
    for (int i = 0; i < colors.length; i++) {
      ColorRange color = colors[i];
      if (this.colorsOnNT.put(
              color.id(),
              new ColorRangeOnNT(
                  color,
                  NetworkTableInstance.getDefault()
                      .getTopic("/Dashboard/camera_signal_" + name + "/" + i)
                      .genericSubscribe(NetworkTableType.kBoolean.getValueStr())))
          != null) {
        throw new IllegalArgumentException("Two color ranges cannot have the same id!");
      }
    }
  }

  public boolean isSignaling(Object id) {
    return colorsOnNT.get(id).sub().getBoolean(false);
  }

  public Trigger getSignalTrigger(Object id) {
    return new Trigger(() -> isSignaling(id));
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {
    packer.packString("/Dashboard/camera_signal_" + name);
    packer.packInt(cameraIndex);
    packer.packDouble(crop);
    packer.packDouble(matchingPixelPercent);
    packer.packInt(colors.length);
    for (ColorRange color : colors) {
      color.sendData(packer);
    }
  }
}
