package frc4488.lib.dashboard.gui;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.io.IOException;
import org.msgpack.core.MessagePacker;

public class SliderWidget extends Widget {

  private final String name;
  private final double defaultValue;
  private final double min;
  private final double max;
  private final GenericSubscriber value;

  public SliderWidget(String name, double value, double min, double max) {
    super("slider");
    this.name = name;
    this.defaultValue = value;
    this.min = min;
    this.max = max;
    this.value =
        NetworkTableInstance.getDefault()
            .getTopic("/Dashboard/slider_" + getId() + "/value")
            .genericSubscribe();
  }

  public double getValue() {
    return value.getDouble(defaultValue);
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {
    packer.packString(name);
    packer.packDouble(defaultValue);
    packer.packDouble(min);
    packer.packDouble(max);
  }
}
