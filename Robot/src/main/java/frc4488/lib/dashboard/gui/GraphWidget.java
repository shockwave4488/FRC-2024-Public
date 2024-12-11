package frc4488.lib.dashboard.gui;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import java.io.IOException;
import org.msgpack.core.MessagePacker;

public class GraphWidget extends Widget {

  private final String name;
  private final String path;
  private GenericPublisher publisher;

  public GraphWidget(String name, String path) {
    super("graph");
    this.name = name;
    this.path = path;
  }

  public GraphWidget(String name) {
    this(name, "/SmartDashboard/" + name);
  }

  public void addValue(NetworkTableValue value) {
    if (publisher == null) {
      publisher =
          NetworkTableInstance.getDefault()
              .getTopic(path)
              .genericPublish(value.getType().getValueStr());
    }
    publisher.set(value);
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {
    packer.packString(name);
    packer.packString(path);
  }
}
