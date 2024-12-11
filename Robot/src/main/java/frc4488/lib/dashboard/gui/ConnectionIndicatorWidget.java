package frc4488.lib.dashboard.gui;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc4488.lib.dashboard.gui.GroupWidget.GroupWidgetDirection;
import java.io.IOException;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import org.msgpack.core.MessagePacker;

/** Uses AJAX requests to determine if the dashboard can connect to an IP */
public class ConnectionIndicatorWidget extends Widget {

  private static enum Type {
    IP,
    NT_HEARTBEAT,
    NT_BOOLEAN
  }

  /**
   * @param name
   * @param ip Uses the same system as {@link CameraWidget}
   */
  public static ConnectionIndicatorWidget forIP(String name, String ip) {
    return new ConnectionIndicatorWidget(name, Type.IP, ip);
  }

  /**
   * @param name
   * @param ntPath A path to a network table entry that changes regularly (no change indicates no
   *     connection)
   */
  public static ConnectionIndicatorWidget forNTEntry(String name, String ntPath) {
    return new ConnectionIndicatorWidget(name, Type.NT_HEARTBEAT, ntPath);
  }

  /**
   * @param name
   * @param connected Queried every robot loop
   */
  public static ConnectionIndicatorWidget forBoolean(String name, BooleanSupplier connected) {
    return new ConnectionIndicatorWidget(name, connected);
  }

  private final String name;
  private final Type type;
  private final String value;

  private ConnectionIndicatorWidget(String name, Type type, String value) {
    super("connection_indicator");
    this.name = name;
    this.type = type;
    this.value = value;
  }

  private ConnectionIndicatorWidget(String name, BooleanSupplier connected) {
    super("connection_indicator");
    this.name = name;
    this.type = Type.NT_BOOLEAN;
    this.value = "/Dashboard/connection_indicator_" + getId() + "/connected";

    GenericPublisher pub =
        NetworkTableInstance.getDefault()
            .getTopic(value)
            .genericPublish(NetworkTableType.kBoolean.getValueStr());
    AtomicBoolean prevConnected = new AtomicBoolean(connected.getAsBoolean());
    pub.setBoolean(prevConnected.getPlain());

    CommandScheduler.getInstance()
        .getDefaultButtonLoop()
        .bind(
            () -> {
              boolean newConnected = connected.getAsBoolean();
              if (prevConnected.getPlain() != newConnected) {
                pub.setBoolean(newConnected);
                prevConnected.setPlain(newConnected);
              }
            });
  }

  public Widget withRestartButton(Runnable restart) {
    GroupWidget group = new GroupWidget(GroupWidgetDirection.HORIZONTAL, true, false);
    group.addWidget(this);
    group.addWidget(new ButtonWidget("Restart", restart));
    return group;
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {
    packer.packString(name);
    packer.packByte((byte) type.ordinal());
    packer.packString(value);
  }
}
