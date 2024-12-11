package frc4488.lib.dashboard.gui;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.io.IOException;
import org.msgpack.core.MessagePacker;
import org.msgpack.core.MessageUnpacker;

public class ButtonWidget extends Widget {

  private final String name;
  private final Runnable onClick;
  private final GenericPublisher clickIdPub;
  private int clickId;

  public ButtonWidget(String name, Runnable onClick) {
    super("button");
    this.name = name;
    this.onClick = onClick;
    this.clickIdPub =
        NetworkTableInstance.getDefault()
            .getTopic("/Dashboard/button_" + getId() + "/clickId")
            .genericPublish(NetworkTableType.kInteger.getValueStr());
    this.clickIdPub.setInteger(clickId);
    SmartDashboard.putData(
        name, new InstantCommand(this::click).ignoringDisable(true).withName(name));
  }

  public void click() {
    // This notifies the dashboard that the click was received
    clickIdPub.setInteger(++clickId);
    onClick.run();
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {
    packer.packString(name);
  }

  @Override
  public void handlePacket(MessageUnpacker msg) {
    click();
  }
}
