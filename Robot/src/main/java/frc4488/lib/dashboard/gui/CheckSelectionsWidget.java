package frc4488.lib.dashboard.gui;

import frc4488.lib.dashboard.DashboardServer;
import java.io.IOException;
import org.msgpack.core.MessagePacker;
import org.msgpack.core.MessageUnpacker;

public class CheckSelectionsWidget extends Widget {

  private final String name;
  private final AbstractSelectionWidget<?>[] selections;
  private final DashboardServer dashboard;

  public CheckSelectionsWidget(
      String name, DashboardServer dashboard, AbstractSelectionWidget<?>... selections) {
    super("check_selections");
    this.name = name;
    this.dashboard = dashboard;
    this.selections = selections;
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {
    packer.packString(name);
    packer.packInt(selections.length);
    for (AbstractSelectionWidget<?> widget : selections) {
      packer.packLong(widget.getId());
    }
  }

  @Override
  public void handlePacket(MessageUnpacker msg) throws IOException {
    for (int i = 0; i < selections.length; i++) {
      if (!selections[i].getSelectedKey().equals(msg.unpackString())) {
        sendResult(false);
        return;
      }
    }
    sendResult(true);
  }

  private void sendResult(boolean success) {
    sendPacket(packer -> packer.packBoolean(success), dashboard);
  }
}
