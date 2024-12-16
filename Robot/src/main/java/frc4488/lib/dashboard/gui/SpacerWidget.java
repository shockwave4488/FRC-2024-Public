package frc4488.lib.dashboard.gui;

import java.io.IOException;
import org.msgpack.core.MessagePacker;

public class SpacerWidget extends Widget {

  public SpacerWidget() {
    super("spacer");
    super.setSizeLocked(true);
  }

  @Override
  public Widget setSizeLocked(boolean sizeLocked) {
    if (!sizeLocked) {
      throw new UnsupportedOperationException("SpacerWidgets must be size locked!");
    }
    return this;
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {}
}
