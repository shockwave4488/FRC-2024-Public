package frc4488.lib.dashboard.gui;

import frc4488.lib.dashboard.DashboardServer;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.Base64;
import org.msgpack.core.MessagePack;
import org.msgpack.core.MessagePacker;
import org.msgpack.core.MessageUnpacker;

public abstract class Widget {

  @FunctionalInterface
  protected interface PacketBuilder {
    public void build(MessagePacker packer) throws IOException;
  }

  private static long lastId = System.currentTimeMillis();

  private static long getNextId() {
    return ++lastId;
  }

  private final String type;
  private final long id;
  private boolean sizeLocked;

  public Widget(String type) {
    this.id = getNextId();
    this.type = type;
    this.sizeLocked = false;
  }

  public long getId() {
    return id;
  }

  public Widget setSizeLocked(boolean sizeLocked) {
    this.sizeLocked = sizeLocked;
    return this;
  }

  public boolean isSizeLocked() {
    return sizeLocked;
  }

  public final void sendWidget(MessagePacker packer) throws IOException {
    packer.packString(type);
    packer.packLong(id);
    packer.packBoolean(sizeLocked);
    sendData(packer);
  }

  protected abstract void sendData(MessagePacker packer) throws IOException;

  public Widget getWidget(long id) {
    if (this.id == id) {
      return this;
    }
    return null;
  }

  public void handlePacket(MessageUnpacker msg) throws IOException {}

  public void sendPacket(PacketBuilder builder, DashboardServer dashboard) {
    try (ByteArrayOutputStream buf = new ByteArrayOutputStream();
        MessagePacker packer = MessagePack.newDefaultPacker(buf); ) {
      packer.packLong(id);
      builder.build(packer);
      packer.flush();
      dashboard.sendPacket("WidgetR2D", Base64.getEncoder().encodeToString(buf.toByteArray()));
    } catch (IOException e) {
      // Impossible
      e.printStackTrace();
    }
  }
}
