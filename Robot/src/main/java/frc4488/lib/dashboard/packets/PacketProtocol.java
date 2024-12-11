package frc4488.lib.dashboard.packets;

import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

public interface PacketProtocol extends AutoCloseable {
  public String getName();

  public void setPacketListeners(Map<String, List<Consumer<String>>> listeners);

  public void addPacketListener(String name, Consumer<String> listener);

  public void sendPacket(String name, String value);
}
