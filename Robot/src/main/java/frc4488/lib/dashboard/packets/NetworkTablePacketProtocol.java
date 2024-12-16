package frc4488.lib.dashboard.packets;

import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

public class NetworkTablePacketProtocol implements PacketProtocol {

  public NetworkTablePacketProtocol() {
    NetworkTablePackets.init();
  }

  @Override
  public String getName() {
    return PacketProtocolFactory.NETWORK_TABLE_PROTOCOL;
  }

  @Override
  public void setPacketListeners(Map<String, List<Consumer<String>>> listeners) {
    NetworkTablePackets.setListeners(listeners);
  }

  @Override
  public void addPacketListener(String name, Consumer<String> listener) {
    NetworkTablePackets.addListener(name, listener);
  }

  @Override
  public void sendPacket(String name, String value) {
    NetworkTablePackets.sendPacket(name, value);
  }

  @Override
  public void close() throws Exception {}
}
