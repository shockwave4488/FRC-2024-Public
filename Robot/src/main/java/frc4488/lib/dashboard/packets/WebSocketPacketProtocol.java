package frc4488.lib.dashboard.packets;

import frc4488.lib.logging.LogLevel;
import frc4488.lib.logging.LogManager;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import org.msgpack.core.MessagePack;
import org.msgpack.core.MessagePacker;
import org.msgpack.core.MessageUnpacker;

public class WebSocketPacketProtocol extends WebSocketServer implements PacketProtocol {

  private static int findPort(int... ports) throws IOException {
    for (int port : ports) {
      try (ServerSocket server = new ServerSocket(port)) {
        server.setReuseAddress(true);
        return port;
      } catch (IOException e) {
        // Expected, try next port
      }
    }
    throw new IOException("Unable to find an open port!");
  }

  private final LogManager logger;
  private final Map<String, List<Consumer<String>>> listeners;

  public WebSocketPacketProtocol(LogManager logger, int... ports) throws IOException {
    super(new InetSocketAddress(findPort(ports)));
    this.logger = logger;
    this.listeners = new HashMap<>();
    this.start();
  }

  @Override
  public String getName() {
    return PacketProtocolFactory.WEBSOCKET_PROTOCOL;
  }

  @Override
  public void setPacketListeners(Map<String, List<Consumer<String>>> listeners) {
    this.listeners.clear();
    this.listeners.putAll(listeners);
    this.listeners.replaceAll((name, specificListeners) -> new ArrayList<>(specificListeners));
  }

  @Override
  public void addPacketListener(String name, Consumer<String> listener) {
    listeners.computeIfAbsent(name, key -> new ArrayList<>()).add(listener);
  }

  @Override
  public void sendPacket(String name, String value) {
    try (ByteArrayOutputStream buf = new ByteArrayOutputStream();
        MessagePacker packer = MessagePack.newDefaultPacker(buf)) {
      packer.packString(name);
      packer.packString(value);
      packer.flush();
      byte[] packet = buf.toByteArray();
      for (WebSocket conn : getConnections()) {
        conn.send(packet);
      }
    } catch (Exception e) {
      onError(null, e);
    }
  }

  private void receivePacket(String name, String value) {
    try {
      List<Consumer<String>> specificListeners = listeners.get(name);
      if (specificListeners == null) {
        return;
      }
      for (Consumer<String> listener : specificListeners) {
        listener.accept(value);
      }
    } catch (Exception e) {
      onError(null, e);
    }
  }

  @Override
  public void onStart() {
    logger
        .getLogFile("Dashboard")
        .println(LogLevel.INFO, "Dashboard web socket started on port " + getPort());
  }

  @Override
  public void onOpen(WebSocket conn, ClientHandshake handshake) {}

  @Override
  public void onClose(WebSocket conn, int code, String reason, boolean remote) {}

  @Override
  public void onMessage(WebSocket conn, String message) {
    onError(conn, new UnsupportedOperationException("Only binary messages supported!"));
  }

  @Override
  public void onMessage(WebSocket conn, ByteBuffer message) {
    try (MessageUnpacker in = MessagePack.newDefaultUnpacker(message)) {
      receivePacket(in.unpackString(), in.unpackString());
    } catch (Exception e) {
      onError(conn, e);
    }
  }

  @Override
  public void onError(WebSocket conn, Exception e) {
    logger.getLogFile("Dashboard").println(LogLevel.ERROR, e);
  }

  @Override
  public void close() throws Exception {
    this.stop();
  }
}
