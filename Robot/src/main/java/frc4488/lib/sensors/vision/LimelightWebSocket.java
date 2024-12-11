package frc4488.lib.sensors.vision;

import frc4488.lib.logging.LogFile;
import frc4488.lib.logging.LogLevel;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;

public class LimelightWebSocket extends WebSocketClient {

  // Found by looking at the packets sent when clicking the Restart Selected button in the Limelight
  // Hardware Manager using Wireshark
  private static final String RESTART_MSG = "restart_vision_server 0*#";

  private static URI createURI(String name) {
    try {
      return new URI("ws://" + name + ".local:5805");
    } catch (URISyntaxException e) {
      throw new RuntimeException("Failed to create Limelight URI", e);
    }
  }

  private final LogFile log;
  private final Queue<Runnable> queuedActions;

  public LimelightWebSocket(String name, LogFile log) {
    super(createURI(name));
    this.log = log;
    this.queuedActions = new ConcurrentLinkedQueue<>();
  }

  @Override
  public void onOpen(ServerHandshake data) {
    log.println(LogLevel.INFO, "WebSocket connected");
    while (!queuedActions.isEmpty()) {
      queuedActions.remove().run();
    }
  }

  @Override
  public void onMessage(String message) {}

  @Override
  public void onClose(int code, String reason, boolean remote) {
    log.println(LogLevel.INFO, "WebSocket disconnected");
  }

  @Override
  public void onError(Exception e) {
    log.println(LogLevel.ERROR, "Unknown error in Limelight WebSocket", e);
  }

  public void restart() {
    if (!isOpen()) {
      queuedActions.add(this::restart);
      return;
    }

    log.println(LogLevel.INFO, "Restarting limelight ...");
    send(RESTART_MSG);
  }
}
