package frc4488.lib.dashboard;

import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import frc4488.lib.dashboard.actions.Action;
import frc4488.lib.dashboard.actions.CommandAction;
import frc4488.lib.dashboard.actions.PrintAction;
import frc4488.lib.dashboard.gui.DashboardWebsite;
import frc4488.lib.dashboard.packets.PacketProtocol;
import frc4488.lib.dashboard.packets.PacketProtocolFactory;
import frc4488.lib.logging.Console;
import frc4488.lib.logging.LogFile;
import frc4488.lib.logging.LogLevel;
import frc4488.lib.logging.LogManager;
import frc4488.lib.misc.Util;
import frc4488.robot.Robot;
import java.awt.Desktop;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.apache.commons.io.output.TeeOutputStream;

public class DashboardServer implements Console {

  private class SysStream extends OutputStream {
    private final String prefix;
    private final StringBuffer line;

    public SysStream(String prefix) {
      this.prefix = prefix;
      this.line = new StringBuffer();
    }

    @Override
    public void write(int b) throws IOException {
      if (b == '\n') {
        if (isDebugEnabled()) {
          println(prefix + line);
        }
        line.setLength(0);
      } else {
        line.append((char) b);
      }
    }
  }

  public static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("Dashboard");

  private static String[] parseArguments(String cmd) {
    List<String> output = new ArrayList<>();
    StringBuilder arg = new StringBuilder();
    boolean quoted = false;
    boolean escaped = false;
    for (char c : cmd.toCharArray()) {
      if (escaped) {
        arg.append(c);
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        quoted = !quoted;
      } else if (c == ' ' && !quoted) {
        if (!arg.isEmpty()) {
          output.add(arg.toString());
          arg.setLength(0);
        }
      } else {
        arg.append(c);
      }
    }
    if (!arg.isEmpty()) {
      output.add(arg.toString());
    }
    return output.toArray(String[]::new);
  }

  private final LogManager logger;
  private final LogFile dashboardLog;
  private final Map<String, Action> actionHandlers;
  private final DashboardWebsite site;
  private final OutputStream outputStream;
  private final OutputStream errorStream;
  private final BooleanSubscriber debug;
  private final Map<String, List<Consumer<String>>> packetListeners;
  private PacketProtocol packetProtocol;

  public DashboardServer(LogManager logger, PacketProtocol packetProtocol) throws IOException {
    this.logger = logger;
    this.dashboardLog = logger.getLogFile("Dashboard");
    this.actionHandlers = new HashMap<>();
    this.site = new DashboardWebsite(this);
    this.outputStream = new DashboardOutputStream(this::println);
    this.errorStream = new DashboardOutputStream(this::errorln);
    this.packetListeners = new HashMap<>();

    addPacketListener("WidgetD2R", site::handleWidgetPacket, true);
    addPacketListener("ConsoleAction", this::callAction, true);
    addPacketListener("Logs", this::handleLogsPacket, true);
    addPacketListener("SetPacketProtocol", this::setPacketProtocol, true);
    setPacketProtocol(packetProtocol);
    debug = TABLE.getBooleanTopic("DebugMode").subscribe(false);
  }

  /**
   * Most of the time <code>useMainThread</code> should be set to <code>true</code>! Only set this
   * to <code>false</code> when you know that the listener is safe to be called on a different
   * thread. The thread that this is called on depends on the packet listener, so it shouldn't be
   * assumed that it is ok to block that thread even though it isn't the main thread. <br>
   * <br>
   * Examples of things that should definitely be ran on the main thread:
   *
   * <ul>
   *   <li>Command scheduling & cancelling
   *   <li>Dashboard stuff (changing widgets, sending messages, etc.)
   *   <li>Managing motors (zeroing, changing speed, etc.)
   *   <li>A lot more
   * </ul>
   *
   * Examples of things that can be ran on a different thread:
   *
   * <ul>
   *   <li>Setting NT values
   *   <li>Reading/writing to files not used by anything else
   * </ul>
   */
  public DashboardServer addPacketListener(
      String name, Consumer<String> listener, boolean useMainThread) {
    if (useMainThread) {
      Consumer<String> unsafeListener = listener;
      listener =
          (msg) -> {
            Robot.runOnMainThread(() -> unsafeListener.accept(msg));
          };
    }
    packetListeners.computeIfAbsent(name, key -> new ArrayList<>()).add(listener);
    return this;
  }

  public DashboardServer setPacketProtocol(PacketProtocol packetProtocol) {
    Objects.requireNonNull(packetProtocol);
    if (this.packetProtocol == packetProtocol) {
      return this;
    }

    if (this.packetProtocol != null) {
      final PacketProtocol oldPacketProtocol = this.packetProtocol;
      // Prevents thread from interrupting itself or deadlocking if the packet protocol's main
      // thread calls setPacketProtocol
      new Thread(
              () -> {
                try {
                  oldPacketProtocol.close();
                } catch (Exception e) {
                  dashboardLog.println(LogLevel.ERROR, "Error while closing packet protocol", e);
                }
              },
              "Closing Packet Protocol")
          .start();
    }
    this.packetProtocol = packetProtocol;

    packetProtocol.setPacketListeners(packetListeners);
    TABLE.putValue("PacketProtocol", NetworkTableValue.makeString(packetProtocol.getName()));

    return this;
  }

  public DashboardServer setPacketProtocol(String name) {
    return setPacketProtocol(
        PacketProtocolFactory.createOrFallback(name, logger, () -> this.packetProtocol));
  }

  public DashboardServer bindToSystem() {
    Charset charset = Charset.defaultCharset();
    System.setOut(
        new PrintStream(
            new TeeOutputStream(System.out, new SysStream("SysOut > ")), true, charset));
    System.setErr(
        new PrintStream(
            new TeeOutputStream(System.err, new SysStream("SysErr > ")), true, charset));
    return this;
  }

  @Override
  public DashboardServer registerActionHandler(String name, Action handler) {
    actionHandlers.put(name, handler);
    return this;
  }

  @Override
  public DashboardServer registerActionHandler(String name, Runnable handler) {
    actionHandlers.put(
        name,
        new Action() {
          @Override
          public String getUsage() {
            return "";
          }

          @Override
          public void onCall(DashboardServer server, String[] args) {
            if (args.length == 0) {
              handler.run();
            } else {
              errorln(
                  "Unexpected input: "
                      + Stream.of(args)
                          .map(
                              arg ->
                                  "\""
                                      + arg.replaceAll("\\\\", "\\\\").replaceAll("\"", "\\\"")
                                      + "\"")
                          .reduce((a, b) -> a + " " + b)
                          .get());
            }
          }
        });
    return this;
  }

  public DashboardServer registerDefaultActions() {
    return registerActionHandler("command", new CommandAction())
        .registerActionHandler("print", new PrintAction());
  }

  public DashboardServer registerControlsAction(String controlFileName) {
    String controls =
        new String(site.getFile("controls/" + controlFileName + ".txt"), StandardCharsets.UTF_8);
    return registerActionHandler("controls", () -> println(controls));
  }

  public DashboardWebsite getWebsite() {
    return site;
  }

  public DashboardServer ready() {
    TABLE
        .getTopic("Ready")
        .genericPublish(NetworkTableType.kBoolean.getValueStr())
        .setBoolean(true);
    return this;
  }

  public void sendPacket(String name, String value) {
    packetProtocol.sendPacket(name, value);
  }

  public void callAction(String msg) {
    if (msg.isEmpty()) {
      return;
    }
    String[] cmd = parseArguments(msg);
    Action handler = actionHandlers.get(cmd[0]);
    if (handler == null) {
      StringBuilder helpMsg = new StringBuilder();
      if (!cmd[0].equals("help")) {
        helpMsg.append("Invalid action: ");
        helpMsg.append(cmd[0]);
        helpMsg.append('\n');
      }
      helpMsg.append("Options:\n");
      actionHandlers.forEach(
          (name, handler2) -> {
            helpMsg.append(" - ");
            helpMsg.append(name);
            helpMsg.append(' ');
            helpMsg.append(handler2.getUsage());
            helpMsg.append('\n');
          });
      if (cmd[0].equals("help")) {
        print(helpMsg.toString());
      } else {
        error(helpMsg.toString());
      }
    } else {
      String[] args = new String[cmd.length - 1];
      System.arraycopy(cmd, 1, args, 0, args.length);
      handler.onCall(this, args);
    }
  }

  @Override
  public void print(String msg) {
    packetProtocol.sendPacket("ConsoleMsg", msg);
  }

  @Override
  public void error(String msg) {
    print("### " + msg);
  }

  /**
   * Outputs an exception in this format: <br>
   * <br>
   * --------------------------------------------------------- <br>
   * Attempted to point <br>
   * <br>
   * Exception in thread "main" java.lang.NullPointerException <br>
   * at Pointer.point(Pointer.java:10) <br>
   * at Hand.use(Hand.java:10) <br>
   * ---------------------------------------------------------
   *
   * @param msg An accompanying message
   * @param e The exception
   */
  @Override
  public void errorln(String msg, Throwable e) {
    List<String> eLines = new ArrayList<>();
    int longestLine = msg.length();
    try (StringWriter str = new StringWriter();
        PrintWriter writer = new PrintWriter(str)) {
      e.printStackTrace(writer);
      String eMsg = str.toString();
      for (String line : eMsg.split("\n")) {
        eLines.add(line);
        if (line.length() > longestLine) {
          longestLine = line.length();
        }
      }
    } catch (IOException e2) {
      // Impossible
      e2.printStackTrace();
    }

    String border = "-".repeat(longestLine);
    errorln(border);
    errorln(msg);
    errorln("");
    for (String line : eLines) {
      errorln(line);
    }
    errorln(border);
  }

  @Override
  public void debug(String msg) {
    if (isDebugEnabled()) {
      print("* " + msg);
    }
  }

  @SuppressFBWarnings("EI_EXPOSE_REP")
  @Override
  public OutputStream getOutputStream() {
    return outputStream;
  }

  @SuppressFBWarnings("EI_EXPOSE_REP")
  @Override
  public OutputStream getErrorStream() {
    return errorStream;
  }

  @Override
  public boolean isDebugEnabled() {
    return debug.get(false);
  }

  @SuppressFBWarnings("EI_EXPOSE_REP")
  public LogManager getLogger() {
    return logger;
  }

  public LogFile getDashboardLog() {
    return dashboardLog;
  }

  private void handleLogsPacket(String content) {
    if (content.isEmpty()) {
      errorln("Invalid log packet: expected a code");
      return;
    }
    String path = content.substring(1);
    switch (content.charAt(0)) {
      case 'L' -> handleLoadLogsPacket(path);
      case 'V' -> handleViewLogsPacket(path);
      case 'D' -> handleDeleteLogsPacket(path);
      default -> errorln("Invalid log packet: invalid code '" + content.charAt(0) + "'");
    }
  }

  // L
  private void handleLoadLogsPacket(String path) {
    try {
      packetProtocol.sendPacket("LogsResponse", "L" + logger.generateFileList());
    } catch (IOException e) {
      dashboardLog.println(LogLevel.ERROR_CONSOLE, "Error while handling load logs packet", e);
    }
  }

  // V <file>
  private void handleViewLogsPacket(String path) {
    Path log = LogManager.allLogsPath.resolve(path);
    if (logger.isLogFile(log)) { // Security check to prevent ../../<sensitive file>
      logger.flush(true);
      try {
        packetProtocol.sendPacket("LogsResponse", "V" + path + "\0" + Files.readString(log));
      } catch (IOException e) {
        dashboardLog.println(LogLevel.ERROR_CONSOLE, "Error while handling view logs packet", e);
      }
    } else {
      errorln("Invalid view log packet: the requested file wasn't a log");
    }
  }

  // D [\0 | <file>] - response codes: 0 = success, 1 = log in use, 2 = error
  private void handleDeleteLogsPacket(String path) {
    if (path.isEmpty() || path.equals("\0")) { // Delete All or Delete Old
      try {
        Path dateDir = logger.getCurrentLogsPath().toAbsolutePath().getParent();
        for (Path otherDateDir : Files.list(LogManager.allLogsPath).toList()) {
          if (!Files.isSameFile(otherDateDir, dateDir)) {
            Util.deleteDir(otherDateDir);
          }
        }
        if (path.isEmpty()) { // Delete All instead of Delete Old
          for (Path otherTimeDir : Files.list(dateDir).toList()) {
            if (!Files.isSameFile(otherTimeDir, logger.getCurrentLogsPath())) {
              Util.deleteDir(otherTimeDir);
            }
          }
        }
      } catch (IOException e) {
        dashboardLog.println(LogLevel.ERROR_CONSOLE, "Error while handling delete logs packet", e);
      }
      handleLoadLogsPacket(""); // Send new log list
    } else {
      Path log = LogManager.allLogsPath.resolve(path);
      if (logger.isLogFile(log)) { // Security check to prevent ../../<sensitive file>
        if (logger.isLogFileInUse(log)) {
          packetProtocol.sendPacket("LogsResponse", "D1" + path);
        } else {
          packetProtocol.sendPacket(
              "LogsResponse",
              "D" + (Desktop.getDesktop().moveToTrash(log.toFile()) ? "0" : "2") + path);
        }
      } else {
        errorln("Invalid delete log packet: the requested file wasn't a log");
      }
    }
  }
}
