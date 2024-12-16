package frc4488.lib.dashboard.packets;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.Topic;
import frc4488.lib.dashboard.DashboardServer;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.function.Consumer;

class NetworkTablePackets {

  private static final Random rand =
      new Random(); // Used to make sure every packet value is different to trigger an update
  private static final Map<String, List<Consumer<String>>> listeners = new HashMap<>();
  private static final Map<String, GenericPublisher> publishers = new HashMap<>();

  static {
    MultiSubscriber sub =
        new MultiSubscriber(
            NetworkTableInstance.getDefault(),
            new String[] {"/Dashboard/Packets/"},
            PubSubOption.sendAll(true));
    Map<Integer, String> topics = new HashMap<>();
    NetworkTableInstance.getDefault()
        .addListener(
            sub,
            EnumSet.of(NetworkTableEvent.Kind.kPublish, NetworkTableEvent.Kind.kValueRemote),
            event -> {
              if (event.is(NetworkTableEvent.Kind.kPublish)) {
                topics.put(event.topicInfo.topic, event.topicInfo.name);
                return;
              }
              String name = topics.get(event.valueData.topic);
              if (name
                  == null) { // Happens sometimes when robot is restarted without dashboard reload
                for (Topic topic : NetworkTableInstance.getDefault().getTopics()) {
                  if (topic.getHandle() == event.valueData.topic) {
                    name = topic.getName();
                    topics.put(event.valueData.topic, name);
                    break;
                  }
                }
                if (name == null) {
                  System.err.println("Failed to find topic with id: " + event.valueData.topic);
                  return;
                }
              }
              name = name.substring(name.indexOf(":") + 1);
              List<Consumer<String>> matchingListeners = listeners.get(name);
              if (matchingListeners != null) {
                String value = event.valueData.value.getString();
                value = value.substring(value.indexOf(":") + 1);
                for (Consumer<String> listener : matchingListeners) {
                  listener.accept(value);
                }
              }
            });
  }

  public static void init() {
    // Load class
  }

  public static void setListeners(Map<String, List<Consumer<String>>> listeners) {
    NetworkTablePackets.listeners.clear();
    NetworkTablePackets.listeners.putAll(listeners);
    NetworkTablePackets.listeners.replaceAll(
        (name, specificListeners) -> new ArrayList<>(specificListeners));
  }

  public static void addListener(String name, Consumer<String> listener) {
    listeners.computeIfAbsent(name, key -> new ArrayList<>()).add(listener);
  }

  public static void sendPacket(String name, String value) {
    publishers
        .computeIfAbsent(
            name,
            key ->
                DashboardServer.TABLE
                    .getTopic("Packets/" + genRandHex() + ":" + name)
                    .genericPublish(NetworkTableType.kString.getValueStr()))
        .setString(genRandHex() + ":" + value);
  }

  private static String genRandHex() {
    return Integer.toHexString(rand.nextInt());
  }
}
