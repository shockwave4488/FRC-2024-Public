package frc4488.lib.dashboard.gui;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.msgpack.core.MessagePacker;

/** Internally uses a SendableChooser as a backup */
public abstract class AbstractSelectionWidget<T> extends Widget {

  private final String name;
  private final String path;
  private final SendableChooser<T> chooser;
  private final List<Consumer<T>> listeners;
  private boolean following;
  private GenericPublisher pub;

  protected AbstractSelectionWidget(String type, String name) {
    super(type);
    this.name = name;
    this.path = "/SmartDashboard/" + name;
    this.chooser = new SendableChooser<>();
    this.listeners = new ArrayList<>();
    SmartDashboard.putData(name, chooser);
  }

  protected AbstractSelectionWidget(
      String type, String name, String path, SendableChooser<T> chooser) {
    super(type);
    this.name = name;
    this.path = path;
    this.chooser = chooser;
    this.listeners = new ArrayList<>();
  }

  protected void addOption(String name, T value, boolean isDefault) {
    if (isDefault) {
      chooser.setDefaultOption(name, value);
    } else {
      chooser.addOption(name, value);
    }
  }

  /**
   * @return The key of the selection, or null if the {@link SendableChooser} was passed in to the
   *     constructor and hasn't been put to NT yet
   */
  public String getSelectedKey() {
    return NetworkTableInstance.getDefault().getEntry(path + "/active").getString(null);
  }

  public AbstractSelectionWidget<T> follow(Supplier<String> toFollow) {
    if (following) {
      throw new IllegalStateException("Can only follow one thing at a time!");
    }
    following = true;

    AtomicReference<String> lastValue = new AtomicReference<>(toFollow.get());
    setSelected(lastValue.getPlain());
    CommandScheduler.getInstance()
        .getDefaultButtonLoop()
        .bind(
            () -> {
              String newValue = toFollow.get();
              if (!lastValue.getPlain().equals(newValue)) {
                lastValue.setPlain(newValue);
                setSelected(newValue);
              }
            });

    return this;
  }

  public void setSelected(String name) {
    if (pub == null) {
      pub =
          NetworkTableInstance.getDefault()
              .getTopic(path + "/selected")
              .genericPublish(NetworkTableType.kString.getValueStr());
    }
    pub.setString(name);
  }

  protected T getSelected() {
    return chooser.getSelected();
  }

  public AbstractSelectionWidget<T> addListener(Consumer<T> onChange) {
    if (listeners.isEmpty()) {
      chooser.onChange(
          value -> {
            for (Consumer<T> listener : listeners) {
              listener.accept(value);
            }
          });
    }
    listeners.add(onChange);
    return this;
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {
    packer.packString(name);
    packer.packString(path);
  }
}
