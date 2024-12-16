package frc4488.lib.dashboard.gui;

import java.util.function.BooleanSupplier;

public class SwitchWidget extends AbstractSelectionWidget<Boolean> {

  private final boolean defaultValue;

  public SwitchWidget(String name, boolean enabled) {
    super("switch", name);
    this.defaultValue = enabled;

    addOption("false", false, !enabled);
    addOption("true", true, enabled);
  }

  public SwitchWidget followEnabled(BooleanSupplier toFollow) {
    follow(() -> toFollow.getAsBoolean() ? "true" : "false");
    return this;
  }

  public void setEnabled(boolean enabled) {
    setSelected(enabled ? "true" : "false");
  }

  public boolean isEnabled() {
    Boolean value = getSelected();
    if (value == null) {
      return defaultValue;
    }
    return value;
  }
}
