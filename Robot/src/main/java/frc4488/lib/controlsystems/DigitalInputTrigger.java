package frc4488.lib.controlsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc4488.lib.preferences.PreferencesParser;

public class DigitalInputTrigger {
  private final Trigger trigger;
  private final DigitalInput digitalInput;

  public DigitalInputTrigger(int id) {
    if (id != -1) {
      this.digitalInput = new DigitalInput(id);
      this.trigger = new Trigger(() -> this.digitalInput.get());
    } else {
      this.digitalInput = null;
      this.trigger = new Trigger(() -> false);
    }
  }

  public DigitalInputTrigger(String key, PreferencesParser prefs) {
    this(prefs.getInt(key));
  }

  public Trigger get() {
    return this.trigger;
  }
}
