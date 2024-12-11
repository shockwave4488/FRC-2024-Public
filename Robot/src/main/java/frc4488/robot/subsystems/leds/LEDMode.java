package frc4488.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc4488.robot.commands.LEDs.SetLEDMode;
import java.util.function.Function;

public record LEDMode(int id, int arg) {
  public static class Vortex {
    public static enum Priorities implements Function<LEDController, Command> {
      DISABLED(idle()),
      STARTUP(startup()),
      ENABLED(enabled(false)),
      ARM_ACTIVE(enabled(true)),
      CLIMBING(climbing()),
      MUSIC(false, null),
      // Toggled via a switch on the dashboard, so this isolates it from the CLIMBING logic
      RAINBOW(climbing());

      private final boolean isModeFinal;
      private LEDMode mode;

      private Priorities(boolean isModeFinal, LEDMode mode) {
        this.isModeFinal = isModeFinal;
        this.mode = mode;
      }

      private Priorities(LEDMode mode) {
        this(true, mode);
      }

      public void setMode(LEDMode mode) {
        if (isModeFinal) {
          throw new UnsupportedOperationException(
              "The LED mode priority " + this + " doesn't support changing modes");
        }
        this.mode = mode;
      }

      @Override
      public Command apply(LEDController controller) {
        return new SetLEDMode(controller, mode);
      }
    }

    public static LEDMode startup() {
      return new LEDMode(1);
    }

    public static LEDMode idle() {
      return new LEDMode(2);
    }

    /**
     * @param angry Changes the dragon to be red
     */
    public static LEDMode enabled(boolean angry) {
      return new LEDMode(3, angry ? 1 : 0);
    }

    public static LEDMode climbing() {
      return new LEDMode(4);
    }

    /**
     * @param songId Must be in the range 0-255
     * @param skipMillis Some of the data will be discarded, so songs more than 4.6 hours long
     *     cannot be accurately resumed
     */
    public static LEDMode music(int songId, int skipMillis) {
      if (songId < 0 || songId > 255) {
        throw new IllegalArgumentException("The song id must be between 0 and 255");
      }
      if (skipMillis > 0xFF_FFFF) {
        throw new IllegalArgumentException("The skip millis must be <= 0xFFFFFF");
      }
      // Top byte is the song id, the rest are the time
      return new LEDMode(5, (songId << 24) | (skipMillis & 0xFF_FFFF));
    }
  }

  public static class Supercell {
    /**
     * Set the entire strip to one color
     *
     * @param color The color in RGB format (0xFFAA00 is orange for example)
     */
    public static LEDMode solid(int color) {
      return new LEDMode(1, color);
    }

    public static LEDMode seismic() {
      return new LEDMode(2);
    }

    /**
     * @param speed The amount the animation progresses every iteration, where 256 is a complete
     *     loop (no movement)
     */
    public static LEDMode stripe(byte speed) {
      return new LEDMode(3, speed);
    }

    /**
     * @param speed The delay between every iteration
     */
    public static LEDMode rainbow(byte speed) {
      return new LEDMode(4, speed);
    }
  }

  public static class Eruption {
    public static LEDMode seismic() {
      return new LEDMode(2);
    }
  }

  public static LEDMode blank() {
    return new LEDMode(0);
  }

  public LEDMode(int id) {
    this(id, 0);
  }
}
