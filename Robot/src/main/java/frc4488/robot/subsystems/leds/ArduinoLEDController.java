package frc4488.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

public class ArduinoLEDController extends LEDController {
  // Refer to top of LEDs.ino for protocol spec
  private class LEDDriver extends Thread {
    public LEDDriver() {
      setName("LEDDriver");
    }

    @Override
    public void run() {
      int index = queuedModes.size() - 1;
      for (int i = 0; i < index; i++) {
        queuedModes.remove();
      }
      LEDMode mode = queuedModes.remove();

      boolean[] data = new boolean[64];
      splitInt(mode.id(), data, 0);
      splitInt(mode.arg(), data, 32);

      long startTime = System.currentTimeMillis();
      boolean toggle = true;
      for (int i = 0; i < data.length; i++) {
        dataOutput.set(data[i]);
        indicatorOutput.set(toggle);
        while (indicatorInput.get() != toggle) {
          if (System.currentTimeMillis() - startTime > 1000) {
            indicatorOutput.set(false);
            return;
          }
        }
        toggle = !toggle;
      }
      indicatorOutput.set(false);
    }

    /**
     * Gets the individual bits from the <code>num</code> and puts them into <code>target</code>,
     * starting at index <code>offset</code> in the target array
     */
    private void splitInt(int num, boolean[] target, int offset) {
      for (int i = 31; i >= 0; i--) {
        target[i + offset] = ((num & 0b1) != 0);
        num >>>= 1;
      }
    }
  }

  private final DigitalOutput indicatorOutput;
  private final DigitalOutput dataOutput;
  private final DigitalInput indicatorInput;
  private final Queue<LEDMode> queuedModes;
  private LEDDriver driver;

  public ArduinoLEDController(LEDMode defaultMode, int[] ledDioIds) {
    super(defaultMode);
    this.indicatorOutput = new DigitalOutput(ledDioIds[0]);
    this.dataOutput = new DigitalOutput(ledDioIds[1]);
    this.indicatorInput = new DigitalInput(ledDioIds[2]);
    this.queuedModes = new ConcurrentLinkedQueue<>();

    onModeChange(defaultMode);
  }

  @Override
  protected void onModeChange(LEDMode mode) {
    queuedModes.add(mode);
    if (driver == null || !driver.isAlive()) {
      driver = new LEDDriver();
      driver.start();
    }
  }
}
