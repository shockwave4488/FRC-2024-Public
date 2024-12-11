package shockwave.leds;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import shockwave.leds.canvas.LEDCanvas;
import shockwave.leds.canvas.LEDConcat;
import shockwave.leds.canvas.LEDFill;
import shockwave.leds.canvas.LEDManager;
import shockwave.leds.canvas.LEDReverser;
import shockwave.leds.canvas.LEDStrip;
import shockwave.leds.canvas.LEDStrip.LEDEffect;
import shockwave.leds.controllers.LEDControllerThread;
import shockwave.leds.modes.BlankMode;
import shockwave.leds.modes.ClimbingMode;
import shockwave.leds.modes.EnabledMode;
import shockwave.leds.modes.FireBreathingDragonMode;
import shockwave.leds.modes.Mode;
import shockwave.leds.modes.MusicMode;
import shockwave.leds.modes.StartupMode;

public class LEDs {

  private static final int NUM_LEDS = 143;

  public static void main(String[] args) throws IOException {
    boolean simulation = args.length > 0 && args[0].equals("simulation");
    // Change IP to your computer to test in simulation
    NetworkTableInstance.getDefault().setServer("10.44.88.2", 5810);
    NetworkTableInstance.getDefault().startClient4("LEDs");
    new LEDs(simulation);
  }

  private static LEDCanvas unwindEq(LEDCanvas canvas, int height) {
    if (canvas.getNumLeds() % height != 0) {
      throw new IllegalArgumentException("This canvas isn't a multiple of the height!");
    }

    LEDManager manager = new LEDManager(canvas);
    List<LEDCanvas> columns = new ArrayList<>();
    for (int i = 0; i < canvas.getNumLeds(); i += height) {
      LEDCanvas column = manager.newSection().takeRangeLen(i, height).build();
      if ((i / height) % 2 == 1) {
        column = new LEDReverser(column);
      }
      columns.add(column);
    }
    return new LEDConcat(columns.toArray(LEDCanvas[]::new));
  }

  private static LEDCanvas compressEq(LEDCanvas canvas, int height) {
    if (canvas.getNumLeds() % height != 0) {
      throw new IllegalArgumentException("This canvas isn't a multiple of the height!");
    }

    LEDManager manager = new LEDManager(canvas);
    List<LEDCanvas> columns = new ArrayList<>();
    for (int i = 0; i < canvas.getNumLeds(); i += height) {
      LEDCanvas column = manager.newSection().takeRangeLen(i, height).build();
      columns.add(new LEDFill(column));
    }
    return new LEDConcat(columns.toArray(LEDCanvas[]::new));
  }

  private final NetworkTable ledTable;
  private final NetworkTableEntry doneEntry;
  private final NetworkTableEntry musicShift;
  private final NetworkTableEntry musicSpeed;
  private final GenericSubscriber armPercentSub;
  private final GenericSubscriber climberPercentSub;
  private final GenericSubscriber intakePercentSub;
  private final GenericSubscriber shooterPercentSub;
  private final List<LEDStrip> strips;
  private final LEDCanvas frameLeftStrip;
  private final LEDCanvas frameRightStrip;
  private final LEDCanvas dragonStrip;
  private final LEDCanvas armLeftStrip;
  private final LEDCanvas armRightStrip;
  private final EQ leftEq;
  private final EQ rightEq;
  private final LEDCanvas leftEqCompressed;
  private final LEDCanvas rightEqCompressed;
  private final LEDControllerThread thread;
  private final Map<Integer, Supplier<Mode>> modes;
  private int currentModeId;
  private Mode currentMode;

  public LEDs(boolean simulation) throws IOException {
    ledTable = NetworkTableInstance.getDefault().getTable("LEDs");
    doneEntry = ledTable.getEntry(TableKeys.DONE);
    musicShift = ledTable.getEntry(TableKeys.MUSIC_SHIFT);
    musicShift.setInteger(MusicMode.SHIFT);
    musicSpeed = ledTable.getEntry(TableKeys.MUSIC_SPEED);
    musicSpeed.setDouble(MusicMode.SPEED);
    armPercentSub =
        ledTable.getTopic(TableKeys.ARM_PERCENT).genericSubscribe(PubSubOption.periodic(0.02));
    climberPercentSub = ledTable.getTopic(TableKeys.CLIMBER_PERCENT).genericSubscribe();
    intakePercentSub = ledTable.getTopic(TableKeys.INTAKE_PERCENT).genericSubscribe();
    shooterPercentSub =
        ledTable.getTopic(TableKeys.SHOOTER_PERCENT).genericSubscribe(PubSubOption.periodic(0.02));
    strips = new ArrayList<>();

    if (simulation) {
      frameLeftStrip = LEDStripSimulation.showNewSimulation("Frame Left", 6, -6);
      frameRightStrip = LEDStripSimulation.showNewSimulation("Frame Right", 6, -6);
      dragonStrip = LEDStripSimulation.showNewSimulation("Dragon", 26, 1);
      armLeftStrip = LEDStripSimulation.showNewSimulation("Arm Left", 1, -23);
      armRightStrip = LEDStripSimulation.showNewSimulation("Arm Right", 1, -23);

      strips.add((LEDStrip) frameLeftStrip);
      strips.add((LEDStrip) frameRightStrip);
      strips.add((LEDStrip) dragonStrip);
      strips.add((LEDStrip) armLeftStrip);
      strips.add((LEDStrip) armRightStrip);
    } else {
      LEDStrip strip =
          LEDStrip.forSerialPort(
              "ttyUSB0",
              NUM_LEDS,
              LEDEffect.SQUARE
                  .andThen(LEDEffect.colorCorrect(1, 0.45, 0.25))
                  .andThen(LEDEffect.GRB));
      strips.add(strip);

      LEDManager manager = new LEDManager(strip);
      frameLeftStrip =
          unwindEq(
              new LEDConcat(
                  LEDStrip.nullStrip(1), manager.newSection().takeRangeLen(0, 35).build()),
              6);
      frameRightStrip = unwindEq(manager.newSection().takeRangeLen(35, 36).build(), 6);
      dragonStrip = manager.newSection().takeRangeLen(71, 26).build();
      armLeftStrip = manager.newSection().takeRangeLen(97, 23).build();
      armRightStrip = manager.newSection().takeRangeLen(120, 23).build();
    }

    leftEq = new EQ(6, 6, frameLeftStrip);
    rightEq = new EQ(6, 6, frameRightStrip);

    leftEqCompressed = compressEq(frameLeftStrip, 6);
    rightEqCompressed = compressEq(frameRightStrip, 6);

    thread =
        new LEDControllerThread(
            () -> {
              for (LEDStrip strip : strips) {
                strip.show();
              }
            });
    thread.start();

    modes = new HashMap<>();
    modes.put(0, BlankMode::new);
    modes.put(1, StartupMode::new);
    modes.put(2, FireBreathingDragonMode::new);
    modes.put(3, EnabledMode::new);
    modes.put(4, ClimbingMode::new);
    modes.put(5, MusicMode::new);

    currentModeId = 0;
    currentMode = new BlankMode();

    ledTable.getTopic(TableKeys.MODE).genericSubscribe(PubSubOption.sendAll(true));
    ledTable.addListener(
        TableKeys.MODE,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        (table, key, event) -> {
          long data = event.valueData.value.getInteger();
          int modeId = (int) (data >> 32);
          int modeArg = (int) (data & 0xFFFFFFFF);

          System.out.println("New Mode: " + modeId + " [" + modeArg + "]");

          if (currentModeId == modeId) {
            if (currentMode.onNewArg(this, modeArg)) {
              return;
            }
          }

          Supplier<Mode> mode = modes.get(modeId);
          if (mode == null) {
            done();
          } else {
            thread.clearControllers();
            currentModeId = modeId;
            currentMode = mode.get();
            currentMode.set(this, modeArg);
          }
        });
  }

  public void done() {
    doneEntry.setInteger(doneEntry.getInteger(0) + 1);
  }

  public long getMusicShift() {
    return musicShift.getInteger(MusicMode.SHIFT);
  }

  public double getMusicSpeed() {
    return musicSpeed.getDouble(MusicMode.SPEED);
  }

  public double getArmPercent() {
    return armPercentSub.getDouble(0);
  }

  public double getClimberPercent() {
    return climberPercentSub.getDouble(0);
  }

  public double getIntakePercent() {
    return intakePercentSub.getDouble(0);
  }

  public double getShooterPercent() {
    return shooterPercentSub.getDouble(0);
  }

  public List<LEDStrip> getStrips() {
    return strips;
  }

  public LEDCanvas getFrameLeftStrip() {
    return frameLeftStrip;
  }

  public LEDCanvas getFrameRightStrip() {
    return frameRightStrip;
  }

  public LEDCanvas getDragonStrip() {
    return dragonStrip;
  }

  public LEDCanvas getArmLeftStrip() {
    return armLeftStrip;
  }

  public LEDCanvas getArmRightStrip() {
    return armRightStrip;
  }

  public EQ getLeftEq() {
    return leftEq;
  }

  public EQ getRightEq() {
    return rightEq;
  }

  public LEDCanvas getLeftEqCompressed() {
    return leftEqCompressed;
  }

  public LEDCanvas getRightEqCompressed() {
    return rightEqCompressed;
  }

  public LEDControllerThread getThread() {
    return thread;
  }
}
