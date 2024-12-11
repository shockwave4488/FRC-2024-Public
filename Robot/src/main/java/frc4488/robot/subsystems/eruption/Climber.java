package frc4488.robot.subsystems.eruption;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc4488.lib.logging.LeveledSmartDashboard;
import frc4488.lib.logging.LogManager;
import frc4488.lib.math.EpsilonUtil;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.lib.sensors.BeamBreak;
import frc4488.lib.wpiextensions.ShockwaveSubsystemBase;

public class Climber extends ShockwaveSubsystemBase {

  private final TalonSRX climber;
  private final PreferencesParser prefs;
  private final BeamBreak leftInductiveSensor;
  private final BeamBreak rightInductiveSensor;
  private int robotDestructionTicks = 0; // 20000 when climbing
  private static final int CLIMBER_TOP_LIMIT = 329000; // 333000 old
  private int desiredTicks = 0;
  private static final int REALLY_BIG_NUMBER = 999999;
  private NeutralMode climberNeutralMode;

  // private final Tuner tuner;
  private double p;
  private double i;
  private double d;
  private double f;

  private int countCycles = 0;
  private int minCountCycles;
  private int errorEpsilon;

  public Climber(
      int climberID,
      PreferencesParser prefs,
      int leftInductionSensorID,
      int rightInductionSensorID) {
    climber = new TalonSRX(climberID);
    leftInductiveSensor = new BeamBreak(leftInductionSensorID);
    rightInductiveSensor = new BeamBreak(rightInductionSensorID);
    climber.configFactoryDefault();
    climber.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    this.prefs = prefs;
    climberNeutralMode = NeutralMode.Brake;
    setClimberNeutralState(climberNeutralMode);

    updateFromPrefs();

    /*
    tuner = new Tuner(this::updateTuner, 2, prefs);

    tuner.addValueFromPrefs("ClimberP", 0);
    tuner.addValueFromPrefs("ClimberI", 0);
    tuner.addValueFromPrefs("ClimberD", 0);
    tuner.addValueFromPrefs("ClimberF", 0);

    tuner.start();
    */
  }

  /*
  public void updateTuner(Map<String, Double> vals) {
    p = vals.get("ClimberP");
    i = vals.get("ClimberI");
    d = vals.get("ClimberD");
    f = vals.get("ClimberF");

    updatePIDF(p, i, d, f);
  }
  */

  private synchronized void updatePIDF(double p, double i, double d, double f) {
    climber.config_kF(0, f);
    climber.config_kP(0, p);
    climber.config_kI(0, i);
    climber.config_kD(0, d);
  }

  public void updateFromPrefs() {
    p = prefs.tryGetValue(prefs::getDouble, "ClimberP", 0.0);
    i = prefs.tryGetValue(prefs::getDouble, "ClimberI", 0.0);
    d = prefs.tryGetValue(prefs::getDouble, "ClimberD", 0.0);
    f = prefs.tryGetValue(prefs::getDouble, "ClimberF", 0.0);

    updatePIDF(p, i, d, f);
  }

  public void setClimberNeutralState(NeutralMode state) {
    climberNeutralMode = state;
    climber.setNeutralMode(state);
  }

  /**
   * Use this to set the climber position without caring about this subsystem's isStable boolean
   * method. isStable will always return true.
   *
   * @param ticks The desired position for the climber in ticks
   */
  public void setClimberPosition(int ticks) {
    setClimberPosition(ticks, 0, REALLY_BIG_NUMBER);
  }

  public void setClimberPosition(int ticks, int minCountCycles, int epsilon) {
    /* Stop the robot from moving arm any further down if it's about to destroy the robot while
    still allowing it to move the arm back up */
    ticks = Math.max(ticks, robotDestructionTicks);
    ticks = Math.min(ticks, CLIMBER_TOP_LIMIT);

    desiredTicks = ticks;
    countCycles = 0;
    this.minCountCycles = minCountCycles;
    errorEpsilon = epsilon;
  }

  public int getClimberPosition() {
    return (int) climber.getSelectedSensorPosition();
  }

  public boolean isStable() {
    return (countCycles >= minCountCycles);
  }

  public void doneCycle() {
    if (EpsilonUtil.epsilonEquals(getClimberPosition(), desiredTicks, errorEpsilon)) {
      countCycles++;
    } else {
      countCycles = 0;
    }
  }

  @Override
  public void periodic() {
    if (isSStopped()) {
      return;
    }
    climber.set(ControlMode.Position, desiredTicks);
    doneCycle();
  }

  public int getUpperLimit() {
    return CLIMBER_TOP_LIMIT;
  }

  public int getLowerLimit() {
    return robotDestructionTicks;
  }

  public void setLowerLimit(int limit) {
    robotDestructionTicks = limit;
  }

  public boolean getLeftInductiveSensor() {
    return !leftInductiveSensor.get();
  }

  public boolean getRightInductiveSensor() {
    return !rightInductiveSensor.get();
  }

  @Override
  public void onStart(boolean sStopped) {
    if (sStopped) {
      return;
    }
    setClimberNeutralState(NeutralMode.Brake);
  }

  @Override
  public void onStop(boolean sStopped) {
    climber.set(ControlMode.Disabled, 0);
  }

  @Override
  public void zeroSensors() {
    climber.setSelectedSensorPosition(0);
  }

  @Override
  public void updateSmartDashboard() {
    LeveledSmartDashboard.INFO.putNumber("Climber Position Ticks", getClimberPosition());
    // SmartDashboard.putString("Climber Neutral Mode", climberNeutralMode.toString());
    LeveledSmartDashboard.INFO.putNumber("Climber Desired Ticks", desiredTicks);
    LeveledSmartDashboard.INFO.putBoolean(
        "Climber Left Inductive Sensor", leftInductiveSensor.get());
    LeveledSmartDashboard.INFO.putBoolean(
        "Climber Right Inductive Sensor", rightInductiveSensor.get());
  }

  @Override
  public void setUpTrackables(LogManager logger) {
    logger
        .getLogFile("Climber/ActualTicks")
        .addTracker("ClimberActualTicks", () -> this.getClimberPosition(), 10);

    logger
        .getLogFile("Climber/DesiredTicks")
        .addTracker("ClimberDesiredTicks", () -> desiredTicks, 5);
  }

  @Override
  public void onSStop(boolean robotEnabled) {
    climber.set(ControlMode.Disabled, 0);
  }

  @Override
  public void onSRestart(boolean robotEnabled) {
    if (robotEnabled) {
      setClimberNeutralState(NeutralMode.Brake);
    }
  }
}
