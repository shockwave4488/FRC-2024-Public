package frc4488.robot.subsystems.mock;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc4488.lib.devices.TalonFXMotor;
import frc4488.lib.logging.LeveledSmartDashboard;
import frc4488.lib.logging.LogManager;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.lib.sensors.LaserCAN;
import frc4488.lib.wpiextensions.ShockwaveSubsystemBase;
import frc4488.robot.subsystems.leds.RaspberryPiLEDController;

public class Intake extends ShockwaveSubsystemBase {

  private static final int HOLDING_HAY_DISTANCE = 80; // In millimeters
  private static final int SUPPLY_CURRENT_LIMIT = 40; // In Amps
  private static final double SUPPLY_CURRENT_LIMIT_TIME = 0.02; // In Seconds
  private static final int STATOR_CURRENT_LIMIT = 100; // In Amps

  private final TalonFXMotor intakeMotor;
  private final LaserCAN haySensor;
  private RollerState rollerState = RollerState.OFF;
  private int intakeMotorID;
  private int haySensorID;
  private final GenericPublisher ledPercent;

  public static enum RollerState {
    REVERSE(5),
    SLOW_REVERSE(3),
    OFF(0),
    FORWARD(-20);

    private double rps;

    private RollerState(double rps) {
      this.rps = rps;
    }
  }

  public Intake(PreferencesParser prefs) {
    updateFromPrefs(prefs);

    TalonFX intakeMotor = new TalonFX(intakeMotorID);
    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    currentConfigs.withStatorCurrentLimitEnable(true);
    currentConfigs.withSupplyCurrentLimitEnable(true);
    currentConfigs.withStatorCurrentLimit(STATOR_CURRENT_LIMIT);
    currentConfigs.withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT);
    currentConfigs.withSupplyTimeThreshold(SUPPLY_CURRENT_LIMIT_TIME);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = prefs.getDouble("IntakeP");
    configs.Slot0.kI = prefs.getDouble("IntakeI");
    configs.Slot0.kD = prefs.getDouble("IntakeD");
    configs.Slot0.kV = prefs.getDouble("IntakeV");
    configs.withCurrentLimits(currentConfigs);
    intakeMotor.getConfigurator().apply(configs);
    this.intakeMotor = new TalonFXMotor(intakeMotor, false, true, false);

    haySensor = new LaserCAN(haySensorID);

    ledPercent =
        RaspberryPiLEDController.TABLE
            .getTopic(RaspberryPiLEDController.TableKeys.INTAKE_PERCENT)
            .genericPublish(NetworkTableType.kDouble.getValueStr());
  }

  private void updateFromPrefs(PreferencesParser prefs) {
    // Defaults are just placeholders for now (same thing with the arbitrary numbers in prefs)
    intakeMotorID = prefs.tryGetValue(prefs::getInt, "IntakeMotorID", 10);
    haySensorID = prefs.tryGetValue(prefs::getInt, "IntakeHaySensorID", 5);
  }

  public void setCoastMode(boolean coastMode) {
    intakeMotor.setCoastModeButtonState(coastMode);
  }

  public void setRollerState(RollerState power) {
    rollerState = power;
  }

  private double getRollerPower() {
    return rollerState.rps;
  }

  public Command loadWithSensor() {
    return intakeCommand(RollerState.FORWARD).until(() -> hasHay());
  }

  public Command intakeCommand(RollerState entryPower) {
    return Commands.startEnd(() -> setRollerState(entryPower), () -> {}, this);
  }

  public int getDistance() {
    return haySensor.getDistance();
  }

  public boolean hasHay() {
    return haySensor.getDistance() <= HOLDING_HAY_DISTANCE;
  }

  @Override
  public void onStart(boolean sStopped) {
    intakeMotor.onStart();
    setRollerState(RollerState.OFF);
  }

  @Override
  public void periodic() {
    if (isSStopped()) {
      return;
    }

    if (rollerState == RollerState.OFF) {
      intakeMotor.applyNeutralMode(false);
    } else {
      intakeMotor.setRPS(rollerState.rps);
    }
  }

  @Override
  public void onStop(boolean sStopped) {
    intakeMotor.onStop();
    setRollerState(RollerState.OFF);
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void updateSmartDashboard() {
    LeveledSmartDashboard.HIGH.putBoolean("Has Entry Hay", hasHay());
    LeveledSmartDashboard.INFO.putNumber("Roller Power", getRollerPower());
    LeveledSmartDashboard.INFO.putNumber("Current Drawn (intake)", intakeMotor.getStatorCurrent());
    LeveledSmartDashboard.INFO.putNumber("Intake Speed", intakeMotor.getRPS());
    LeveledSmartDashboard.INFO.putNumber("Intake Distance", haySensor.getDistance());

    ledPercent.setDouble(getRollerPower() / RollerState.FORWARD.rps);
  }

  @Override
  public void setUpTrackables(LogManager logger) {
    logger
        .getLogFile("Intake")
        .setDefaultFrequency(10)
        .addTracker("IntakeRollerPower", () -> getRollerPower(), 4)
        .addTracker("HaySensor", this::hasHay);
  }

  @Override
  public void onSStop(boolean robotEnabled) {
    intakeMotor.onSStop(robotEnabled);
  }

  @Override
  public void onSRestart(boolean robotEnabled) {
    intakeMotor.onSRestart(robotEnabled);
  }
}
