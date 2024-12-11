package frc4488.robot.subsystems.vortex;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc4488.lib.commands.CommandUtil;
import frc4488.lib.commands.DoneCycleCommand;
import frc4488.lib.controlsystems.DoneCycleMachine;
import frc4488.lib.controlsystems.SimManager;
import frc4488.lib.devices.TalonFXMotor;
import frc4488.lib.logging.LeveledSmartDashboard;
import frc4488.lib.logging.LogManager;
import frc4488.lib.misc.Util;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.lib.wpiextensions.ShockwaveSubsystemBase;
import frc4488.robot.constants.Constants2024.RobotConstants.ShooterConstants;
import frc4488.robot.subsystems.leds.RaspberryPiLEDController;
import java.util.Optional;

public class Shooter extends ShockwaveSubsystemBase {
  private static final int STATOR_CURRENT_LIMIT = 88; // In Amps
  private static final int SUPPLY_CURRENT_LIMIT = 44; // In Amps
  private static final int SUBWOOFER_SPEED = 100;
  private static final int SUBWOOFER_PREP_SPEED = 40;
  private static final int AMP_SPEED_LOWER = 5;
  private static final int AMP_SPEED_UPPER = 30;
  private static final int REVERSE_SPEED = -10;

  private final TalonFXMotor lowerMotor;
  private final TalonFXMotor upperMotor;
  private double lowerRequestedRPS;
  private double upperRequestedRPS;
  public final double MAX_RPS;
  private final Optional<FlywheelSim> flywheelSim;
  private final GenericPublisher ledPercent;

  public Shooter(PreferencesParser prefs) {
    TalonFX lowerMotor = new TalonFX(prefs.getInt("LowerFlywheelID"));
    TalonFX upperMotor = new TalonFX(prefs.getInt("UpperFlywheelID"));
    MAX_RPS = prefs.getDouble("ShooterMaxRPS");
    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    currentConfigs.withStatorCurrentLimitEnable(true);
    currentConfigs.withSupplyCurrentLimitEnable(true);
    currentConfigs.withStatorCurrentLimit(STATOR_CURRENT_LIMIT);
    currentConfigs.withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT);
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = prefs.getDouble("ShooterP");
    configs.Slot0.kI = prefs.getDouble("ShooterI");
    configs.Slot0.kD = prefs.getDouble("ShooterD");
    configs.Slot0.kV = prefs.getDouble("ShooterV");
    if (prefs.getBoolean("ShooterInverted")) {
      configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
    configs.withCurrentLimits(currentConfigs);
    Util.retry(() -> lowerMotor.getConfigurator().apply(configs), StatusCode::isOK, 5);
    Util.retry(() -> upperMotor.getConfigurator().apply(configs), StatusCode::isOK, 5);
    this.lowerMotor = new TalonFXMotor(lowerMotor, false, false, false);
    this.upperMotor = new TalonFXMotor(upperMotor, false, false, false);

    if (RobotBase.isSimulation()) {
      DCMotor simMotor = DCMotor.getFalcon500(1);
      FlywheelSim flywheelSim = new FlywheelSim(simMotor, 1, 1);
      this.flywheelSim = Optional.of(flywheelSim);
    } else {
      this.flywheelSim = Optional.empty();
    }

    ledPercent =
        RaspberryPiLEDController.TABLE
            .getTopic(RaspberryPiLEDController.TableKeys.SHOOTER_PERCENT)
            .genericPublish(NetworkTableType.kDouble.getValueStr());
  }

  public void setRPS(double RPS) {
    setRPS(RPS, RPS);
  }

  public void setRPS(double lowerRPS, double upperRPS) {
    lowerRequestedRPS = Math.signum(lowerRPS) * Math.min(Math.abs(lowerRPS), MAX_RPS);
    upperRequestedRPS = Math.signum(upperRPS) * Math.min(Math.abs(upperRPS), MAX_RPS);
    lowerMotor.setRPS(lowerRequestedRPS);
    upperMotor.setRPS(upperRequestedRPS);
  }

  public void coastOut() {
    lowerRequestedRPS = 0;
    upperRequestedRPS = 0;
    lowerMotor.applyNeutralMode(false);
    upperMotor.applyNeutralMode(false);
  }

  public void brake() {
    lowerMotor.applyNeutralMode(true);
    upperMotor.applyNeutralMode(true);
  }

  public Command shootSubwooferCommand() {
    return CommandUtil.indefiniteInstantCommand(() -> setRPS(SUBWOOFER_SPEED), this);
  }

  public Command prepSubwooferCommand() {
    return CommandUtil.indefiniteInstantCommand(() -> setRPS(SUBWOOFER_PREP_SPEED), this);
  }

  public Command shootAmpCommand() {
    return CommandUtil.indefiniteInstantCommand(
        () -> setRPS(AMP_SPEED_LOWER, AMP_SPEED_UPPER), this);
  }

  public Command reverseCommand() {
    return CommandUtil.indefiniteInstantCommand(() -> setRPS(REVERSE_SPEED), this);
  }

  public double getLowerRPS() {
    return lowerMotor.getRPS();
  }

  public double getUpperRPS() {
    return upperMotor.getRPS();
  }

  public boolean isStable(boolean requireAbove, boolean subwoofer) {
    double threshold =
        (subwoofer ? ShooterConstants.THRESHOLD_SUBWOOFER : ShooterConstants.THRESHOLD);
    if (requireAbove) {
      return Util.isInOneSidedRangeWithThreshold(getLowerRPS(), lowerRequestedRPS, threshold)
          && Util.isInOneSidedRangeWithThreshold(getUpperRPS(), upperRequestedRPS, threshold);
    } else {
      return Util.isInRangeWithThreshold(getLowerRPS(), lowerRequestedRPS, threshold)
          && Util.isInRangeWithThreshold(getUpperRPS(), upperRequestedRPS, threshold);
    }
  }

  public Command detectCurrentSpikeCommand() {
    return new DoneCycleCommand<>(new InstantCommand(), true)
        .withDoneCycles(
            DoneCycleMachine.supplierWithMinCycles(
                () ->
                    ((upperMotor.getTorqueCurrent() >= 20)
                        && (lowerMotor.getTorqueCurrent() >= 20)),
                2))
        .andThen(
            new DoneCycleCommand<>(new InstantCommand(), true)
                .withDoneCycles(
                    DoneCycleMachine.supplierWithMinCycles(
                        () ->
                            upperMotor.getTorqueCurrent() < 10
                                && lowerMotor.getTorqueCurrent() < 10,
                        2)));
  }

  @Override
  public void simulationPeriodic() {
    @SuppressWarnings("deprecation")
    TalonFXSimState lowerMotorSim = lowerMotor.getInternalMotor().getSimState();
    FlywheelSim flywheelSimValue = flywheelSim.get();
    flywheelSimValue.setInputVoltage(lowerMotorSim.getMotorVoltage());
    flywheelSimValue.update(SimManager.getDeltaSimTime());
    lowerMotorSim.setRotorVelocity(flywheelSimValue.getAngularVelocityRPM() / 60);
    LeveledSmartDashboard.INFO.putNumber("lower voltage", lowerMotorSim.getMotorVoltage());
  }

  @Override
  public void onStart(boolean sStopped) {
    lowerMotor.onStart();
    upperMotor.onStart();
  }

  @Override
  public void onStop(boolean sStopped) {
    lowerMotor.onStop();
    upperMotor.onStop();
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void updateSmartDashboard() {
    LeveledSmartDashboard.INFO.putNumber(
        "Current Drawn (shooter upper)", upperMotor.getSupplyCurrent());
    LeveledSmartDashboard.INFO.putNumber(
        "Current Drawn (shooter lower)", lowerMotor.getSupplyCurrent());
    LeveledSmartDashboard.INFO.putNumber("Shooter RPS (upper)", getUpperRPS());
    LeveledSmartDashboard.INFO.putNumber("Shooter RPS (lower)", getLowerRPS());

    ledPercent.setDouble(
        RaspberryPiLEDController.getSubsystemPercent(
            getLowerRPS(), lowerRequestedRPS, 0, SUBWOOFER_SPEED));
  }

  @Override
  public void setUpTrackables(LogManager logger) {
    logger
        .getLogFile("shooter")
        .setDefaultFrequency(5)
        .addTracker("Lower Requested RPS", () -> lowerRequestedRPS)
        .addTracker("Lower RPS", () -> getLowerRPS())
        .addTracker("Upper Requested RPS", () -> upperRequestedRPS)
        .addTracker("Upper RPS", () -> getUpperRPS());
  }

  @Override
  public void onSStop(boolean robotEnabled) {
    lowerMotor.onSStop(robotEnabled);
    upperMotor.onSStop(robotEnabled);
  }

  @Override
  public void onSRestart(boolean robotEnabled) {
    lowerMotor.onSRestart(robotEnabled);
    upperMotor.onSRestart(robotEnabled);
  }
}
