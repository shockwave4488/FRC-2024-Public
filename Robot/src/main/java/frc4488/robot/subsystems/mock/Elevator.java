package frc4488.robot.subsystems.mock;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc4488.lib.devices.REVMotor;
import frc4488.lib.devices.REVMotor.REVMotorPIDController;
import frc4488.lib.logging.LeveledSmartDashboard;
import frc4488.lib.logging.LogManager;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.lib.wpiextensions.ShockwaveSubsystemBase;
import java.util.function.Function;

public class Elevator extends ShockwaveSubsystemBase {
  private static final double CONVERSION_HEIGHT = 45;
  private double elevatorP;
  private double elevatorI;
  private double elevatorD;
  private final REVMotor motor;
  private final REVMotorPIDController pidController;

  public enum Position {
    TOP("ElevatorTopPos"),
    BOTTOM("ElevatorBottomPos");

    private final Function<PreferencesParser, Double> init;
    private Double value;

    private Position(Function<PreferencesParser, Double> init) {
      this.init = init;
      this.value = null;
    }

    private Position(String key) {
      this.init = prefs -> prefs.getDouble(key);
      this.value = null;
    }

    private Position(double value) {
      this.init = null;
      this.value = value;
    }

    private void updateFromPrefs(PreferencesParser prefs) {
      if (value == null) {
        value = init.apply(prefs);
      }
    }

    public double getValue() {
      return value;
    }
  }

  public Elevator(PreferencesParser prefs) {
    updateFromPrefs(prefs);

    CANSparkMax canMotor = new CANSparkMax(prefs.getInt("ElevatorID"), MotorType.kBrushless);
    canMotor.setSmartCurrentLimit(15);
    canMotor.setInverted(true);
    this.motor = new REVMotor(canMotor, true, true, true);

    pidController = motor.getPIDController();
    pidController.setP(elevatorP);
    pidController.setI(elevatorI);
    pidController.setD(elevatorD);
  }

  private void updateFromPrefs(PreferencesParser prefs) {
    elevatorP = prefs.getDouble("ElevatorP");
    elevatorI = prefs.getDouble("ElevatorI");
    elevatorD = prefs.getDouble("ElevatorD");
    for (Position pos : Position.values()) {
      pos.updateFromPrefs(prefs);
    }
  }

  public void goToHeight(double height) {
    // convert height to rotations
    double target = height * CONVERSION_HEIGHT;
    pidController.setReference(target, ControlType.kPosition, 0.1);
  }

  public Command goToHeightCommand(Position pos) {
    double targetHeight = pos.getValue();
    return Commands.startEnd(() -> goToHeight(targetHeight), () -> {}, this)
        .withName("Elevator Height [" + targetHeight + "]");
  }

  public double getHeight() {
    return motor.getPosition() / CONVERSION_HEIGHT;
  }

  @Override
  public void onSStop(boolean robotEnabled) {
    motor.onSStop(robotEnabled);
  }

  @Override
  public void onSRestart(boolean robotEnabled) {
    motor.onSRestart(robotEnabled);
  }

  @Override
  public void onStart(boolean sStopped) {
    motor.onStart();
  }

  @Override
  public void onStop(boolean sStopped) {
    motor.onStop();
  }

  @Override
  public void zeroSensors() {
    motor.setPosition(0);
  }

  @Override
  public void updateSmartDashboard() {
    LeveledSmartDashboard.HIGH.putNumber("elevator height", getHeight());
  }

  @Override
  public void setUpTrackables(LogManager logger) {
    logger.getLogFile("Elevator").addTracker("Height", () -> getHeight(), 10);
  }
}
