package frc4488.robot.subsystems.vortex;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc4488.lib.devices.REVMotor;
import frc4488.lib.devices.REVMotor.REVMotorPIDController;
import frc4488.lib.logging.LeveledSmartDashboard;
import frc4488.lib.logging.LogManager;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.lib.wpiextensions.ShockwaveSubsystemBase;
import frc4488.robot.subsystems.leds.RaspberryPiLEDController;

public class Climber extends ShockwaveSubsystemBase {

  public enum Position {
    UP("ClimberUpPos"),
    DOWN(0),
    DOWN_LIMIT(REVERSE_SOFT_LIMIT),
    BREAK();

    private final String key;
    private Double value;

    private Position(String key) {
      this.key = key;
      this.value = null;
    }

    private Position(double value) {
      this.key = null;
      this.value = value;
    }

    private Position() {
      this.key = null;
      this.value = null;
    }

    private void updateFromPrefs(PreferencesParser prefs) {
      if (value == null && this != BREAK) {
        value = prefs.getDouble(key);
      }
    }

    public double getRadians() {
      return value;
    }
  }

  private static final int CURRENT_LIMIT = 80;
  private static final float REVERSE_SOFT_LIMIT = -0.15f;
  private static final double SERVO_ON = 0;
  private static final double SERVO_OFF = 1;
  private double kP;
  private double kI;
  private double kD;
  private int climberID;

  private final Servo climberServo;
  private int servoID;
  private final REVMotor motor;
  private final REVMotorPIDController pidController;
  private Position requestedPosition;
  private final GenericPublisher ledPercent;

  public Climber(PreferencesParser prefs) {
    requestedPosition = Position.DOWN;

    updateFromPrefs(prefs);
    CANSparkMax motor = new CANSparkMax(climberID, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(CURRENT_LIMIT);
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) prefs.getDouble("ClimberSoftLimit"));
    motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    motor.setSoftLimit(SoftLimitDirection.kReverse, REVERSE_SOFT_LIMIT);
    motor.setInverted(prefs.getBoolean("ClimberInverted"));
    motor.burnFlash();
    this.motor = new REVMotor(motor, true, true, true);
    climberServo = new Servo(servoID);

    pidController = this.motor.getPIDController();

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);

    ledPercent =
        RaspberryPiLEDController.TABLE
            .getTopic(RaspberryPiLEDController.TableKeys.CLIMBER_PERCENT)
            .genericPublish(NetworkTableType.kDouble.getValueStr());

    setDefaultCommand(
        getBreakCommand()
            .alongWith(new InstantCommand(() -> setRatchet(true)))
            .withName("Climber Default command"));

    // motor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
  }

  public void updateFromPrefs(PreferencesParser prefs) {
    climberID = prefs.tryGetValue(prefs::getInt, "ClimberID", 32);
    servoID = prefs.tryGetValue(prefs::getInt, "ClimberServoID", 0);
    kP = prefs.tryGetValue(prefs::getDouble, "ClimberP", 0.0);
    kI = prefs.tryGetValue(prefs::getDouble, "ClimberI", 0.0);
    kD = prefs.tryGetValue(prefs::getDouble, "ClimberD", 0.0);

    for (Position pos : Position.values()) {
      pos.updateFromPrefs(prefs);
    }
  }

  public void setCoastMode(boolean coastMode) {
    motor.setCoastModeButtonState(coastMode);
  }

  public void setRequestedPosition(Position position) {
    requestedPosition = position;
    if (position != Position.BREAK) {
      pidController.setReference(position.getRadians(), ControlType.kPosition);
    }
  }

  public double getRequestedPosition() {
    return requestedPosition == Position.BREAK
        ? getActualPosition()
        : requestedPosition.getRadians();
  }

  public double getActualPosition() {
    return motor.getPosition();
  }

  public Command getMoveToCommand(Position pos) {
    return Commands.startEnd(() -> setRequestedPosition(pos), () -> {}, this)
        .withName("Move Climber [" + pos.getRadians() + "]");
  }

  public Command getBreakCommand() {
    return Commands.startEnd(() -> setRequestedPosition(Position.BREAK), () -> {}, this)
        .withName("Break Climber");
  }

  /** Use to allow the ratchet to release by causing the motor to move to as low as possible */
  public Command getMoveDownCommand() {
    return Commands.startEnd(() -> setRequestedPosition(Position.DOWN_LIMIT), () -> {}, this)
        .withName("Move Climber Down");
  }

  public void setRatchet(boolean engaged) {
    if (engaged) {
      climberServo.set(SERVO_ON);
    } else {
      climberServo.set(SERVO_OFF);
    }
  }

  public double getServoPosition() {
    return climberServo.get();
  }

  @Override
  public void onStart(boolean sStopped) {
    motor.onStart();
  }

  // TODO: add simulation code
  /* @Override
  public void simulationPeriodic() {
    TalonFXSimState sim = motor.getSimState();
    sim.setRawRotorPosition(getActualPosition() + sim.getMotorVoltage() * 0.05);
    sim.setRotorVelocity(sim.getMotorVoltage());
  }
  */

  @Override
  public void periodic() {
    if (requestedPosition == Position.BREAK) {
      motor.applyNeutralMode(true);
    } else {
      pidController.setReference(requestedPosition.getRadians(), ControlType.kPosition);
    }
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
    LeveledSmartDashboard.INFO.putNumber("Climber Position", getActualPosition());
    LeveledSmartDashboard.INFO.putNumber("Climber Requested Position", getRequestedPosition());
    LeveledSmartDashboard.INFO.putNumber("Climber Current", motor.getSupplyCurrent());
    LeveledSmartDashboard.INFO.putNumber("Servo Value", getServoPosition());

    ledPercent.setDouble(
        RaspberryPiLEDController.getSubsystemPercent(
            getActualPosition(),
            getRequestedPosition(),
            Position.DOWN.getRadians(),
            Position.UP.getRadians()));
  }

  @Override
  public void setUpTrackables(LogManager logger) {
    logger
        .getLogFile("Climber")
        .setDefaultFrequency(10)
        .addTracker("Position", () -> getActualPosition())
        .addTracker("Requested Position", () -> requestedPosition);
  }

  @Override
  public void onSStop(boolean robotEnabled) {
    motor.onSStop(robotEnabled);
  }

  @Override
  public void onSRestart(boolean robotEnabled) {
    motor.onSRestart(robotEnabled);
  }
}
