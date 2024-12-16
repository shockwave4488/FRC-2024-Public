package frc4488.robot.subsystems.supercell;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.util.Units;
import frc4488.lib.logging.LeveledSmartDashboard;
import frc4488.lib.logging.LogManager;
import frc4488.lib.wpiextensions.ShockwaveSubsystemBase;
import frc4488.robot.constants.Constants2023.RobotConstants.IntakeConstants;

@SuppressWarnings({"deprecation", "removal"})
public class Intake extends ShockwaveSubsystemBase {

  public enum Speed {
    FULL(1.0),
    FAST(0.65),
    SLOW(0.50),
    HOLD(0.1),
    STOPPED(0),
    REV_HOLD(-0.1),
    REV_SLOW(-0.50),
    REV_FAST(-0.65),
    REV_FULL(-1.0);

    private final double value;

    private Speed(double value) {
      this.value = value;
    }

    public double getValue() {
      return value;
    }
  }

  private static final double INTAKE_EDGE_MARGIN = 0.05;
  private final TimeOfFlight timeOfFlight; // Returns range in millimeters, not meters
  private final TalonFX motor;
  private Speed speed = Speed.STOPPED;

  public Intake(int motorPort, int timeOfFlightPort) {

    motor = new TalonFX(motorPort);
    timeOfFlight = new TimeOfFlight(timeOfFlightPort);

    motor.configFactoryDefault();
    // motor.setInverted(true); Not needed right now.
    motor.setNeutralMode(NeutralMode.Brake);
    motor.enableVoltageCompensation(true);
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 35, 0));
  }

  @Override
  public void onStart(boolean sStopped) {}

  @Override
  public void onStop(boolean sStopped) {}

  @Override
  public void zeroSensors() {}

  @Override
  public void updateSmartDashboard() {
    LeveledSmartDashboard.INFO.putNumber("Intake > Cone Position", getConePosition());
    LeveledSmartDashboard.INFO.putNumber("Intake > TOF", timeOfFlight.getRange());
    LeveledSmartDashboard.INFO.putNumber("Intake > Motor Current", motor.getStatorCurrent());
  }

  @Override
  public void setUpTrackables(LogManager logger) {
    int loggingFrequency = 5;

    logger
        .getLogFile("Intake_State")
        .setDefaultFrequency(loggingFrequency)
        .addTracker("Intake_Position", () -> this.getConePosition())
        .addTracker("Intake_Time_Of_Flight", () -> timeOfFlight.getRange())
        .addTracker("Motor_Current", () -> motor.getStatorCurrent());
  }

  public void setSpeed(Speed speed) {
    if (isSStopped()) {
      return;
    }
    this.speed = speed;
    motor.set(ControlMode.PercentOutput, speed.getValue());
  }

  public double getMotorCurrent() {
    return motor.getStatorCurrent();
  }

  public Speed getSpeed() {
    return speed;
  }

  /**
   * @return The position of the cone, relative to the center of the intake
   */
  public double getConePosition() {
    double pos =
        timeOfFlight.getRange() / 1000.0
            - IntakeConstants.FLIGHT_SENSORS_MAX_VALUE / 2
            + Units.inchesToMeters(2.5);
    return Math.min(
        IntakeConstants.FLIGHT_SENSORS_MAX_VALUE / 2,
        Math.max(-IntakeConstants.FLIGHT_SENSORS_MAX_VALUE / 2, pos));
  }

  public boolean isPieceLoaded() {
    return timeOfFlight.getRange() < IntakeConstants.FLIGHT_SENSORS_MAX_VALUE - INTAKE_EDGE_MARGIN;
  }

  @Override
  public void onSStop(boolean robotEnabled) {
    motor.set(ControlMode.Disabled, 0);
  }

  @Override
  public void onSRestart(boolean robotEnabled) {}
}
