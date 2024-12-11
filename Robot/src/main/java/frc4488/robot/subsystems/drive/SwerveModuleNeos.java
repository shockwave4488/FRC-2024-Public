package frc4488.robot.subsystems.drive;

import static frc4488.robot.constants.Constants.TAU;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc4488.lib.controlsystems.SimPID;
import frc4488.lib.drive.SwerveParameters;
import frc4488.lib.logging.LogManager;
import frc4488.lib.misc.Timed;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.lib.sensors.Potentiometer;

/** A SwerveModule class that operates Spark Neos */
public class SwerveModuleNeos extends SwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;
  private final Potentiometer turningEncoder;

  private final SparkPIDController drivePIDController;
  private final SimPID turningPIDController;

  private boolean sStopped;

  /**
   * Constructs a SwerveModuleNeo.
   *
   * @param parameters Module-specific parameters
   */
  public SwerveModuleNeos(SwerveParameters parameters, LogManager logger, PreferencesParser prefs) {
    super(parameters, logger, parameters.absoluteEncoderResolution);
    driveMotor = new CANSparkMax(parameters.driveMotorChannel, MotorType.kBrushless);
    turningMotor = new CANSparkMax(parameters.turningMotorChannel, MotorType.kBrushless);
    turningEncoder = new Potentiometer(parameters.turningEncoderChannel);
    driveMotor.setClosedLoopRampRate(1);
    driveMotor.setIdleMode(IdleMode.kBrake);
    turningMotor.setIdleMode(IdleMode.kBrake);
    drivePIDController = driveMotor.getPIDController();
    drivePIDController.setP(prefs.tryGetValue(prefs::getDouble, "SwerveNeosDriveP", 0.0));
    drivePIDController.setI(prefs.tryGetValue(prefs::getDouble, "SwerveNeosDriveI", 0.0));
    drivePIDController.setD(prefs.tryGetValue(prefs::getDouble, "SwerveNeosDriveD", 0.0));
    drivePIDController.setFF(prefs.tryGetValue(prefs::getDouble, "SwerveNeosDriveFF", 0.0));
    turningPIDController =
        new SimPID(
            prefs.tryGetValue(prefs::getDouble, "SwerveNeosTurnP", 0.0),
            prefs.tryGetValue(prefs::getDouble, "SwerveNeosTurnI", 0.0),
            prefs.tryGetValue(prefs::getDouble, "SwerveNeosTurnD", 0.0));
    turningPIDController.setWrapAround(0, 4096);
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    if (sStopped) {
      return;
    }

    super.setDesiredState(desiredState);

    double targetAngleTicks = desiredModuleAngle / TAU * angleEncoderResolution;

    turningPIDController.setDesiredValue(targetAngleTicks);
    double power = turningPIDController.calcPID(getAngleRadians() / TAU * angleEncoderResolution);
    turningMotor.set(power);

    // Calculate the drive output from the drive PID controller.
    drivePIDController.setReference(
        metersPerSecToRPM(desiredModuleSpeed), CANSparkMax.ControlType.kVelocity);
  }

  @Override
  public double getAngleRadians() {
    return (angleEncoderResolution
            - ((turningEncoder.get() - parameters.absoluteEncoderOffset + angleEncoderResolution)
                % angleEncoderResolution))
        / angleEncoderResolution
        * TAU;
  }

  @Override
  public Timed<Double> getDriveRadians() {
    return new Timed<>(Timer.getFPGATimestamp(), driveMotor.getEncoder().getPosition() * TAU);
  }

  @Override
  public double getDriveRPM() {
    return driveMotor.getEncoder().getVelocity(); // get speed from spark
  }

  @Override
  public void zeroSensors() {
    // Always uses absolute encoder
  }

  public void onSStop(boolean robotEnabled) {
    sStopped = true;
    driveMotor.setIdleMode(IdleMode.kCoast);
    turningMotor.setIdleMode(IdleMode.kCoast);
    driveMotor.set(0);
    turningMotor.set(0);
  }

  @Override
  public void onSRestart(boolean robotEnabled) {
    sStopped = false;
    if (robotEnabled) {
      driveMotor.setIdleMode(IdleMode.kBrake);
      driveMotor.setIdleMode(IdleMode.kBrake);
    }
  }

  @Override
  public boolean isEncoderConnected() {
    return Double.isFinite(turningEncoder.get());
  }
}
