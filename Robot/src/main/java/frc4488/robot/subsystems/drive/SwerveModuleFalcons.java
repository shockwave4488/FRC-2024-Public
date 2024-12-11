package frc4488.robot.subsystems.drive;

import static frc4488.robot.constants.Constants.TAU;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc4488.lib.devices.Conductor;
import frc4488.lib.drive.SwerveParameters;
import frc4488.lib.logging.LeveledSmartDashboard;
import frc4488.lib.logging.LogLevel;
import frc4488.lib.logging.LogManager;
import frc4488.lib.misc.Timed;
import frc4488.lib.misc.Util;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.lib.sensors.MagneticEncoder;
import frc4488.lib.sensors.absoluteencoder.AbsoluteEncoder;
import frc4488.lib.sensors.absoluteencoder.CANEncoder;

public class SwerveModuleFalcons extends SwerveModule {
  private static final double DEFAULT_DRIVE_P = 0.0004;
  private static final double DEFAULT_DRIVE_I = 0;
  private static final double DEFAULT_DRIVE_D = 0;
  private static final double DEFAULT_DRIVE_FF = 0.055;
  private static final double DEFAULT_TURN_P = 0.2;
  private static final double DEFAULT_TURN_I = 0;
  private static final double DEFAULT_TURN_D = 0;

  private static final double DRIVE_LIMIT_CURRENT_THRESHOLD = 90; // 50
  private static final double DRIVE_SUPPLY_LIMITED_CURRENT = 60; // 40
  private static final double DRIVE_STATOR_LIMITED_CURRENT = 300;
  private static final double TURN_LIMIT_CURRENT_THRESHOLD = 50;
  private static final double TURN_LIMITED_CURRENT = 40;
  private static final double LIMIT_CURRENT_AFTER = 0.5;

  private final TalonFX driveMotor;
  private final TalonFX turningMotor;
  private final AbsoluteEncoder turningMagneticEncoder;
  private boolean sStopped;

  /**
   * Constructs a SwerveModuleFalcon.
   *
   * @param parameters Module-specific parameters
   */
  public SwerveModuleFalcons(
      SwerveParameters parameters,
      Conductor conductor,
      LogManager logger,
      PreferencesParser prefs) {
    super(parameters, logger, parameters.relativeTurningEncoderResolution);

    boolean success = true;
    String canbus = prefs.tryGetValue(prefs::getString, "CanivoreBusName", "rio");

    driveMotor = new TalonFX(parameters.driveMotorChannel, canbus);
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig
        .CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(DRIVE_SUPPLY_LIMITED_CURRENT)
        .withSupplyCurrentThreshold(DRIVE_LIMIT_CURRENT_THRESHOLD)
        .withSupplyTimeThreshold(LIMIT_CURRENT_AFTER)
        .withStatorCurrentLimit(DRIVE_STATOR_LIMITED_CURRENT);
    driveConfig
        .Slot0
        .withKP(prefs.tryGetValue(prefs::getDouble, "SwerveFalconsDriveP", DEFAULT_DRIVE_P))
        .withKI(prefs.tryGetValue(prefs::getDouble, "SwerveFalconsDriveI", DEFAULT_DRIVE_I))
        .withKD(prefs.tryGetValue(prefs::getDouble, "SwerveFalconsDriveD", DEFAULT_DRIVE_D))
        .withKV(prefs.tryGetValue(prefs::getDouble, "SwerveFalconsDriveFF", DEFAULT_DRIVE_FF));
    success &=
        Util.retry(() -> driveMotor.getConfigurator().apply(driveConfig), StatusCode::isOK, 5);
    success &=
        Util.retry(
            () -> driveMotor.getRotorPosition().setUpdateFrequency(100), StatusCode::isOK, 5);
    success &=
        Util.retry(
            () -> driveMotor.getRotorVelocity().setUpdateFrequency(100), StatusCode::isOK, 5);

    turningMotor = new TalonFX(parameters.turningMotorChannel, canbus);
    TalonFXConfiguration turningConfig = new TalonFXConfiguration();
    turningConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    turningConfig
        .CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(TURN_LIMITED_CURRENT)
        .withSupplyCurrentThreshold(TURN_LIMIT_CURRENT_THRESHOLD)
        .withSupplyTimeThreshold(LIMIT_CURRENT_AFTER);
    turningConfig
        .Slot0
        .withKP(prefs.tryGetValue(prefs::getDouble, "SwerveFalconsTurnP", DEFAULT_TURN_P))
        .withKI(prefs.tryGetValue(prefs::getDouble, "SwerveFalconsTurnI", DEFAULT_TURN_I))
        .withKD(prefs.tryGetValue(prefs::getDouble, "SwerveFalconsTurnD", DEFAULT_TURN_D));
    success &=
        Util.retry(() -> turningMotor.getConfigurator().apply(turningConfig), StatusCode::isOK, 5);
    success &=
        Util.retry(
            () -> turningMotor.getRotorPosition().setUpdateFrequency(100), StatusCode::isOK, 5);
    success &=
        Util.retry(
            () -> turningMotor.getRotorVelocity().setUpdateFrequency(100), StatusCode::isOK, 5);

    if (!success) {
      log.println(
          LogLevel.ERROR_CONSOLE, "Failed to initialize module: " + parameters.modulePosition);
    }

    if (canbus.equals("rio")) {
      turningMagneticEncoder =
          new MagneticEncoder(
              parameters.turningEncoderChannel,
              parameters.absoluteEncoderResolution,
              parameters.absoluteEncoderOffset,
              false,
              4095);
    } else {
      turningMagneticEncoder =
          new CANEncoder(
              parameters.turningEncoderChannel, canbus, parameters.absoluteEncoderOffset);
    }
    zeroSensors();
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
    turningMotor.setNeutralMode(NeutralModeValue.Brake);
    driveMotor.setControl(new CoastOut());
    turningMotor.setControl(new CoastOut());

    if (conductor != null) {
      conductor.addInstrument(driveMotor, true);
      conductor.addInstrument(turningMotor, true);
    }

    log.addTracker("Mag Angle", () -> Math.toDegrees(turningMagneticEncoder.getAngleOffset()));
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    if (sStopped) {
      return;
    }

    super.setDesiredState(desiredState);

    // Calculate the drive output from the drive PID controller.
    double speedRPM = metersPerSecToRPM(desiredModuleSpeed);
    double currentAngleRadians = getAngleRadians();
    double currentAngleRadiansMod = MathUtil.angleModulus(currentAngleRadians);

    // Get target angle by adding the delta of the desired bounded angle and actual bounded angle to
    // the current unbounded angle
    double targetAngleRadians =
        (currentAngleRadians + MathUtil.angleModulus(desiredModuleAngle - currentAngleRadiansMod));

    driveMotor.setControl(new VelocityVoltage(speedRPM / 60));
    turningMotor.setControl(
        new PositionVoltage(targetAngleRadians / TAU * parameters.turnGearRatio));
  }

  @Override
  public Timed<Double> getDriveRadians() {
    StatusSignal<Double> rotations = driveMotor.getRotorPosition();
    return new Timed<>(
        Timer.getFPGATimestamp() - rotations.getTimestamp().getLatency(),
        rotations.getValue() * TAU / parameters.driveGearRatio);
  }

  @Override
  public double getDriveRPM() {
    return driveMotor.getRotorVelocity().getValue() * 60;
  }

  @Override
  public double getAngleRadians() {
    return turningMotor.getRotorPosition().getValue() * TAU / parameters.turnGearRatio;
  }

  @Override
  public void onStart(boolean sStopped) {
    zeroSensors();
    if (!sStopped) {
      driveMotor.setControl(new StaticBrake());
      turningMotor.setControl(new StaticBrake());
    }
  }

  @Override
  public void onStop(boolean sStopped) {
    driveMotor.setControl(new CoastOut());
    turningMotor.setControl(new CoastOut());
  }

  @Override
  public void zeroSensors() {
    Util.retry(
        () ->
            turningMotor.setPosition(
                turningMagneticEncoder.getAngleOffset() // radians
                    / TAU
                    * parameters.turnGearRatio),
        StatusCode::isOK,
        5);
  }

  @Override
  public boolean isEncoderConnected() {
    return Double.isFinite(turningMagneticEncoder.getAngle());
  }

  @Override
  public void updateSmartDashboard() {
    super.updateSmartDashboard();
    LeveledSmartDashboard.INFO.putNumber(
        parameters.modulePosition + " Mag Angle (Radians)",
        turningMagneticEncoder.getAngleOffset());
    LeveledSmartDashboard.INFO.putNumber(
        parameters.modulePosition + " Mag Angle Raw (Radians)", turningMagneticEncoder.getAngle());
    LeveledSmartDashboard.INFO.putNumber(
        parameters.modulePosition + " Rotor Position (Radians)",
        driveMotor.getRotorPosition().getValueAsDouble() * TAU / parameters.driveGearRatio);
    LeveledSmartDashboard.INFO.putNumber(
        parameters.modulePosition + " Rotor Position (Rotations)",
        driveMotor.getRotorPosition().getValueAsDouble());
  }

  @Override
  public void onSStop(boolean robotEnabled) {
    sStopped = true;
    driveMotor.setControl(new CoastOut());
    turningMotor.setControl(new CoastOut());
  }

  @Override
  public void onSRestart(boolean robotEnabled) {
    sStopped = false;
    if (robotEnabled) {
      driveMotor.setControl(new StaticBrake());
      turningMotor.setControl(new StaticBrake());
    }
  }
}
