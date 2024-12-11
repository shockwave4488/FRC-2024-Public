package frc4488.robot.subsystems.vortex;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc4488.lib.commands.DoneCycleCommand;
import frc4488.lib.commands.LogCommand;
import frc4488.lib.controlsystems.DoneCycleMachine;
import frc4488.lib.devices.REVMotor;
import frc4488.lib.devices.REVMotor.REVMotorPIDController;
import frc4488.lib.logging.LeveledSmartDashboard;
import frc4488.lib.logging.LogManager;
import frc4488.lib.math.PoseEstimationUtil;
import frc4488.lib.misc.Util;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.lib.wpiextensions.ShockwaveSubsystemBase;
import frc4488.robot.constants.Constants;
import frc4488.robot.constants.Constants2024;
import frc4488.robot.constants.Constants2024.RobotConstants.ArmConstants;
import frc4488.robot.subsystems.drive.SwerveDrive;
import frc4488.robot.subsystems.leds.RaspberryPiLEDController;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class Arm extends ShockwaveSubsystemBase {

  private static final double WRAP_AROUND_AVOIDANCE = -Math.PI;
  public static final double LENGTH = 0.6;
  public static final Translation3d ARM_ORIGIN = new Translation3d(-0.17, 0, 0.26);

  public enum Position {
    AMP("ArmAmpPos"),
    SPEAKER("ArmSpeakerPos"),
    PODIUM("ArmPodiumPos"),
    INTAKE("ArmIntakePos"),
    CROUCH("ArmCrouchPos"),
    ZERO(0),
    HORIZONTAL_SHOOTER(prefs -> prefs.getDouble("ArmOutputAngle") + Math.PI / 2);

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

    public double getRadians() {
      return value;
    }
  }

  private int armID;
  private int followID;

  @SuppressWarnings("unused")
  private final Optional<SingleJointedArmSim> armSim;

  private double requestedPosition;
  private REVMotorPIDController pidController;
  private final REVMotor armMotor;
  private REVMotor follow;
  private static final double MAX_ARM_ANGLE = 1.65;
  private static final double MIN_ARM_ANGLE = -0.06;
  private static final double I_ZONE = 0.05;
  private static final double ERROR_SAMPLING_STEP = 0.01;
  private double MIN_OUTPUT = -1;
  private double MAX_OUTPUT = 1;
  private static final double DEFAULT_OFFSET = -2.173;
  private static final int ARM_CURRENT_LIMIT = 40; // In amps
  private double kP;
  private double kI;
  private double kD;
  private double kS;
  private double kG;
  private double kV;
  private final ArmFeedforward feedForward;
  private double encoderOffset;
  private double outputAngle; // Angle between perpendicular to arm and actual output
  private final GenericPublisher ledPercent;

  private TrapezoidProfile trapezoidProfile =
      new TrapezoidProfile(
          new Constraints(
              Constants2024.RobotConstants.MAX_ARM_VELOCITY,
              Constants2024.RobotConstants.MAX_ARM_ACCELERATION));
  private TrapezoidProfile.State profileGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State profileSetpoint = new TrapezoidProfile.State();

  public Arm(PreferencesParser prefs) {
    updateFromPrefs(prefs);
    CANSparkMax armMotor = new CANSparkMax(armID, MotorType.kBrushless);
    CANSparkMax follow = new CANSparkMax(followID, MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    follow.restoreFactoryDefaults();
    this.armMotor = new REVMotor(armMotor, true, true, false);
    this.follow = new REVMotor(follow, true, true, false);
    this.armMotor.setUseAbsoluteEncoder(true);
    armMotor.getAbsoluteEncoder().setInverted(true);
    armMotor
        .getAbsoluteEncoder()
        .setZeroOffset(((encoderOffset + WRAP_AROUND_AVOIDANCE) / Constants.TAU + 1) % 1);

    if (RobotBase.isSimulation()) {
      SingleJointedArmSim armSim =
          new SingleJointedArmSim(
              DCMotor.getNEO(1), 180, 5, LENGTH, MIN_ARM_ANGLE, MAX_ARM_ANGLE, true, MIN_ARM_ANGLE);
      this.armSim = Optional.of(armSim);
      REVPhysicsSim.getInstance().addSparkMax(armMotor, DCMotor.getNEO(1));
    } else {
      this.armSim = Optional.empty();
    }

    armMotor.setSmartCurrentLimit(ARM_CURRENT_LIMIT);
    follow.setSmartCurrentLimit(ARM_CURRENT_LIMIT);
    follow.follow(armMotor, true);
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotor.setSoftLimit(SoftLimitDirection.kForward, (float) convertToRevValue(MAX_ARM_ANGLE));
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) convertToRevValue(MIN_ARM_ANGLE));

    pidController = this.armMotor.getPIDController();
    pidController.enableWrapping(0, 1);
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(I_ZONE);
    pidController.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);

    armMotor.burnFlash();
    follow.burnFlash();

    feedForward = new ArmFeedforward(kS, kG, kV);

    ledPercent =
        RaspberryPiLEDController.TABLE
            .getTopic(RaspberryPiLEDController.TableKeys.ARM_PERCENT)
            .genericPublish(NetworkTableType.kDouble.getValueStr());

    setDefaultCommand(getMoveToCommand(Position.CROUCH));
    resetGoal();
  }

  /**
   * Converts arm radians to rotations in the range 0-1, taking into account wrap around avoidance
   */
  private double convertToRevValue(double radians) {
    return ((radians - WRAP_AROUND_AVOIDANCE) / Constants.TAU + 1) % 1;
  }

  private void resetGoal() {
    profileGoal = new State(getActualPosition(), 0);
    profileSetpoint = new State(getActualPosition(), 0);
  }

  private void updateFromPrefs(PreferencesParser prefs) {
    kP = prefs.tryGetValue(prefs::getDouble, "ArmP", 0.0);
    kI = prefs.tryGetValue(prefs::getDouble, "ArmI", 0.0);
    kD = prefs.tryGetValue(prefs::getDouble, "ArmD", 0.0);
    kS = prefs.tryGetValue(prefs::getDouble, "ArmS", 0.0);
    kG = prefs.tryGetValue(prefs::getDouble, "ArmG", 0.0);
    kV = prefs.tryGetValue(prefs::getDouble, "ArmV", 0.0);
    armID = prefs.tryGetValue(prefs::getInt, "RightArmID", 30);
    followID = prefs.tryGetValue(prefs::getInt, "LeftArmID", 31);
    encoderOffset = prefs.tryGetValue(prefs::getDouble, "ArmEncoderOffset", DEFAULT_OFFSET);
    outputAngle = prefs.tryGetValue(prefs::getDouble, "ArmOutputAngle", 0.0);

    for (Position pos : Position.values()) {
      pos.updateFromPrefs(prefs);
    }
  }

  public void setCoastMode(boolean coastMode) {
    armMotor.setCoastModeButtonState(coastMode);
    follow.setCoastModeButtonState(coastMode);
  }

  public void setTargetAngle(Rotation2d position) {
    requestedPosition = position.getRadians();
    profileGoal = new State(requestedPosition, 0.0);
  }

  public double getRequestedPosition() {
    return requestedPosition;
  }

  public double getActualPosition() {
    return armMotor.getPosition() * Constants.TAU + WRAP_AROUND_AVOIDANCE;
  }

  public boolean isStable(boolean subwoofer) {
    return Util.isInRangeWithThreshold(
        getActualPosition(),
        requestedPosition,
        subwoofer ? ArmConstants.THRESHOLD_SUBWOOFER : ArmConstants.THRESHOLD);
  }

  public Command getMoveToCommand(Position pos) {
    Rotation2d angle = Rotation2d.fromRadians(pos.getRadians());
    return Commands.startEnd(() -> setTargetAngle(angle), () -> {}, this)
        .withName("Move Arm [" + angle + "]");
  }

  public Command getMoveToCommand(Position pos, Supplier<Double> offset) {
    return Commands.startEnd(
            () -> setTargetAngle(Rotation2d.fromRadians(pos.getRadians() + offset.get())),
            () -> {},
            this)
        .withName("Move Arm [" + pos.getRadians() + "]");
  }

  public Command getTrackCommand(
      SwerveDrive swerve, double shootingMps, Supplier<Double> angleOffset) {
    return LogCommand.repeat(
            new InstantCommand(
                () -> {
                  Translation3d pos =
                      new Pose3d(swerve.getOdometry())
                          .plus(new Transform3d(ARM_ORIGIN, new Rotation3d()))
                          .getTranslation();
                  Optional<Translation3d> speakerOptional =
                      PoseEstimationUtil.accountForRobotVelocity(
                          Constants2024.FieldConstants.getInstance().speakerOpeningCenter,
                          swerve,
                          shootingMps);
                  if (speakerOptional.isEmpty()) {
                    return;
                  }
                  Translation3d speaker = speakerOptional.get();

                  calcArmAngle(
                          speaker.toTranslation2d().getDistance(pos.toTranslation2d()),
                          speaker.getZ() - pos.getZ(),
                          shootingMps)
                      .ifPresent(
                          armAngle ->
                              setTargetAngle(Rotation2d.fromRadians(armAngle + angleOffset.get())));
                },
                this))
        .withName("Track");
  }

  private Optional<Double> calcArmAngle(
      double groundDist, double verticalDist, double shootingMps) {
    // Samples the arm angle error and finds the closest to 0 on the negative and positive sides

    double lowTheta = Double.NEGATIVE_INFINITY;
    double lowError = Double.NEGATIVE_INFINITY;
    double highTheta = Double.POSITIVE_INFINITY;
    double highError = Double.POSITIVE_INFINITY;

    for (int i = 0; i <= (MAX_ARM_ANGLE - MIN_ARM_ANGLE) / ERROR_SAMPLING_STEP; i++) {
      double armAngle = MIN_ARM_ANGLE + i * ERROR_SAMPLING_STEP;

      double error =
          calcArmAngleError(groundDist, verticalDist, shootingMps, armAngle).orElse(Double.NaN);
      if (error == 0) {
        return Optional.of(armAngle);
      } else if (error < 0) {
        if (lowError < error) {
          lowTheta = armAngle;
          lowError = error;
          if (Double.isFinite(highTheta)) {
            break;
          }
        }
      } else if (error > 0) {
        if (error < highError) {
          highTheta = armAngle;
          highError = error;
          if (Double.isFinite(lowTheta)) {
            break;
          }
        }
      }
    }

    if (!Double.isFinite(lowTheta) || !Double.isFinite(highTheta)) {
      return Optional.empty();
    }

    // Creates a linear approximation of the error function using the two points and calculates the
    // zero

    double slope = (lowError - highError) / (lowTheta - highTheta);
    return Optional.of(lowTheta - lowError / slope);
  }

  private Optional<Double> calcArmAngleError(
      double groundDist, double verticalDist, double shootingMps, double armAngle) {
    // Based on docs/calcArmAngleError_proof.png

    double thetaTotal = armAngle + -outputAngle + Math.PI / 2;
    double vx = shootingMps * Math.cos(thetaTotal);
    double vy = shootingMps * Math.sin(thetaTotal);

    double t = -(groundDist + LENGTH * Math.cos(armAngle)) / vx;
    if (t <= 0) {
      return Optional.empty();
    }

    double y = Constants.GRAVITY / 2 * t * t + vy * t + LENGTH * Math.sin(armAngle);

    return Optional.of(y - verticalDist);
  }

  public Command getAimCommand(
      SwerveDrive swerve, double shootingMps, Supplier<Double> angleOffset) {
    return new DoneCycleCommand<>(getTrackCommand(swerve, shootingMps, angleOffset), true)
        .withDoneCycles(
            DoneCycleMachine.supplierWithMinCycles(
                () -> Math.abs(getActualPosition() - getRequestedPosition()) < 0.05, 10))
        .withName("Aim");
  }

  @Override
  public void onStart(boolean sStopped) {
    armMotor.onStart();
    follow.onStart();
    resetGoal();
  }

  @Override
  public void onStop(boolean sStopped) {
    armMotor.onStop();
    follow.onStop();
  }

  @Override
  public void simulationPeriodic() {
    // TODO: replace with the REV version later
    // REVPhysicsSim motorSim = armMotor.getSimState();
    // SingleJointedArmSim armSimValue = armSim.get();
    // armSimValue.setInputVoltage(motorSim.getMotorVoltage());
    // armSimValue.update(SimManager.getDeltaSimTime());
    // motorSim.setRawRotorPosition(armSimValue.getAngleRads());
    // motorSim.setRotorVelocity(armSimValue.getVelocityRadPerSec());
  }

  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      setTargetAngle(Rotation2d.fromRadians(getActualPosition()));
    }

    profileSetpoint =
        trapezoidProfile.calculate(Constants.ROBOT_LOOP_SECONDS, profileSetpoint, profileGoal);
    pidController.setReference(
        (profileSetpoint.position - WRAP_AROUND_AVOIDANCE) / (Constants.TAU),
        CANSparkMax.ControlType.kPosition,
        feedForward.calculate(profileSetpoint.position, profileSetpoint.velocity));
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void updateSmartDashboard() {
    // LeveledSmartDashboard.INFO.putNumber(
    //     "Arm Raw Absolute Encoder (rad)",
    //     MathUtil.angleModulus(
    //         (armMotor.getAbsoluteEncoder().getPosition()
    //                 + armMotor.getAbsoluteEncoder().getZeroOffset())
    //             * Constants.TAU)); // getPosition returns the offseted value, so it must be
    // undone
    LeveledSmartDashboard.INFO.putNumber("Arm Requested Position (rad)", requestedPosition);
    LeveledSmartDashboard.INFO.putNumber("Arm Absolute Encoder (rad)", getActualPosition());
    LeveledSmartDashboard.INFO.putNumber("Arm setpoint (rad)", profileSetpoint.position);
    LeveledSmartDashboard.INFO.putNumber("Current Drawn (arm right)", armMotor.getSupplyCurrent());
    LeveledSmartDashboard.INFO.putNumber("Current Drawn (arm left)", follow.getSupplyCurrent());

    ledPercent.setDouble(
        RaspberryPiLEDController.getSubsystemPercent(
            getActualPosition(), requestedPosition, MIN_ARM_ANGLE, MAX_ARM_ANGLE));
  }

  @Override
  public void setUpTrackables(LogManager logger) {
    logger
        .getLogFile("Arm")
        .setDefaultFrequency(10)
        .addTracker("Position", () -> getActualPosition())
        .addTracker("Requested Position", () -> requestedPosition)
        .addTracker("Setpoint", () -> profileSetpoint.position);
  }

  @Override
  public void onSStop(boolean robotEnabled) {
    armMotor.onSStop(robotEnabled);
    follow.onSStop(robotEnabled);
  }

  @Override
  public void onSRestart(boolean robotEnabled) {
    armMotor.onSRestart(robotEnabled);
    follow.onSRestart(robotEnabled);
  }
}
