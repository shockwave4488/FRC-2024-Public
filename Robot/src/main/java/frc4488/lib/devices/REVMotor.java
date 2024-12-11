package frc4488.lib.devices;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;

public class REVMotor extends Motor<CANSparkMax> {

  public class REVMotorPIDController {
    public void disableWrapping() {
      motor.getPIDController().setPositionPIDWrappingEnabled(false);
    }

    public void enableWrapping(double min, double max) {
      motor.getPIDController().setPositionPIDWrappingEnabled(true);
      motor.getPIDController().setPositionPIDWrappingMinInput(min);
      motor.getPIDController().setPositionPIDWrappingMaxInput(max);
    }

    public void setP(double gain) {
      motor.getPIDController().setP(gain);
    }

    public void setI(double gain) {
      motor.getPIDController().setI(gain);
    }

    public void setD(double gain) {
      motor.getPIDController().setD(gain);
    }

    public void setFF(double gain) {
      motor.getPIDController().setFF(gain);
    }

    public void setIZone(double IZone) {
      motor.getPIDController().setIZone(IZone);
    }

    public void setOutputRange(double min, double max) {
      motor.getPIDController().setOutputRange(min, max);
    }

    public void setReference(double value, ControlType ctrl) {
      if (!isSStopped()) {
        motor.getPIDController().setReference(value, ctrl);
      }
    }

    public void setReference(double value, ControlType ctrl, double arbFeedforward) {
      if (!isSStopped()) {
        motor.getPIDController().setReference(value, ctrl, 0, arbFeedforward);
      }
    }
  }

  private final CANSparkMax motor;
  private boolean useAbsoluteEncoder;
  private REVMotorPIDController pid;

  public REVMotor(
      CANSparkMax motor, boolean disabledBrake, boolean enabledBrake, boolean sStoppedBrake) {
    super(disabledBrake, enabledBrake, sStoppedBrake);
    this.motor = motor;
    this.useAbsoluteEncoder = false;
    applyNeutralMode(disabledBrake);
  }

  public void setUseAbsoluteEncoder(boolean use) {
    useAbsoluteEncoder = use;
    if (pid != null) {
      motor
          .getPIDController()
          .setFeedbackDevice(use ? motor.getAbsoluteEncoder() : motor.getEncoder());
    }
  }

  @Override
  @Deprecated
  @SuppressFBWarnings("EI_EXPOSE_REP")
  public CANSparkMax getInternalMotor() {
    return motor;
  }

  @SuppressFBWarnings("EI_EXPOSE_REP")
  public REVMotorPIDController getPIDController() {
    if (pid == null) {
      pid = new REVMotorPIDController();
      if (useAbsoluteEncoder) {
        motor.getPIDController().setFeedbackDevice(motor.getAbsoluteEncoder());
      }
    }
    return pid;
  }

  @Override
  protected void setPercentInternal(double percent) {
    motor.set(percent);
  }

  @Override
  protected void setNeutralModeInternal(boolean brake) {
    motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  protected void applyNeutralModeInternal() {
    motor.set(0);
  }

  @Override
  public void setPosition(double position) {
    motor.getEncoder().setPosition(position);
  }

  @Override
  public double getPosition() {
    return useAbsoluteEncoder
        ? motor.getAbsoluteEncoder().getPosition()
        : motor.getEncoder().getPosition();
  }

  @Override
  public double getPercent() {
    return motor.get();
  }

  @Override
  public double getSupplyCurrent() {
    return motor.getOutputCurrent();
  }
}
