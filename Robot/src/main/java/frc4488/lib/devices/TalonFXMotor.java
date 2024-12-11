package frc4488.lib.devices;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;

public class TalonFXMotor extends Motor<TalonFX> {

  private final TalonFX motor;
  private final VelocityVoltage rpsControl;
  private final NeutralOut neutralControl;
  private double zero;

  public TalonFXMotor(
      TalonFX motor, boolean disabledBrake, boolean enabledBrake, boolean sStoppedBrake) {
    super(disabledBrake, enabledBrake, sStoppedBrake);
    this.motor = motor;
    this.rpsControl = new VelocityVoltage(0);
    this.neutralControl = new NeutralOut();
    applyNeutralMode(disabledBrake);
  }

  @Override
  @Deprecated
  @SuppressFBWarnings("EI_EXPOSE_REP")
  public TalonFX getInternalMotor() {
    return motor;
  }

  @Override
  protected void setPercentInternal(double percent) {
    motor.set(percent);
  }

  public void setRPS(double rps) {
    if (!isSStopped()) {
      motor.setControl(rpsControl.withVelocity(rps));
    }
  }

  @Override
  protected void setNeutralModeInternal(boolean brake) {
    motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  protected void applyNeutralModeInternal() {
    motor.setControl(neutralControl);
  }

  @Override
  public void setPosition(double position) {
    zero = motor.getRotorPosition().getValueAsDouble() - position;
  }

  @Override
  public double getPosition() {
    return motor.getRotorPosition().getValueAsDouble() - zero;
  }

  @Override
  public double getPercent() {
    return motor.get();
  }

  public double getRPS() {
    return motor.getRotorVelocity().getValueAsDouble();
  }

  /**
   * @see #getStatorCurrent()
   * @see #getTorqueCurrent()
   */
  @Override
  public double getSupplyCurrent() {
    return motor.getSupplyCurrent().getValueAsDouble();
  }

  /**
   * @return The current running through the motor, regardless of direction
   * @see #getSupplyCurrent()
   * @see #getTorqueCurrent()
   */
  public double getStatorCurrent() {
    return motor.getStatorCurrent().getValueAsDouble();
  }

  /**
   * @return The current running through the motor, inverted when running backward
   * @see #getSupplyCurrent()
   * @see #getStatorCurrent()
   */
  public double getTorqueCurrent() {
    return motor.getTorqueCurrent().getValueAsDouble();
  }
}
