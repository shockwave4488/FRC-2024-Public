package frc4488.lib.sensors;

import edu.wpi.first.wpilibj.Counter;
import frc4488.lib.sensors.absoluteencoder.AbsoluteEncoder;
import frc4488.robot.constants.Constants;

/**
 * Note that foundation of this class was obtained from Team 3128 Aluminum Narwhals's GitHub
 * repository at the following link:
 * https://github.com/Team3128/3128-robot-2021/blob/main/src/main/java/org/team3128/common/hardware/encoder/both/CTREMagneticEncoder.java
 *
 * <p>Driver for a Magnetic Encoder using DIO ports on the roborio.
 *
 * <p>When instantiated, it sets the quadrature distance from the absolute angle. So, between 0 and
 * 1 rotations. When reset, the distance goes to zero.
 *
 * <p>Internally, it uses a Counter to measure the PWM.
 *
 * @author Narwhal, modified by Shockwave
 */
public class MagneticEncoder implements AbsoluteEncoder {

  private Counter pwmCounter;
  private double offset = 0;
  private double maximumPulse;
  private boolean inverted;

  /**
   * @param pwmPort DIO port connected to pin 9 on the encoder, the PWM pin
   * @param offset The offest you want to apply to the encoder, put 0 if you don't know or don't
   *     want one
   * @param inverted whether or not the encoder is inverted
   * @param maximumPulse the maximum pulse of the encoder
   */
  public MagneticEncoder(
      int pwmPort,
      double pulsesPerRevolution,
      double offset,
      boolean inverted,
      double maximumPulse) {
    this.offset = offset;
    pwmCounter = new Counter(pwmPort);
    pwmCounter.setSemiPeriodMode(true);
    this.maximumPulse = maximumPulse;
    this.inverted = inverted;

    pwmCounter.setDistancePerPulse(1);

    // wait for the pwm signal to be counted
    try {
      Thread.sleep(5);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  public double getAngle() {
    // from 1 to maximumPulse us
    // maximumPulse = 4095 for CTRE magnetic encoders and 1023 for REV magnetic encoders
    double angle = ((pwmCounter.getPeriod() - 1e-6) / (maximumPulse / 1e6)) * Constants.TAU;
    if (inverted) {
      angle = angle * -1;
    }
    return angle;
  }

  @Override
  public double getOffset() {
    return this.offset;
  }

  @Override
  public void setOffset(double offset) {
    this.offset = offset;
  }
}
