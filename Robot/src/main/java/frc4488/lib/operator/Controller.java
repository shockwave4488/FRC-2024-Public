package frc4488.lib.operator;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controller {
  public double getRightX();

  public double getRightY();

  public double getLeftX();

  public double getLeftY();

  public Trigger rightBumper();

  public Trigger leftBumper();

  public double getRightTriggerAxis();

  public double getLeftTriggerAxis();

  public Trigger rightTrigger();

  public Trigger leftTrigger();

  public Trigger rightTrigger(double threshold);

  public Trigger leftTrigger(double threshold);

  public Trigger a();

  public Trigger b();

  public Trigger x();

  public Trigger y();

  public Trigger start();

  public Trigger back();

  public Trigger povUp();

  public Trigger povDown();

  public Trigger povRight();

  public Trigger povLeft();

  public Trigger pov(int angle);

  public void rumble(double strength);
}
