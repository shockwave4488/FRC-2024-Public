package frc4488.lib.operator;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PlaystationController implements Controller {

  private final CommandPS4Controller controller;

  public PlaystationController(int port) {
    controller = new CommandPS4Controller(port);
  }

  @Override
  public double getRightX() {
    return controller.getRightX();
  }

  @Override
  public double getRightY() {
    return controller.getRightY();
  }

  @Override
  public double getLeftX() {
    return controller.getLeftX();
  }

  @Override
  public double getLeftY() {
    return controller.getLeftY();
  }

  @Override
  public Trigger rightBumper() {
    return controller.R1();
  }

  @Override
  public Trigger leftBumper() {
    return controller.L1();
  }

  @Override
  public double getRightTriggerAxis() {
    return controller.getR2Axis();
  }

  @Override
  public double getLeftTriggerAxis() {
    return controller.getL2Axis();
  }

  @Override
  public Trigger rightTrigger() {
    return controller.R2();
  }

  @Override
  public Trigger leftTrigger() {
    return controller.L2();
  }

  @Override
  public Trigger rightTrigger(double threshold) {
    return new BooleanEvent(
            CommandScheduler.getInstance().getDefaultButtonLoop(),
            () -> getRightTriggerAxis() > threshold)
        .castTo(Trigger::new);
  }

  @Override
  public Trigger leftTrigger(double threshold) {
    return new BooleanEvent(
            CommandScheduler.getInstance().getDefaultButtonLoop(),
            () -> getLeftTriggerAxis() > threshold)
        .castTo(Trigger::new);
  }

  @Override
  public Trigger a() {
    return controller.cross();
  }

  @Override
  public Trigger b() {
    return controller.circle();
  }

  @Override
  public Trigger x() {
    return controller.square();
  }

  @Override
  public Trigger y() {
    return controller.triangle();
  }

  @Override
  public Trigger start() {
    return controller.options();
  }

  @Override
  public Trigger back() {
    return controller.share();
  }

  @Override
  public Trigger povUp() {
    return controller.povUp();
  }

  @Override
  public Trigger povDown() {
    return controller.povDown();
  }

  @Override
  public Trigger povRight() {
    return controller.povRight();
  }

  @Override
  public Trigger povLeft() {
    return controller.povLeft();
  }

  @Override
  public Trigger pov(int angle) {
    return controller.pov(angle);
  }

  @Override
  public void rumble(double strength) {
    controller.getHID().setRumble(RumbleType.kBothRumble, strength);
  }
}
