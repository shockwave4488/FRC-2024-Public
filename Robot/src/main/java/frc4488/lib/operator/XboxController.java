package frc4488.lib.operator;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxController implements Controller {

  private final CommandXboxController controller;

  public XboxController(int port) {
    controller = new CommandXboxController(port);
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
    return controller.rightBumper();
  }

  @Override
  public Trigger leftBumper() {
    return controller.leftBumper();
  }

  @Override
  public double getRightTriggerAxis() {
    return controller.getRightTriggerAxis();
  }

  @Override
  public double getLeftTriggerAxis() {
    return controller.getLeftTriggerAxis();
  }

  @Override
  public Trigger rightTrigger() {
    return controller.rightTrigger();
  }

  @Override
  public Trigger leftTrigger() {
    return controller.leftTrigger();
  }

  @Override
  public Trigger rightTrigger(double threshold) {
    return controller.rightTrigger(threshold);
  }

  @Override
  public Trigger leftTrigger(double threshold) {
    return controller.leftTrigger(threshold);
  }

  @Override
  public Trigger a() {
    return controller.a();
  }

  @Override
  public Trigger b() {
    return controller.b();
  }

  @Override
  public Trigger x() {
    return controller.x();
  }

  @Override
  public Trigger y() {
    return controller.y();
  }

  @Override
  public Trigger start() {
    return controller.start();
  }

  @Override
  public Trigger back() {
    return controller.back();
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
