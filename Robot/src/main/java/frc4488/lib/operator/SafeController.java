package frc4488.lib.operator;

import frc4488.robot.DemoLevel;

public class SafeController extends DynamicController {

  private static final double SAFE_TRANSLATION_SCALE = 0.5;
  private static final double SAFE_ROTATION_SCALE = 1;

  /** The speed is significantly reduced for new drivers */
  public static Controller forDemoLevel(DemoLevel level, Controller controller) {
    return switch (level) {
      case NONE, PRESENTATION -> controller;
      case NEW_DRIVER -> new SafeController(
          controller, SAFE_TRANSLATION_SCALE, SAFE_ROTATION_SCALE);
    };
  }

  private final double leftJoystickScaling;
  private final double rightJoystickScaling;

  public SafeController(
      Controller controller, double leftJoystickScaling, double rightJoystickScaling) {
    super(controller);
    this.leftJoystickScaling = leftJoystickScaling;
    this.rightJoystickScaling = rightJoystickScaling;
  }

  @Override
  public double getLeftX() {
    return super.getLeftX() * leftJoystickScaling;
  }

  @Override
  public double getLeftY() {
    return super.getLeftY() * leftJoystickScaling;
  }

  @Override
  public double getRightX() {
    return super.getRightX() * rightJoystickScaling;
  }

  @Override
  public double getRightY() {
    return super.getRightY() * rightJoystickScaling;
  }
}
