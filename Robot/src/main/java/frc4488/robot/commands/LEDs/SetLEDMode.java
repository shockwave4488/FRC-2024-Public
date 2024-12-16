package frc4488.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc4488.robot.subsystems.leds.LEDController;
import frc4488.robot.subsystems.leds.LEDMode;

public class SetLEDMode extends Command {
  private final LEDController ledController;
  private final LEDMode mode;

  public SetLEDMode(LEDController ledController, LEDMode mode) {
    this.ledController = ledController;
    this.mode = mode;
    addRequirements(ledController);
  }

  @Override
  public void initialize() {
    ledController.setMode(mode);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
