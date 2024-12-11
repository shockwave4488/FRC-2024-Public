package frc4488.robot.commands.eruption.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc4488.robot.subsystems.eruption.Climber;
import java.util.function.DoubleSupplier;

public class ClimbMotionTest extends Command {
  private final Climber climber;
  private final DoubleSupplier power;
  private static final int CLIMB_TICK_RATE = 500000;
  private static final double INPUT_THRESHOLD = 0.75;
  private int desiredTicks;

  public ClimbMotionTest(Climber climber, DoubleSupplier power) {
    this.climber = climber;
    this.power = power;
  }

  @Override
  public void initialize() {
    desiredTicks = climber.getClimberPosition();
  }

  @Override
  public void execute() {
    double doublePower = power.getAsDouble();
    if (doublePower > INPUT_THRESHOLD) {
      desiredTicks += CLIMB_TICK_RATE;
    } else if (doublePower < -INPUT_THRESHOLD) {
      desiredTicks -= CLIMB_TICK_RATE;
    }
    climber.setClimberPosition(desiredTicks);
  }
}
