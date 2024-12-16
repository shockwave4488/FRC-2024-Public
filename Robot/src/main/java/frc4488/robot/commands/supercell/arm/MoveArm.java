package frc4488.robot.commands.supercell.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc4488.lib.commands.DoneCycleCommand;
import frc4488.lib.commands.LogCommand;
import frc4488.lib.controlsystems.DoneCycleMachine;
import frc4488.robot.subsystems.supercell.Arm;

public class MoveArm extends Command {
  public static Command createForAuto(Arm arm, double goal) {
    return LogCommand.endAfter(
        new DoneCycleCommand<>(new MoveArm(arm, goal), true)
            .withDoneCycles(
                DoneCycleMachine.supplierWithMinCycles(
                    () -> Math.abs(arm.getMeasurement() - goal) <= Math.toRadians(10), 10)),
        5);
  }

  private final Arm arm;
  private final double goal;

  public MoveArm(Arm arm, double goal) {
    this.arm = arm;
    this.goal = goal;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setArmPosition(goal);
  }
}
