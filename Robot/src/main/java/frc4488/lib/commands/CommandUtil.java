package frc4488.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc4488.lib.misc.Util;
import frc4488.lib.wpiextensions.ShockwaveSubsystemBase;
import java.lang.reflect.Field;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Predicate;

public class CommandUtil {
  private CommandUtil() {}

  /**
   * Add the child requirements of the specified {@code parentRequirement} subsystem to the {@code
   * command}.
   */
  public static <T extends Command> T withChildRequirementsOf(
      T command, ShockwaveSubsystemBase... parentRequirements) {
    command.addRequirements(parentRequirements);
    for (ShockwaveSubsystemBase parentRequirement : parentRequirements) {
      command.addRequirements(parentRequirement.childRequirements.toArray(Subsystem[]::new));
    }
    return command;
  }

  /** {@link Command#until(BooleanSupplier)} that preserves the original command name. */
  public static ParallelRaceGroup endWhen(Command command, BooleanSupplier condition) {
    ParallelRaceGroup untilCommand = command.until(condition);
    untilCommand.setName(command.getName());
    return untilCommand;
  }

  /** {@link Command#withTimeout(double)} that preserves the original command name. */
  public static ParallelRaceGroup endAfter(Command command, double seconds) {
    ParallelRaceGroup timeoutCommand = command.withTimeout(seconds);
    timeoutCommand.setName(command.getName());
    return timeoutCommand;
  }

  /**
   * Constructs a command that executes the given action when it initializes, and doesn't finish
   * until interrupted.
   */
  public static StartEndCommand indefiniteInstantCommand(
      Runnable onInit, Subsystem... requirements) {
    return new StartEndCommand(onInit, () -> {}, requirements);
  }

  public static <T extends Command> T withName(T command, String name) {
    return Util.returnAfterModifying(command, cmd -> cmd.setName(name));
  }

  /** Will revert uninterruptible commands */
  public static void revertToDefault(ShockwaveSubsystemBase subsystem) {
    if (subsystem.childRequirements.isEmpty()) {
      Command command = CommandScheduler.getInstance().requiring(subsystem);
      if (command != null) {
        command.cancel();
      }
    } else {
      for (Subsystem child : subsystem.childRequirements) {
        Command command = CommandScheduler.getInstance().requiring(child);
        if (command != null) {
          command.cancel();
        }
      }
    }
  }

  @SuppressWarnings("unchecked")
  public static Set<Command> getRunningCommands() {
    try {
      Field field = CommandScheduler.class.getDeclaredField("m_scheduledCommands");
      field.setAccessible(true);
      return Collections.unmodifiableSet((Set<Command>) field.get(CommandScheduler.getInstance()));
    } catch (Exception e) {
      throw new RuntimeException("Error getting running commands", e);
    }
  }

  /** Will stop any command, including ones labeled with cancel incoming */
  public static boolean forceStop(Predicate<Command> shouldStop) {
    boolean anyStopped = false;
    for (Command command : new HashSet<>(getRunningCommands())) {
      if (shouldStop.test(command)) {
        command.cancel();
        anyStopped = true;
      }
    }
    return anyStopped;
  }

  public static boolean isRequiring(Command command, ShockwaveSubsystemBase subsystem) {
    if (subsystem.childRequirements.isEmpty()) {
      return command.hasRequirement(subsystem);
    }
    return subsystem.childRequirements.stream().anyMatch(child -> command.hasRequirement(child));
  }
}
