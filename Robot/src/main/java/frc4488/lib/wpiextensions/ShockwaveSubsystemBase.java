package frc4488.lib.wpiextensions;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4488.lib.commands.CommandUtil;
import frc4488.lib.logging.LogManager;
import frc4488.robot.subsystems.SStoppable;
import java.util.ArrayList;
import java.util.List;

/** Insert comment on what exactly this class does */
public abstract class ShockwaveSubsystemBase extends SubsystemBase implements SStoppable {

  private class SStopCommand extends Command {
    public SStopCommand() {
      if (childRequirements.isEmpty()) {
        addRequirements(ShockwaveSubsystemBase.this);
      } else {
        addRequirements(childRequirements.toArray(Subsystem[]::new));
      }
    }

    @Override
    public String getName() {
      return "S-Stop for " + ShockwaveSubsystemBase.this.getClass().getName();
    }

    public boolean runsWhenDisabled() {
      return true;
    }

    public InterruptionBehavior getInterruptionBehavior() {
      return InterruptionBehavior.kCancelIncoming;
    }

    public boolean isFinished() {
      return !ShockwaveSubsystemBase.this.sStopped;
    }
  }

  public List<Subsystem> childRequirements = new ArrayList<>();
  private boolean sStopped;

  /**
   * What the subsystem should do upon starting up, replacement of onStart() from our old Loop class
   */
  public abstract void onStart(boolean sStopped);

  /**
   * What the subsystem should do upon shutting down, replacement of onStop() from our old Loop
   * class
   */
  public abstract void onStop(boolean sStopped);

  /** Where all sensors should be zeroed/reset */
  public abstract void zeroSensors();

  /** Where you should update all desired values to Smart Dashboard */
  public abstract void updateSmartDashboard();

  /** Where you should set up the trackables to be logged */
  public abstract void setUpTrackables(LogManager logger);

  public final void setSStopped(boolean sStopped) {
    if (this.sStopped == sStopped) {
      return;
    }
    this.sStopped = sStopped;
    if (sStopped) {
      // Remove any commands currently using the subsystem
      CommandUtil.forceStop(command -> CommandUtil.isRequiring(command, this));
      // Block commands from using the subsystem in the future by running an uninterruptable command
      new SStopCommand().schedule();
      onSStop(RobotState.isEnabled());
    } else {
      onSRestart(RobotState.isEnabled());
    }
  }

  public boolean isSStopped() {
    return sStopped;
  }

  /**
   * Makes a dummy subsystem that can act as a lock on part of this subsystem's functionality. If
   * you require the parent subsystem, the child requirements are <b>not</b> automatically added
   * too, so commands using those requirements will not be interrupted. If this is the desired
   * behavior, pass the command requiring the whole subsystem into {@link
   * frc4488.lib.commands.CommandUtil#withChildRequirements(edu.wpi.first.wpilibj2.command.Command)
   * withChildRequirements(Command)}.
   */
  protected SubsystemBase childRequirement() {
    SubsystemBase childRequirement = new SubsystemBase() {};
    childRequirements.add(childRequirement);
    return childRequirement;
  }
}
