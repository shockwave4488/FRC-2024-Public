package frc4488.lib.wpiextensions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashSet;
import java.util.Set;
import java.util.TreeSet;
import java.util.WeakHashMap;
import java.util.function.Function;

/** Uses enabled states and their corresponding priorities to determine what should be executed */
public abstract class PriorityManager<T extends Enum<T>> {

  /**
   * Use to completely wrap a subsystem with a priority-based system. Avoid using the subsystem
   * directly.
   */
  public static class SubsystemPriorityManager<
          S extends Subsystem, T extends Enum<T> & Function<S, Command>>
      extends PriorityManager<T> {
    private final S subsystem;
    private final WeakHashMap<Command, T> commandToState;

    public SubsystemPriorityManager(S subsystem, Class<T> states) {
      super(states);
      this.subsystem = subsystem;
      this.commandToState = new WeakHashMap<>();
      CommandScheduler.getInstance().onCommandFinish(this::onCommandFinish);
      CommandScheduler.getInstance()
          .onCommandInterrupt(
              (cmd, interruptor) -> {
                if (interruptor.isEmpty()) {
                  onCommandFinish(cmd);
                }
              });
    }

    @Override
    protected void onStateChange(T newState) {
      Command command = newState.apply(subsystem);
      commandToState.put(command, newState);
      command.schedule();
    }

    private void onCommandFinish(Command command) {
      T state = commandToState.get(command);
      if (state != null) {
        disableState(state);
      }
    }
  }

  private final T defaultState;
  private final Set<T> tempStates;
  private final TreeSet<T> states;
  private T currentState;

  public PriorityManager(Class<T> states) {
    this.defaultState = states.getEnumConstants()[0];
    this.tempStates = new HashSet<>();
    this.states = new TreeSet<>();
    this.currentState = defaultState;
  }

  /**
   * Attempt to execute a state. If this is the highest priority state, it will be executed.
   * Otherwise, it will be executed once all higher priority states are disabled
   */
  public void enableState(T state) {
    states.add(state);
    updateCurrentState();
  }

  /**
   * Attempt to execute a state. If this is the highest priority state, it will be executed and
   * automatically disabled once a higher priority state is enabled. Otherwise, it will not be
   * executed (even if higher priority states are disabled)
   */
  public void enableTempState(T state) {
    tempStates.add(state);
    enableState(state);
  }

  /**
   * Stops attempting to execute a state. If this is the highest priority state, the next highest
   * priority state will start being executed
   */
  public void disableState(T state) {
    states.remove(state);
    updateCurrentState();
  }

  /** Calls either {@link #enableState(Enum)} or {@link #disableState(Enum)} */
  public void setState(T state, boolean enabled) {
    if (enabled) {
      enableState(state);
    } else {
      disableState(state);
    }
  }

  /**
   * @return A command that enables a state when initialized and disables it when interrupted
   */
  public Command toggleState(T state) {
    return Commands.startEnd(() -> enableState(state), () -> disableState(state));
  }

  private void updateCurrentState() {
    T newState = states.isEmpty() ? defaultState : states.last();
    if (currentState == newState) {
      return;
    }
    currentState = newState;
    tempStates.removeIf(
        state -> {
          if (state != currentState) {
            states.remove(state);
            return true;
          }
          return false;
        });
    onStateChange(newState);
  }

  /**
   * @return The highest priority state of the enabled states
   */
  public T getCurrentState() {
    return currentState;
  }

  /** Is NOT automatically called when constructed */
  protected abstract void onStateChange(T newState);
}
