package frc4488.lib.operator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Arrays;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

public class DynamicController implements Controller {

  public static Controller getXboxOrPlaystation(int port) {
    XboxController xboxController = new XboxController(port);
    PlaystationController playstationController = new PlaystationController(port);
    DynamicController output = new DynamicController(playstationController, xboxController);
    new Command() {
      public void execute() {
        boolean xbox = DriverStation.getJoystickIsXbox(port);
        if (xbox == (output.getSelected() == xboxController)) {
          return;
        }
        if (xbox) {
          output.setSelected(xboxController);
        } else {
          output.setSelected(playstationController);
        }
      }
    }.withName("DynamicControllerUpdater [" + port + "]").ignoringDisable(true).schedule();
    return output;
  }

  private final Controller[] options;
  private Controller selected;

  public DynamicController(Controller... options) {
    this.options = options;
    this.selected = options[0];
  }

  public void setSelected(Controller selected) {
    this.selected = selected;
  }

  public Controller getSelected() {
    return selected;
  }

  @Override
  public double getRightX() {
    return selected.getRightX();
  }

  @Override
  public double getRightY() {
    return selected.getRightY();
  }

  @Override
  public double getLeftX() {
    return selected.getLeftX();
  }

  @Override
  public double getLeftY() {
    return selected.getLeftY();
  }

  @Override
  public Trigger rightBumper() {
    return getTrigger(Controller::rightBumper);
  }

  @Override
  public Trigger leftBumper() {
    return getTrigger(Controller::leftBumper);
  }

  @Override
  public double getRightTriggerAxis() {
    return selected.getRightTriggerAxis();
  }

  @Override
  public double getLeftTriggerAxis() {
    return selected.getLeftTriggerAxis();
  }

  @Override
  public Trigger rightTrigger() {
    return getTrigger(Controller::rightTrigger);
  }

  @Override
  public Trigger leftTrigger() {
    return getTrigger(Controller::leftTrigger);
  }

  @Override
  public Trigger rightTrigger(double threshold) {
    return getTrigger(controller -> controller.rightTrigger(threshold));
  }

  @Override
  public Trigger leftTrigger(double threshold) {
    return getTrigger(controller -> controller.leftTrigger(threshold));
  }

  @Override
  public Trigger a() {
    return getTrigger(Controller::a);
  }

  @Override
  public Trigger b() {
    return getTrigger(Controller::b);
  }

  @Override
  public Trigger x() {
    return getTrigger(Controller::x);
  }

  @Override
  public Trigger y() {
    return getTrigger(Controller::y);
  }

  @Override
  public Trigger start() {
    return getTrigger(Controller::start);
  }

  @Override
  public Trigger back() {
    return getTrigger(Controller::back);
  }

  @Override
  public Trigger povUp() {
    return getTrigger(Controller::povUp);
  }

  @Override
  public Trigger povDown() {
    return getTrigger(Controller::povDown);
  }

  @Override
  public Trigger povRight() {
    return getTrigger(Controller::povRight);
  }

  @Override
  public Trigger povLeft() {
    return getTrigger(Controller::povLeft);
  }

  @Override
  public Trigger pov(int angle) {
    return getTrigger(controller -> controller.pov(angle));
  }

  private Trigger getTrigger(Function<Controller, Trigger> btn) {
    Map<Controller, Trigger> triggers =
        Arrays.stream(options).collect(Collectors.toMap(controller -> controller, btn));
    return new Trigger(() -> triggers.get(selected).getAsBoolean());
  }

  @Override
  public void rumble(double strength) {
    selected.rumble(strength);
  }
}
