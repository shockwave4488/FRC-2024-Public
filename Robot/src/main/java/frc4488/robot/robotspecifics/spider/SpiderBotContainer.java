package frc4488.robot.robotspecifics.spider;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc4488.lib.BaseRobotContainer;
import frc4488.lib.logging.LogManager;
import frc4488.lib.operator.Controller;
import frc4488.lib.operator.DynamicController;
import frc4488.lib.preferences.PreferencesParser;
import frc4488.robot.Robot;
import frc4488.robot.constants.Constants.OIConstants;
import frc4488.robot.subsystems.SmartPCM;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SpiderBotContainer extends BaseRobotContainer {
  private final SmartPCM smartPCM;

  @SuppressWarnings("unused")
  private final Solenoid extensionSolenoid;

  @SuppressWarnings("unused")
  private final Solenoid retractionSolenoid;

  @SuppressWarnings("unused")
  private final Controller driverJoystick;

  /**
   * The robot container for our spider bot, this is where all classes relevant to this robot are
   * created and where its default command(s) are set
   */
  public SpiderBotContainer(PreferencesParser prefs, LogManager logger) {
    super(prefs, logger);

    smartPCM = new SmartPCM(prefs.getInt("PCM_ID"));
    driverJoystick = DynamicController.getXboxOrPlaystation(OIConstants.DRIVER_CONTROLLER_PORT);

    smartPCM.setDefaultCommand(
        new StartEndCommand(() -> smartPCM.startCompressor(), () -> {}, smartPCM)
            .withName("CompressorCommand"));

    extensionSolenoid =
        new Solenoid(
            prefs.tryGetValue(prefs::getInt, "PCM_ID", 0),
            PneumaticsModuleType.CTREPCM,
            prefs.tryGetValue(prefs::getInt, "ClimberSolenoidExtendID", 1));
    retractionSolenoid =
        new Solenoid(
            prefs.tryGetValue(prefs::getInt, "PCM_ID", 0),
            PneumaticsModuleType.CTREPCM,
            prefs.tryGetValue(prefs::getInt, "ClimberSolenoidRetractID", 1));

    addSubsystems();
    configureButtonBindings();
  }

  protected void addSubsystems() {
    subsystems.add(smartPCM);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
