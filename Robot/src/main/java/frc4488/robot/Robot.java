// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4488.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc4488.lib.IRobotContainer;
import frc4488.lib.commands.CommandLogger;
import frc4488.lib.controlsystems.SimManager;
import frc4488.lib.dashboard.DashboardInitializer;
import frc4488.lib.dashboard.DashboardServer;
import frc4488.lib.dashboard.packets.PacketProtocolFactory;
import frc4488.lib.freezedetector.FreezeDetectorThread;
import frc4488.lib.logging.Console;
import frc4488.lib.logging.LeveledSmartDashboard;
import frc4488.lib.logging.LogLevel;
import frc4488.lib.logging.LogManager;
import frc4488.lib.misc.MatchUtil;
import frc4488.lib.misc.MatchUtil.MatchPhase;
import frc4488.lib.misc.TimeUtil;
import frc4488.lib.preferences.PreferencesParser;
import java.io.IOException;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // There can only ever be 1 driver station
  // And servers can handle multiple connections anyway
  private static DashboardServer dashboard;

  private static void initDashboard(LogManager logger, IRobotContainer container) {
    try {
      PacketProtocolFactory.registerDefaultProtocols(logger);
      dashboard =
          new DashboardServer(
                  logger,
                  PacketProtocolFactory.create(PacketProtocolFactory.NETWORK_TABLE_PROTOCOL))
              .bindToSystem()
              .registerDefaultActions()
              .ready();
    } catch (IOException e) {
      logger.getMainLog().println(LogLevel.ERROR, "Error while starting dashboard", e);
      return;
    }

    if (container instanceof DashboardInitializer dashContainer) {
      dashContainer.onDashboardInit(dashboard);
      dashboard.getWebsite().sendWidgets();
    }
  }

  @SuppressFBWarnings("MS_EXPOSE_REP")
  public static Console getConsole() {
    if (dashboard == null) {
      return Console.VOID;
    }
    return dashboard;
  }

  private static final Queue<Runnable> mainThreadQueue = new ConcurrentLinkedQueue<>();

  public static void runOnMainThread(Runnable runnable) {
    mainThreadQueue.add(runnable);
  }

  private Command m_autonomousCommand;
  private final LogManager logger = new LogManager().withDefaultLoggers();
  private final PreferencesParser prefs = new PreferencesParser(logger);
  private final RobotSelector robotSelector = new RobotSelector(prefs, logger);
  private IRobotContainer m_robotContainer;
  private FreezeDetectorThread freezeDetector;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Enable printToConsole and FreezeDetectorThread.STACK for debugging
    freezeDetector =
        new FreezeDetectorThread(
            Thread.currentThread(),
            logger.getLogFile("FreezeDetector"),
            false,
            FreezeDetectorThread.PERIODIC);
    TimeUtil.init();

    String gitBranch = BuildConstants.GIT_BRANCH;
    int gitDirty = BuildConstants.DIRTY; // Comparing directly causes warning about dead code
    if (gitDirty != 0) {
      gitBranch += "*";
    }
    LeveledSmartDashboard.INFO.putString("GIT_SHA", BuildConstants.GIT_SHA);
    LeveledSmartDashboard.INFO.putString("GIT_BRANCH", gitBranch);
    logger
        .getMainLog()
        .println(LogLevel.INFO, "Git: " + gitBranch + " - " + BuildConstants.GIT_SHA);
    MatchUtil.findIfRealMatch(logger, isSimulation());

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = robotSelector.getRobot();
    m_robotContainer.runZeroSensors();
    m_robotContainer.runSetUpTrackables(logger);

    initDashboard(logger, m_robotContainer);

    CommandLogger.getInstance().setLoggerTrackable(logger);
    LiveWindow.enableTelemetry(CommandLogger.getInstance());

    // TODO: move port forwarding to inside robot containers as different robots have different
    // cameras

    // Cameras
    PortForwarder.add(1180, "limelight.local", 5800);
    PortForwarder.add(5800, "limelight.local", 5801);

    PortForwarder.add(1181, "limelight-right.local", 5800);
    PortForwarder.add(5801, "limelight-right.local", 5801);

    PortForwarder.add(1182, "limelight-left.local", 5800);
    PortForwarder.add(5802, "limelight-left.local", 5801);

    PortForwarder.add(1183, "photonvision.local", 1182);
    PortForwarder.add(5803, "photonvision.local", 5800);

    // LEDs
    PortForwarder.add(5805, "10.44.88.15", 80);

    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    freezeDetector.periodic();
    TimeUtil.checkTimeJump();

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    CommandLogger.getInstance().update();
    logger.update();
    m_robotContainer.runUpdateSmartDashboard();

    // If the queued Runnable adds another to the queue, this prevents a deadlock
    int queueSize = mainThreadQueue.size();
    for (int i = 0; i < queueSize; i++) {
      mainThreadQueue.remove().run();
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.runOnStop();

    logger.beginPhase(null);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // In practice matches, logger.init will only be initialized on enable, after an iteration of
    // robotPeriodic. The modeInit methods run before robotPeriodic, so logger.init will be null
    // here and beginPhase() won't be called. Instead, the Logger takes care of that for practice
    // matches.
    // In real matches, logger.init will initialize when the FMS is connected (before enabling).
    logger.beginPhase(MatchPhase.Autonomous);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_robotContainer.runZeroSensors();
    m_robotContainer.runOnStart();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    logger.getMainLog().println(LogLevel.HIGH, "Autonomous Command Called");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    logger.beginPhase(MatchPhase.Teleop);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.runOnStart();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    logger.beginPhase(MatchPhase.Test);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    SimManager.simulationInit();
  }

  @Override
  public void simulationPeriodic() {
    SimManager.simulationPeriodic();
  }
}
