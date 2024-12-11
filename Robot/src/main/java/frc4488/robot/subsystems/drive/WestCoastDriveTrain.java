package frc4488.robot.subsystems.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc4488.lib.logging.LogManager;
import frc4488.robot.constants.Constants.DriveTrainConstants;

public class WestCoastDriveTrain extends WestCoastDriveBase {

  private Talon leftFront;
  private Talon rightFront;
  private Talon leftBack;
  private Talon rightBack;

  private DifferentialDrive drive;

  public WestCoastDriveTrain() {
    leftFront = new Talon(DriveTrainConstants.LEFT_FRONT_PORT);
    leftFront.setInverted(false);
    rightFront = new Talon(DriveTrainConstants.RIGHT_FRONT_PORT);
    rightFront.setInverted(false);
    leftBack = new Talon(DriveTrainConstants.LEFT_BACK_PORT);
    leftBack.setInverted(false);
    rightBack = new Talon(DriveTrainConstants.RIGHT_BACK_PORT);
    rightBack.setInverted(false);

    leftFront.addFollower(leftBack);
    rightFront.addFollower(rightBack);
    drive = new DifferentialDrive(leftFront, rightFront);
  }

  @Override
  public void driveWithJoysticks(double forward, double rot) {
    if (isSStopped()) {
      return;
    }
    drive.arcadeDrive(forward, rot);
  }

  @Override
  public void onStart(boolean sStopped) {}

  @Override
  public void onStop(boolean sStopped) {}

  @Override
  public void zeroSensors() {}

  @Override
  public void updateSmartDashboard() {}

  @Override
  public void setUpTrackables(LogManager logger) {}

  @Override
  public void onSStop(boolean robotEnabled) {
    leftFront.set(0);
    rightFront.set(0);
  }

  @Override
  public void onSRestart(boolean robotEnabled) {}
}
