package frc4488.robot.commands.eruption.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc4488.lib.sensors.gyro.navx.NavX;
import frc4488.robot.commands.drive.LockedSwerveDrive;
import frc4488.robot.subsystems.drive.SwerveDrive;
import frc4488.robot.subsystems.eruption.Climber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveAndClimb extends SequentialCommandGroup {
  private ClimbMotionTest climbMotionTest;
  private ClimberLiftToHeight climberLiftToHeight;

  public DriveAndClimb(
      SwerveDrive swerve,
      Climber climber,
      NavX gyro,
      int desiredFirstHeight,
      DoubleSupplier power,
      BooleanSupplier doneDriving,
      BooleanSupplier abort) {
    climberLiftToHeight = new ClimberLiftToHeight(climber, desiredFirstHeight, abort);
    climbMotionTest = new ClimbMotionTest(climber, power);

    addCommands(
        climberLiftToHeight,
        new WaitUntilCommand(doneDriving),
        climbMotionTest.alongWith(new LockedSwerveDrive(swerve).asProxy()));
  }
}
