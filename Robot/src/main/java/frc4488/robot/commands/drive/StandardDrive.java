package frc4488.robot.commands.drive;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import frc4488.robot.subsystems.drive.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class StandardDrive extends Command {
  private final SwerveDrive swerve;
  private final Supplier<Pair<Double, Double>> driveValues;
  private final BooleanSupplier fieldRelativeSupplier;
  private boolean fieldRelative = false;

  public StandardDrive(
      SwerveDrive swerve,
      double speedMultiplier,
      Supplier<Pair<Double, Double>> driveValues,
      BooleanSupplier fieldRelativeSupplier) {
    this.swerve = swerve;
    this.driveValues =
        () -> {
          Pair<Double, Double> values = driveValues.get();
          return Pair.of(values.getFirst() * speedMultiplier, values.getSecond() * speedMultiplier);
        };
    this.fieldRelativeSupplier = fieldRelativeSupplier;
    addRequirements(swerve.driveRequirement);
  }

  @Override
  public void execute() {
    fieldRelative = fieldRelativeSupplier.getAsBoolean();
    /*
    The controller's y value effects the drive speed's x value (and vice versa) because the controller input is
    90 degrees off compared to the values SwerveDrive expects (particularly ChassisSpeeds)
    */
    Pair<Double, Double> driveSpeedValues = driveValues.get();
    swerve.setTranslationSpeeds(
        driveSpeedValues.getFirst(), driveSpeedValues.getSecond(), fieldRelative);
  }
}
