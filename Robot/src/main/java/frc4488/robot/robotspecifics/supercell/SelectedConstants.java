package frc4488.robot.robotspecifics.supercell;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc4488.robot.autonomous.modes.supercell.AutonomousChooser.AutonomousMode;
import frc4488.robot.constants.Constants2023.DoubleSubstationSide;
import frc4488.robot.constants.Constants2023.GamePiece;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class SelectedConstants {
  public final Map<Integer, SendableChooser<GamePiece>> gamePieceChoosers;
  public final SendableChooser<AutonomousMode> autoModeChooser = new SendableChooser<>();
  public final SendableChooser<DoubleSubstationSide> substationSideChooser =
      new SendableChooser<>();
  public final SendableChooser<GamePiece> startPieceChooser = new SendableChooser<>();

  public SelectedConstants() {
    ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");

    Map<Integer, SendableChooser<GamePiece>> gamePieceChooserMap = new HashMap<>();
    for (int i = 1; i <= 4; i++) {
      SendableChooser<GamePiece> gamePieceChooser = new SendableChooser<>();
      gamePieceChooser.addOption("Cube", GamePiece.Cube);
      gamePieceChooser.addOption("Cone", GamePiece.Cone);
      competitionTab
          .add("Game piece " + String.valueOf(i), gamePieceChooser)
          .withSize(2, 1)
          .withPosition((i - 1) * 2, 0)
          .withWidget(BuiltInWidgets.kSplitButtonChooser);
      gamePieceChooserMap.put(i, gamePieceChooser);
    }
    gamePieceChoosers = Collections.unmodifiableMap(gamePieceChooserMap);

    for (AutonomousMode mode : AutonomousMode.values()) {
      autoModeChooser.addOption(mode.getNiceName(), mode);
    }
    competitionTab.add("Auto mode", autoModeChooser).withSize(2, 1).withPosition(0, 1);

    substationSideChooser.addOption("Left", DoubleSubstationSide.Left);
    substationSideChooser.addOption("Right", DoubleSubstationSide.Right);
    competitionTab
        .add("Substation side", substationSideChooser)
        .withSize(2, 1)
        .withPosition(2, 1)
        .withWidget(BuiltInWidgets.kSplitButtonChooser);

    startPieceChooser.addOption("Cube", GamePiece.Cube);
    startPieceChooser.addOption("Cone", GamePiece.Cone);
    competitionTab
        .add("Start game piece", startPieceChooser)
        .withSize(2, 1)
        .withPosition(4, 1)
        .withWidget(BuiltInWidgets.kSplitButtonChooser);
  }

  public String getAutoModeChooserKey() {
    return "/Shuffleboard/Competition/Auto mode/";
  }
}
