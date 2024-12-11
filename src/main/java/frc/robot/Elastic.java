package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.GameConstants;

public class Elastic {
    private SendableChooser<String> teamChooser;
    private SendableChooser<String> startPositionChooser;

    public Elastic() {
        teamChooser = new SendableChooser<>();
        teamChooser.setDefaultOption(GameConstants.teamColor, GameConstants.teamColor);
        teamChooser.addOption(GameConstants.kTeamColorRed, GameConstants.kTeamColorRed);
        teamChooser.addOption(GameConstants.kTeamColorBlue, GameConstants.kTeamColorBlue);

        startPositionChooser = new SendableChooser<>();
        startPositionChooser.setDefaultOption(GameConstants.startPosition, GameConstants.startPosition);
        startPositionChooser.addOption(GameConstants.kStartLeftSide, GameConstants.kStartLeftSide);
        startPositionChooser.addOption(GameConstants.kStartRightSide, GameConstants.kStartRightSide);
    }

    public void updateInterface() {
        
    }

    public String getSelectedTeamColor() {
        return teamChooser.getSelected();
    }

    public String getSelectedStartPosition() {
        return startPositionChooser.getSelected();
    }
}
