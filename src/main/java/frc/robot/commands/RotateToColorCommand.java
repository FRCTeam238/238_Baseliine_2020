/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Locale;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.PanelManipulator;

/**
 * Add your docs here.
 */
public class RotateToColorCommand extends Command {

    private PanelManipulator panelManipulator;

    public double getPositionForColor(Color desiredColor) {
        final Color currentColor = panelManipulator.getColor();
        double distance = PanelManipulator.getDistanceToColor(currentColor, desiredColor);
        return distance;
    }

    public static Color getColorFromDriverStaion(String gameData) {
        Color toReturn = Color.kBlack;
        if (gameData != null && gameData.length() > 0) {
            switch (gameData.toLowerCase(Locale.ROOT).charAt(0)) {
            case 'b':
                toReturn = Color.kBlue;
            case 'g':
                toReturn = Color.kGreen;
            case 'r':
                toReturn = Color.kRed;
            case 'y':
                toReturn = Color.kYellow;
            }
        }
        String toStringReturn = toReturn.toString();
        SmartDashboard.putString("Assigned Color", toStringReturn);
        return toReturn;
    }

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return true;
    }
    
    
    @Override
    protected void execute() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();

        Color colorFromStation = getColorFromDriverStaion(gameData); // get the desired color
        double positionToGoTo =  getPositionForColor(colorFromStation); // get the color // get the distance
        
        //TODO: set output to proper number
        panelManipulator.setPosition(positionToGoTo); // set position from subsystem
    }
}
