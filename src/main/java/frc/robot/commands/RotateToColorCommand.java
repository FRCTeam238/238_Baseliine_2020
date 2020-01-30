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

    private static PanelManipulator panelManipulator;
    private boolean done = false;

    @Override
    protected void initialize() {
        done = false;
    }

    public boolean getPositionForColor(Color desiredColor) {
        final Color currentColor = panelManipulator.getColor();
        if (currentColor == desiredColor) {
            panelManipulator.rotate();
        }else{
            panelManipulator.stop();
            done = true;
        }
    return done;
    }

    public static Color getColorFromDriverStaion(String gameData) {
        Color toReturn = Color.kBlack;
        if (gameData != null && gameData.length() > 0) {
            switch (gameData.toLowerCase(Locale.ROOT).charAt(0)) {
            case 'b':
                toReturn = Color.kRed;
            case 'g':
                toReturn = Color.kYellow;
            case 'r':
                toReturn = Color.kBlue;
            case 'y':
                toReturn = Color.kGreen;
            }
        }



        String toStringReturn = toReturn.toString();
        SmartDashboard.putString("Assigned Color", toStringReturn);
        return toReturn;
    }

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return done;
    }
    
    
    @Override
    protected void execute() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();

        Color colorFromStation = getColorFromDriverStaion(gameData); // get the desired color
        boolean positionToGoTo =  getPositionForColor(colorFromStation); // get the color // get the distance
        SmartDashboard.putBoolean("Rotated to Color ??????", positionToGoTo);
        //set position to subsysem
    }
}
