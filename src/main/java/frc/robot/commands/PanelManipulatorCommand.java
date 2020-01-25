/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.PanelManipulator;

/**
 * Add your docs here.
 */
public class PanelManipulatorCommand extends Command {

    public PanelManipulator panelManipulator;
    String gameData;

    public void getColour() {
        final Color currentColor = panelManipulator.getColor();
        final String desiredColor = SmartDashboard.getString("Assigned Color", "Green");
        final Color desiredColourObject = panelManipulator.toColor(desiredColor);
        PanelManipulator.colorSensing(currentColor, desiredColourObject);
    }

    public static Color getColorFromDriverStaion(String gameData) {
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
            case 'B':
                SmartDashboard.putString("Assigned Color", "B");
                return Color.kBlue;
            case 'G':
                SmartDashboard.putString("Assigned Color", "G");
                return Color.kGreen;
            case 'R':
                SmartDashboard.putString("Assigned Color", "R");
                return Color.kRed;
            case 'Y':
                SmartDashboard.putString("Assigned Color", "Y");
                return Color.kYellow;
            default:
                SmartDashboard.putString("Assigned Color", "Nothing");
                return Color.kBlack;
            }
        } else {
            SmartDashboard.putString("Assigned Color", "Nothing");
            return Color.kBlack;
        }
    }

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

    // get the color
    // get the desired color
    // get the distance
    // set position from subsystem
}
