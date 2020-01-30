/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.subsystems.PanelManipulator;

/**
 * Add your docs here.
 */
public class RotatePanelNTimesBySensorCommand extends Command {
    private PanelManipulator panelManipulator;
    private Color startingColor;
    private int timesSpun = 0;
    private boolean started = false;
    private boolean hasMoved = false;
    private boolean done = false;
    private int nTimesToRotate = 6;
    
    @Override
    protected void initialize() {
        startingColor = getCurrentColorFunction();
        started = false;
        timesSpun = 0;
        done = false;
        hasMoved = false;
    }

    public RotatePanelNTimesBySensorCommand(int numberOfTimesToRotate) {
        requires(Robot.panelManipulator);
        panelManipulator = Robot.panelManipulator;
        SmartDashboard.putNumber("Number Of Times Rotated", timesSpun);
        SmartDashboard.putBoolean("Done????", done);
        if (nTimesToRotate > 0) {
            numberOfTimesToRotate = nTimesToRotate;
        }
    }

    @Override
    protected void execute() {    
        SmartDashboard.putNumber("Number Of Times Rotated", timesSpun);
        SmartDashboard.putBoolean("Done????", done); 
        Color currentColor = getCurrentColorFunction();
        if (!started) {
            startingColor = getCurrentColorFunction();
            timesSpun = 0;
            started = true;
        }
        SmartDashboard.putString("Current Color", currentColor.toString());
        if (hasMoved == false && currentColor == startingColor) {
            panelManipulator.rotate();
        }
        if (currentColor != startingColor) {
            hasMoved = true;
            panelManipulator.rotate();
        }
        if (currentColor == startingColor && hasMoved == true) {    
            timesSpun++;          
            SmartDashboard.putNumber("Number Of Times Rotated", timesSpun);
            hasMoved = false;
            panelManipulator.rotate();     
        }
        if (currentColor != startingColor && timesSpun == nTimesToRotate) {
            done = true;
        }
    }
    
    private Color getCurrentColorFunction() {
        final Color currentColor = panelManipulator.getMatchedColor();        
        return currentColor;
    }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return done;
    }
    //get current color
    //rotate the wheel
    //stop rotating after 3 times
}
