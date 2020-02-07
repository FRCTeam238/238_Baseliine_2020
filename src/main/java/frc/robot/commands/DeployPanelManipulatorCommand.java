/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.PanelManipulator;

/**
 * Add your docs here.
 */
public class DeployPanelManipulatorCommand extends Command{
    private boolean isExtended = false;
    PanelManipulator panelManipulator;

    @Override
    protected void execute() {
        if (isExtended) {
            panelManipulator.extend();
        } else {
            panelManipulator.retract();
        }
    }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    
}
