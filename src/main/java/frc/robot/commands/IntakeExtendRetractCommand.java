/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class IntakeExtendRetractCommand extends Command {
    private boolean extendIsTrue = false;

    public IntakeExtendRetractCommand(boolean extend) {
        requires(Robot.intake);
        extendIsTrue = extend;
    }

    @Override
    protected void execute() {
        if (extendIsTrue) {
            Robot.intake.extendIntake();
        } 
        else {
            Robot.intake.retractIntake();            
        }
    }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
}
