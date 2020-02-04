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
public class IntakeInOutCommand extends Command{
    private boolean isIn = false;

    public IntakeInOutCommand(boolean isIn) {
        requires(Robot.intake);
        this.isIn = isIn;
    }

    @Override
    protected void execute() {
        if (isIn) {
            Robot.intake.in();
        } else {
            Robot.intake.out();
        }
    }

    @Override
    protected void end(){
        Robot.intake.stop();
    }

    @Override
    protected void interrupted(){
        end();
    }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }


}
