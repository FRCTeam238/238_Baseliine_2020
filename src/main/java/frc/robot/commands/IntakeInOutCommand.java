/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.core238.Logger;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class IntakeInOutCommand extends Command{
    private double speed = 0;
    private boolean isDone = false;
    private GenericHID controller;
    private int axis;

    public IntakeInOutCommand(GenericHID controller, int axis) {
        requires(Robot.intake);
        this.speed = speed;
        this.controller = controller;
        this.axis = axis;
    }

    @Override
    protected void execute() {
        speed = controller.getRawAxis(axis);
        if (Math.abs(speed) < 0.2){
            Robot.intake.stop();
        } else {
            Robot.intake.in(speed);
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
        return isDone;
    }


}
