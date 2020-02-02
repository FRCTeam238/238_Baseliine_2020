/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class IntakeInOutCommand extends Command{
    //private boolean isIn = false;
    private double axisSenstiviy = 0.5;
    private DoubleSupplier input;

    public IntakeInOutCommand(DoubleSupplier input) {
        requires(Robot.intake);
        this.input = input;
        //isIn = in;
    }

    @Override
    protected void execute() {
        double value = input.getAsDouble();
        //double value = XBoxController.xboxController.getY(Hand.kRight);
        if (value <= -axisSenstiviy) {
            Robot.intake.in();
        } else if (value >= axisSenstiviy) {
            Robot.intake.out();
        } else {
            Robot.intake.stop();
        }
    }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }


}
