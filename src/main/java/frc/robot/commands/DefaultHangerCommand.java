package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Hanger2021;

/**
 * DefaultHangerCommand
 */
public class DefaultHangerCommand extends Command{
    Hanger2021 theHanger = Robot.hanger2021;

    public DefaultHangerCommand() {
        requires(theHanger);
    }


    @Override
    protected void execute() {
        theHanger.controlTalon(0);
        super.execute();
    }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    
}