package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Hanger2021;

/**
 * DeployClawSoleniodCommand
 */
public class DeployClawSolenoidCommand extends Command {
    Hanger2021 theHanger = Robot.hanger2021;

    public DeployClawSolenoidCommand() {
        requires(theHanger);        
    }
    
    @Override
    protected void execute() {
        theHanger.deployClawSolenoid();
    }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }

    
}