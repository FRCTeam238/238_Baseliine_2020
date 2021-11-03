package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import frc.core238.Logger;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Hanger2021;

/**
 * HangerCommand
 */
public class HangerCommand extends Command {
    Hanger2021 theHanger = Robot.hanger2021;
    private GenericHID controller;
    private int axis;
    private double minAxis = RobotMap.ClimberDevices.DEADBAND_MINIMUM_AXIS; //DOUBLE CHECK AXIS
    DriverStation driverStation;

    boolean pullingRobotUp = false;

    public HangerCommand(GenericHID controller, int axis) {
        requires(theHanger);

        this.controller = controller;
        this.axis = axis;
        driverStation = DriverStation.getInstance();
    }

    @Override
    protected void initialize() {
    }

    protected void execute() {
        double matchTime = driverStation.getMatchTime(); 
        double speed = -1 * controller.getRawAxis(axis);
        if (matchTime <= 30) {
            // elevator up
            if ((speed > minAxis) && (theHanger.topSwitch.get() == true)) {
                theHanger.retractBrake();
                theHanger.controlTalon(speed);
                pullingRobotUp = false;
            }
            else if ((speed < -minAxis) && (theHanger.bottomSwitch.get() == true)) { // elevator down
                theHanger.retractBrake();
                theHanger.controlTalon(speed);
                pullingRobotUp = true;
            }
            else if ((pullingRobotUp == true)) { //default/neutral
                theHanger.deployBrake();
            } else {
                theHanger.controlTalon(0);
            }

        }
    }

    //CHECK IF NEEDED----
    //restrict movement if hanger is not in appropriate levels
    //deploy hanger when AT CERTAIN HEIGHT
    //-------------------

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    protected void end() {
        theHanger.deployBrake();
        super.end();
    }
}