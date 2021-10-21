package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Hanger2021;

/**
 * HangerCommand
 */
public class HangerCommand extends Command {
    Hanger2021 theHanger = Robot.hanger2021;
    private GenericHID controller;
    private int axis;
    private double minAxis = 0.25; //THIS IS NOT ALLOWED; I, AARUSH, WILL CHANGE THIS OR ELSE I WILL BE
    //SUBJECT TO PUSH UPS
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
        matchTime = 25;
        if (matchTime <= 30) {
            // neutral
            if ((speed < minAxis) && (speed > -minAxis) && (pullingRobotUp == true)) { //do we need button or joystick? -Aarush 
                theHanger.deployBrake();
            }

            // elevator up
            if (speed > minAxis) {
                theHanger.retractBrake();
                theHanger.controlTalon(speed);
                // DOUBLE CHECK MECHANICAL VIEW OF THE HANGER MOVEMENT OF MOTOR
                theHanger.deployClawSolenoid();
                pullingRobotUp = false;
            }

            // elevator down
            if (speed < -minAxis) {
                theHanger.retractBrake();
                theHanger.controlTalon(speed);
                pullingRobotUp = true;
            }

        }
    }

    //CHECK IF NEEDED----
    //restrict movement if hanger is not in appropriate levels
    //deploy hanger when AT CERTAIN HEIGHT
    //-------------------

    // if time > 30 {
    //   brake fired
    // }
    // if time <= 30 {
    //   if stick pressed up {
    //     move elevtor up
    //     if first time moving elevtor {
    //       fire hook;
    //       retract brake; (pulling piston back)
    //     }
    //   }
    //   if stick pressed down {
    //     applying brake; (firing)
    //     move elevator down;
    //   }
    //   if no stick pressed {
    //     brake fired automatically
    //   }
    // }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
}