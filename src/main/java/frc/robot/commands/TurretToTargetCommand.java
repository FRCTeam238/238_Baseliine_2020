/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.core238.Logger;
import frc.robot.Robot;
import frc.robot.subsystems.NavigationBoard;

/**
 * Add your docs here.
 */
public class TurretToTargetCommand extends Command {
    private NavigationBoard navBoard;
    private double currentDrivetrainAngle = navBoard.getAbsoluteYaw();
    private double currentTurretPos = Robot.turret.getPosition();
    private static double desiredTurretMovement = 0;
    private static double previousYaw = 0;
    private static double delta = 90;

    private static double stopperOne = 105; //TODO
    private static double stopperTwo = 120;

    public TurretToTargetCommand() {
        requires(Robot.turret);
    }

    @Override
    protected void execute() {
        currentDrivetrainAngle = navBoard.getAbsoluteYaw();
        //calculaing how many degress we NEED to turn
        desiredTurretMovement = -currentTurretPos - currentDrivetrainAngle - delta;     
        
        double distanceToStopperOne = stopperOne - currentTurretPos;
        double distanceToStopperTwo = -(360-stopperTwo) - currentTurretPos;

        //chooses the most optimal route
        if (desiredTurretMovement <= 180 && desiredTurretMovement >= -180) {
            desiredTurretMovement = desiredTurretMovement;
        } else if (desiredTurretMovement < -180) {
            desiredTurretMovement += 360;
        } else if (desiredTurretMovement > 180) {
            desiredTurretMovement -= 360;
        }

        //checks for not passing the deadzone
        if (desiredTurretMovement > distanceToStopperOne) {
            desiredTurretMovement = distanceToStopperOne;            
        } 
        if (desiredTurretMovement < distanceToStopperTwo) {
            desiredTurretMovement = distanceToStopperTwo;
        }
        double desiredTurretPosition = currentTurretPos + desiredTurretMovement;
        //Robot.turret.setPosition(currentTurretPos + desiredTurretMovement);
        SmartDashboard.putNumber("Amount of Degress", desiredTurretPosition);
    } 

    /**
     * If the drivetrain orientation(degrees) is greater than the turret
     * position(angle), then it will find how much degrees required to move the
     * turret back to orientaion of 0 so it'll face the target at all times.
     */
    public static double greaterDrivetrain(double yaw, double turretAngle) {
        if (yaw > turretAngle) {
            desiredTurretMovement = turretAngle - yaw + delta;
            turretAngle = desiredTurretMovement;
        }
        return turretAngle;
    }

    /**
     * If the drivetrain orientation(degrees) is lesser than the turret
     * position(angle), then it will find how much degrees required to move the
     * turret back to orientaion of 0 so it'll face the target at all times.
     */
    public static double lesserDrivetrain(double yaw, double turretAngle) {
        if (yaw < turretAngle) {
            desiredTurretMovement = turretAngle - yaw;
            turretAngle = desiredTurretMovement;
        }
        return turretAngle;
    }

    /**
     * If the drivetrain orientation(degrees) is lesser or greater than the turret
     * position(angle), then it will find how much degrees required to move the
     * turret back to orientaion of 0 so it'll face the target at all times, but
     * this method will find the target even if the robot's yaw sets itself to 0
     * anytime.
     */
    public static double zeroDrivetrain(double yaw, double turretAngle) {
        if (yaw > turretAngle) { // 90,0
            if (yaw == 0) {
                previousYaw = yaw;
            } else {
                previousYaw = yaw;
                yaw = 0;
            }
            desiredTurretMovement = turretAngle - previousYaw;
            turretAngle = desiredTurretMovement;
        } else {
            if (yaw == 0) {
                previousYaw = yaw;
            } else {
                previousYaw = yaw;
                yaw = 0;
            }
            desiredTurretMovement = turretAngle - previousYaw;
            turretAngle = desiredTurretMovement;
        }
        return turretAngle;
    }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
}
