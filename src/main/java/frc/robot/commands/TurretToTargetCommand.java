/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavigationBoard;

/**
 * Add your docs here.
 */
public class TurretToTargetCommand extends Command {
    private NavigationBoard navBoard;
    private double currentDrivetrainAngle = navBoard.getAngle();
    private double currentTurretPos = Robot.turret.getPosition();
    private static double offsetDrivetrain = 0;
    private static double newTurretPosition = 0;
    private static double previousYaw = 0;

    public TurretToTargetCommand() {
    }

    @Override
    protected void execute() {
        currentDrivetrainAngle = navBoard.getAngle();
        currentTurretPos = Robot.turret.getPosition();
        if (currentDrivetrainAngle > currentTurretPos && currentDrivetrainAngle > offsetDrivetrain) {
            newTurretPosition = Robot.turret.setPosition(currentTurretPos - currentDrivetrainAngle);
            currentTurretPos = newTurretPosition;
        } else if (currentDrivetrainAngle < currentTurretPos && currentDrivetrainAngle < offsetDrivetrain) {
            newTurretPosition = Robot.turret.setPosition(currentTurretPos - currentDrivetrainAngle);
            currentTurretPos = newTurretPosition;
        }
        // TODO Auto-generated method stub
    }

    /**
     * If the drivetrain orientation(degrees) is greater than the turret
     * position(angle), then it will find how much degrees required to move the
     * turret back to orientaion of 0 so it'll face the target at all times.
     */
    public static double greaterDrivetrain(double yaw, double turretAngle) {
        if (yaw > turretAngle) {
            newTurretPosition = turretAngle - yaw;
            turretAngle = newTurretPosition;
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
            newTurretPosition = turretAngle - yaw;
            turretAngle = newTurretPosition;
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
            newTurretPosition = turretAngle - previousYaw;
            turretAngle = newTurretPosition;
        } else {
            if (yaw == 0) {
                previousYaw = yaw;
            } else {
                previousYaw = yaw;
                yaw = 0;
            }
            newTurretPosition = turretAngle - previousYaw;
            turretAngle = newTurretPosition;
        }
        return turretAngle;
    }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
}
