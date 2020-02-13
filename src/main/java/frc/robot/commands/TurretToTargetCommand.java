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
public class TurretToTargetCommand extends Command{
    private NavigationBoard navBoard;
    private TurnByNavigationBoard turnByNavigationBoard;

    private double currentDrivetrainAngle = navBoard.getAngle();
    private double currentTurretPos = Robot.turret.getPosition();
    private double offsetDrivetrain = 0;
    private double offsetTurret = 0;
    private double newTurretPosition = 0;

    public TurretToTargetCommand() {
    }

    @Override
    protected void execute() {
        currentDrivetrainAngle = navBoard.getAngle(); 
        currentTurretPos = Robot.turret.getPosition();
        if(currentDrivetrainAngle > currentTurretPos && currentDrivetrainAngle > offsetDrivetrain) {
            newTurretPosition = Robot.turret.setPosition(currentTurretPos - currentDrivetrainAngle);
            currentTurretPos = newTurretPosition;
        } else if(currentDrivetrainAngle < currentTurretPos && currentDrivetrainAngle < offsetDrivetrain) {
            newTurretPosition = Robot.turret.setPosition(currentTurretPos - currentDrivetrainAngle);
            currentTurretPos = newTurretPosition; 
        }
        // TODO Auto-generated method stub
    }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
}
