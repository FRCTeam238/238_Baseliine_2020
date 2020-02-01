/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Turret;
import frc.robot.OI;

public class TurnTurretManually extends Command {

  Turret theTurret = Robot.turret;
  private Joystick operatorController;
  double leftOperatorJsValue;
  double turnVelocity = 0;
  double tuningValue = 0.1;

  public TurnTurretManually() {
    requires(theTurret);
    operatorController = Robot.oi.controller;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    turnVelocity = getWantedVelocity();
    theTurret.setTurnVelocity(turnVelocity);
  }

  protected double getWantedVelocity(){
    leftOperatorJsValue = operatorController.getX(Hand.kRight);
    double power = leftOperatorJsValue * ((tuningValue * leftOperatorJsValue * leftOperatorJsValue) + (1-tuningValue));
    return power;
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    theTurret.neutral();
  }
}
