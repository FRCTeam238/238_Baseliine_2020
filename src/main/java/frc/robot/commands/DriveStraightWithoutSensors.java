/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

@AutonomousModeAnnotation(parameterNames = { "DriveSpeed", "DriveTime" })
public class DriveStraightWithoutSensors extends Command implements IAutonomousCommand {
  private Drivetrain driveTrain = Robot.drivetrain;
  private double driveTime = 0;
  private double driveSpeed = 0;
  private boolean isAutonomousMode;
  private boolean isDone = false;
  private double startTime = 0;

  public DriveStraightWithoutSensors() {
    requires(driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    driveTrain.drive(driveSpeed, driveSpeed);
    if((Timer.getFPGATimestamp() - startTime) >= driveTime){
      isDone = true;
      driveTrain.stop();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isDone;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  @Override
  public boolean getIsAutonomousMode() {
    // TODO Auto-generated method stub
    return isAutonomousMode;
  }

  @Override
  public void setIsAutonomousMode(boolean isAutonomousMode) {
    this.isAutonomousMode = isAutonomousMode;

  }

  @Override
  public void setParameters(List<String> parameters) {
    // TODO Auto-generated method stub
    driveSpeed = Double.parseDouble(parameters.get(0));
    driveTime = Double.parseDouble(parameters.get(1));
  }
}
