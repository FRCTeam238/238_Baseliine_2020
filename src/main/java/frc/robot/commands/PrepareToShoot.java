/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.Trig238;
import frc.robot.subsystems.Shooter;


public class PrepareToShoot extends Command {

  private Shooter theShooter = Robot.shooter;

  private final double gravityAcceleration = 386.22;
  private double shootingAngle = 0.6;
  private double velocityBall;
  private double velocityWheel;
  private final double wheelRadius = 6;
  private final double ticksPerRotation = 4096;

  public void Shoot(Shooter theShooter) {
    this.theShooter = theShooter;
    requires(theShooter);

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
    double wantedSpeed = calculateSpeed(shootingAngle, gravityAcceleration, wheelRadius);
    theShooter.setSpeed(wantedSpeed);

  }
  //find speed to run at, in ticks per 100ms
  //tell shooter to run at that speed

//In Velocity mode, output value is in position change / 100ms.
  public double calculateSpeed(double shootingAngle, double gravityAcceleration, double wheelRadius){
    velocityBall = Trig238.calculateBallVelocity(FieldConstants.VisionConstants.getTargetheight(), gravityAcceleration,
    shootingAngle, Robot.vision.getDistanceToTarget());
    velocityWheel = Trig238.calculateSingleWheelShooterVelocity(velocityBall, wheelRadius, FieldConstants.GamePieces.getBallradius());
    double ticksPer100ms = (204.8 * velocityWheel) / (wheelRadius * Math.PI);
    return ticksPer100ms;
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
  }
}
