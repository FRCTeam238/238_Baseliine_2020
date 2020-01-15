/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Trig238;

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {

  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;

  double targetHeight;
  double cameraHeight;
  double heightDifference;

  public Vision(double targHeight, double camHeight){
    targetHeight = targHeight;
    cameraHeight = camHeight;
    initLimelight();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public boolean hasTarget(){
    return true;
  }

  /** @return yaw (horizontal angle) read by limelight */
  public double getYaw(){
    // This requires the limelight to be calibrated so that it reads zero while the barrel is lined up with the target
    // Do this from the web interface before competitions
    tx = table.getEntry("tx");
    double yaw = tx.getDouble(0.0);
    return yaw;
  }

  public void initLimelight(){
    table = NetworkTableInstance.getDefault().getTable("limelight");
    table.getEntry("camMode").setNumber(RobotMap.limelightSettings.cameraMode);
    table.getEntry("ledMode").setNumber(RobotMap.limelightSettings.ledsOff);
    
  }

  /** @return pitch (vertical angle) read by limelight */
  public double getPitch(){
    ty = table.getEntry("ty");
    double pitch = ty.getDouble(0.0);
    return pitch;
  }

  /** @return linear distance from target, in inches */
  public double getDistanceToTarget(){
    heightDifference = Math.abs(targetHeight - cameraHeight);
    double distance = Trig238.calculateDistance(heightDifference, getPitch());
    return distance;
  }
}
