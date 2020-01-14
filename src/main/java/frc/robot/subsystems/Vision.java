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

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("tx");

  public Vision(){
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

  /** @return Yaw (horizontal angle) read by limelight */
  public double getYaw(){
    //this is where we do the math... need to takeinto account camera position, etc...
    // can be used by auto commands or by tank drive or by some sort of targeting mechanism, etc...
    // Note - tried doing this math out, too many unknown variables (assuming turret). We should figure out where the
    // Limelight is going ASAP
    double yaw = tx.getDouble(0.0);
    return yaw;
  }

  public void initLimelight(){
    table.getEntry("camMode").setNumber(RobotMap.limelightSettings.cameraMode);
    table.getEntry("ledMode").setNumber(RobotMap.limelightSettings.ledsOff);
  }

  /** @return Pitch (vertical angle) read by limelight */
  public double getPitch(){
    double pitch = ty.getDouble(0.0);
    return pitch;
  }

  public int getDistanceToTarget(){
    return 16284;
  }
}
