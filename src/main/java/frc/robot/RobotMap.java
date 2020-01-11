/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.core238.wrappers.TalonSRX_238;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public final class RobotMap {


  public static class SpeedControllers {
    private static final int DRIVE_TRAIN_RIGHT_MASTER = 8;
    private static final int DRIVE_TRAIN_RIGHT_SLAVE1 = 7;
    private static final int DRIVE_TRAIN_RIGHT_SLAVE2 = 2;

    private static final int DRIVE_TRAIN_LEFT_MASTER = 5;
    private static final int DRIVE_TRAIN_LEFT_SLAVE1 = 6;
    private static final int DRIVE_TRAIN_LEFT_SLAVE2 = 13;

    public static TalonSRX LeftMaster = new TalonSRX(DRIVE_TRAIN_LEFT_MASTER);
    public static TalonSRX LeftFollower1 = new TalonSRX(DRIVE_TRAIN_LEFT_SLAVE1);
    //public static VictorSPX LeftFollower2 = new VictorSPX(DRIVE_TRAIN_LEFT_SLAVE2);
  
    public static TalonSRX RightMaster = new TalonSRX(DRIVE_TRAIN_RIGHT_MASTER);
    public static TalonSRX RightFollower1 = new TalonSRX(DRIVE_TRAIN_RIGHT_SLAVE1);
    //public static VictorSPX RightFollower2 = new VictorSPX(DRIVE_TRAIN_RIGHT_SLAVE2);
  }
}
