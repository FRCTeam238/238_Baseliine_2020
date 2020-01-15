/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public final class RobotMap {


  public static class DrivetrainControllers {
    public static final int DRIVE_TRAIN_RIGHT_MASTER = 8;
    public static final int DRIVE_TRAIN_RIGHT_SLAVE1 = 7;
    public static final int DRIVE_TRAIN_RIGHT_SLAVE2 = 2;

    public static final int DRIVE_TRAIN_LEFT_MASTER = 5;
    public static final int DRIVE_TRAIN_LEFT_SLAVE1 = 6;
    public static final int DRIVE_TRAIN_LEFT_SLAVE2 = 13;

    public static TalonSRX LeftMaster = new TalonSRX(DRIVE_TRAIN_LEFT_MASTER);
    public static TalonSRX LeftFollower1 = new TalonSRX(DRIVE_TRAIN_LEFT_SLAVE1);
    //public static VictorSPX LeftFollower2 = new VictorSPX(DRIVE_TRAIN_LEFT_SLAVE2);
  
    public static TalonSRX RightMaster = new TalonSRX(DRIVE_TRAIN_RIGHT_MASTER);
    public static TalonSRX RightFollower1 = new TalonSRX(DRIVE_TRAIN_RIGHT_SLAVE1);
    //public static VictorSPX RightFollower2 = new VictorSPX(DRIVE_TRAIN_RIGHT_SLAVE2);
  }

  public static class LimelightSettings {
    public static int visionMode = 0;
    public static int cameraMode = 1;
    public static int ledsOn = 3;
    public static int ledsBlink = 2;
    public static int ledsOff = 1;
  }

  public static class Joysticks {
    public static Joystick driverStickLeft;
    public static Joystick driverStickRight;
    public static Joystick operatorController;

    public static int leftStickPort = 3;
    public static int rightStickPort = 2;
    public static int controllerPort = 1;
  }

  public static class Buttons {
    public static int visionTrack = 1;
  }
}
