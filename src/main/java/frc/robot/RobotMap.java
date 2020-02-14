/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.core238.wrappers.TalonSRX_238;;

/** Used to map hardware ports and define hardware objects - motor controllers, joysticks, cameras, etc */
public final class RobotMap {

  /** Motor controllers for the drive train */
  public static class DrivetrainControllers {
    public static final int DRIVE_TRAIN_RIGHT_MASTER = 0;
    public static final int DRIVE_TRAIN_RIGHT_SLAVE1 = 1;
    public static final int DRIVE_TRAIN_RIGHT_SLAVE2 = 2;

    public static final int DRIVE_TRAIN_LEFT_MASTER = 15;
    public static final int DRIVE_TRAIN_LEFT_SLAVE1 = 14;
    public static final int DRIVE_TRAIN_LEFT_SLAVE2 = 13;

    public static WPI_TalonSRX LeftMaster = TalonSRX_238.create(DRIVE_TRAIN_LEFT_MASTER, Robot.isSimulation());//new TalonSRX(DRIVE_TRAIN_LEFT_MASTER);
    public static WPI_VictorSPX LeftFollower1 = new WPI_VictorSPX(DRIVE_TRAIN_LEFT_SLAVE1);
    public static WPI_VictorSPX LeftFollower2 = new WPI_VictorSPX(DRIVE_TRAIN_LEFT_SLAVE2);
  
    public static WPI_TalonSRX RightMaster = TalonSRX_238.create(DRIVE_TRAIN_RIGHT_MASTER, Robot.isSimulation());
    public static WPI_VictorSPX RightFollower1 = new WPI_VictorSPX(DRIVE_TRAIN_RIGHT_SLAVE1);
    public static WPI_VictorSPX RightFollower2 = new WPI_VictorSPX(DRIVE_TRAIN_RIGHT_SLAVE2);
  }

    /** Driver joysticks and operator controllers */
    public static class Joysticks {
      public static int leftStickPort = 3;
      public static int rightStickPort = 2;
      public static int controllerPort = 1;
  
      public static Joystick driverStickLeft = new Joystick(leftStickPort);
      public static Joystick driverStickRight = new Joystick(rightStickPort);
      public static XboxController operatorController = new XboxController(controllerPort);
    }

  /** Integer settings for the vision subsystem - change camera mode, pipeline, and LED state using these */
  public static class LimelightSettings {
    public static int visionMode = 0;
    public static int cameraMode = 1;
    public static int ledsOn = 3;
    public static int ledsBlink = 2;
    public static int ledsOff = 1;
  }


  public static class ShooterDevices {
    //TODO: change to real number
    public static int SHOOTER_MASTER = 16;
    public static int SHOOTER_FOLLOWER = 17;
    public static CANSparkMax shooterMaster = new CANSparkMax(SHOOTER_MASTER, MotorType.kBrushless);
    public static CANSparkMax shooterFollower = new CANSparkMax(SHOOTER_FOLLOWER, MotorType.kBrushless);
    //public static TalonSRX shooterMaster = TalonSRX_238.create(SHOOTER_MASTER, Robot.isSimulation());
  }

  public static class IntakeDevices {
    //TODO: change to real number
    public static DoubleSolenoid intakeSolenoid;
    public static int INTAKE_MASTER_ID = 9;
    public static TalonSRX INTAKE_MASTER_TALON = new TalonSRX(INTAKE_MASTER_ID);
    public static int FORWARD_CHANNEL = 3;
    public static int REVERSE_CHANNEL = 4;
    //public static TalonSRX rightIntake = TalonSRX_238.create(INTAKE_MASTER, Robot.isSimulation());
  }

  public static class TurretDevices {
    //TODO: change to real number 
    public static int TURRET_MASTER = 11;
    public static TalonSRX turretTalon = TalonSRX_238.create(TURRET_MASTER, Robot.isSimulation());
  }

   public static class FeederDevices {
  //   //TODO: change to real number
     public static int FEEDER_MASTER = 10;
     public static TalonSRX feederTalon = TalonSRX_238.create(FEEDER_MASTER, Robot.isSimulation()); 
   }

  // public static class HangerDevices {
  //   //TODO: change to real number
  //   public static int HANGER_DEVICES = 0;
  //   public static TalonSRX hangerTalon = TalonSRX_238.create(HANGER_DEVICES, Robot.isSimulation()); 
  // }

  public static class PanelManipulatorDevices {
    //TODO: Change to real number
    public static double power = 0.5;

    //TODO: change the number to real numbers
    public static DoubleSolenoid panelSolenoid = new DoubleSolenoid(1,6);
  }

  public static class ClimberDevices {
    public static int CLIMBER_MASTER = 4;
    public static int CLIMBER_SLAVE = 5;
  }
}
