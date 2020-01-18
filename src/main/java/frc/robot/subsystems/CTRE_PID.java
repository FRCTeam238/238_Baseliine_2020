/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class CTRE_PID extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX leftTalon = RobotMap.DrivetrainControllers.LeftMaster;
  TalonSRX rightTalon = RobotMap.DrivetrainControllers.RightMaster;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static void zeroEncoders(int kPIDLoopIdx, int kTimeoutMs, TalonSRX talon){
    talon.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
  }

  public static void initTalonPID(double kP, double kI, double kD, double kF, int kIzone, TalonSRX talon, int kTimeoutMs, int kPIDLoopIdx, double rampRate){

    /* Config the peak and nominal outputs ([-1, 1] represents [-100, 100]%) */
    talon.configNominalOutputForward(0, kTimeoutMs);
    talon.configNominalOutputReverse(0, kTimeoutMs);
    talon.configPeakOutputForward(1, kTimeoutMs);
    talon.configPeakOutputReverse(-1, kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be
     * neutral within this range.
     */
    talon.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

    /* Config closed loop gains for Primary closed loop (Current) */
    talon.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
    talon.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
    talon.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
    talon.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
    talon.config_IntegralZone(kPIDLoopIdx, kIzone, kTimeoutMs);
    
    // Config encoders
    talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);

    talon.configClosedloopRamp(rampRate, kTimeoutMs);

    // Ensure motor output and encoder velocity are proportional to each other
    // If they become inverted, set these to true
    talon.setSensorPhase(true);

    zeroEncoders(kPIDLoopIdx, kTimeoutMs, talon);
  }

  public static double getTicks(TalonSRX talon){
    double ticks = talon.getSelectedSensorPosition();
    return ticks;
  }

  public static void moveToPosition(TalonSRX talon, double ticks){
    talon.set(ControlMode.Position, ticks);
  }
}