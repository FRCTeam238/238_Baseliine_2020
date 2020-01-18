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
import frc.robot.Constants;
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

  /** Takes distance in inches and returns sensor ticks */
  public double getTicks(double distance){
    double inchesPerRev = Constants.robotGeometry.wheelCircumference * Math.PI;
    double revs = distance / inchesPerRev;
    double ticks = revs * Constants.robotGeometry.ticksPerRev;
    return ticks;
  }

  public void zeroEncoders(){
    rightTalon.setSelectedSensorPosition(0, Constants.CTRE_PID.kPIDLoopIdx, Constants.CTRE_PID.kTimeoutMs);
    leftTalon.setSelectedSensorPosition(0, Constants.CTRE_PID.kPIDLoopIdx, Constants.CTRE_PID.kTimeoutMs);
  }

  public void prepTalons(){
    /* Config the peak and nominal outputs ([-1, 1] represents [-100, 100]%) */
    leftTalon.configNominalOutputForward(0, Constants.CTRE_PID.kTimeoutMs);
    leftTalon.configNominalOutputReverse(0, Constants.CTRE_PID.kTimeoutMs);
    leftTalon.configPeakOutputForward(1, Constants.CTRE_PID.kTimeoutMs);
    leftTalon.configPeakOutputReverse(-1, Constants.CTRE_PID.kTimeoutMs);

    rightTalon.configNominalOutputForward(0, Constants.CTRE_PID.kTimeoutMs);
    rightTalon.configNominalOutputReverse(0, Constants.CTRE_PID.kTimeoutMs);
    rightTalon.configPeakOutputForward(1, Constants.CTRE_PID.kTimeoutMs);
    rightTalon.configPeakOutputReverse(-1, Constants.CTRE_PID.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be
     * neutral within this range.
     */
    leftTalon.configAllowableClosedloopError(0, Constants.CTRE_PID.kPIDLoopIdx, Constants.CTRE_PID.kTimeoutMs);
    rightTalon.configAllowableClosedloopError(0, Constants.CTRE_PID.kPIDLoopIdx, Constants.CTRE_PID.kTimeoutMs);

    /* Config closed loop gains for Primary closed loop (Current) */
    leftTalon.config_kP(Constants.CTRE_PID.kPIDLoopIdx, Constants.CTRE_PID.kP, Constants.CTRE_PID.kTimeoutMs);
    leftTalon.config_kI(Constants.CTRE_PID.kPIDLoopIdx, Constants.CTRE_PID.kI, Constants.CTRE_PID.kTimeoutMs);
    leftTalon.config_kD(Constants.CTRE_PID.kPIDLoopIdx, Constants.CTRE_PID.kD, Constants.CTRE_PID.kTimeoutMs);
    leftTalon.config_kF(Constants.CTRE_PID.kPIDLoopIdx, Constants.CTRE_PID.kF, Constants.CTRE_PID.kTimeoutMs);
    leftTalon.config_IntegralZone(Constants.CTRE_PID.kPIDLoopIdx, Constants.CTRE_PID.kIzone, Constants.CTRE_PID.kTimeoutMs);

    rightTalon.config_kP(Constants.CTRE_PID.kPIDLoopIdx, Constants.CTRE_PID.kP, Constants.CTRE_PID.kTimeoutMs);
    rightTalon.config_kI(Constants.CTRE_PID.kPIDLoopIdx, Constants.CTRE_PID.kI, Constants.CTRE_PID.kTimeoutMs);
    rightTalon.config_kD(Constants.CTRE_PID.kPIDLoopIdx, Constants.CTRE_PID.kD, Constants.CTRE_PID.kTimeoutMs);
    rightTalon.config_kF(Constants.CTRE_PID.kPIDLoopIdx, Constants.CTRE_PID.kF, Constants.CTRE_PID.kTimeoutMs);
    rightTalon.config_IntegralZone(Constants.CTRE_PID.kPIDLoopIdx, Constants.CTRE_PID.kIzone, Constants.CTRE_PID.kTimeoutMs);
    
    // Config encoders
    rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.CTRE_PID.kPIDLoopIdx, Constants.CTRE_PID.kTimeoutMs);
    leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.CTRE_PID.kPIDLoopIdx, Constants.CTRE_PID.kTimeoutMs);

    // Ensure motor output and encoder velocity are proportional to each other
    // If they become inverted, set these to true
    rightTalon.setSensorPhase(false);
    leftTalon.setSensorPhase(false);

    zeroEncoders();
  }

  public void driveStraight(double inches){
    zeroEncoders();
    double targetTicks = getTicks(inches);
    rightTalon.set(ControlMode.Position, targetTicks);
    leftTalon.set(ControlMode.Position, targetTicks);
  }
}