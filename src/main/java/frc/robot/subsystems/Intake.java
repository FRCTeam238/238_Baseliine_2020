/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
    private final TalonSRX intakeMasterDrive = RobotMap.Shooter.shooterMaster;

    public Intake() {
        initTalons();
    }

    public void initTalons() {
        intakeMasterDrive.configFactoryDefault();
    }

    @Override
    public void initDefaultCommand() { 
    }

    public void resetEncoder(){
        intakeMasterDrive.setSelectedSensorPosition(0,0,0);
    }
    public double getEncoderTicks(){
        double encoderTicks = intakeMasterDrive.getSelectedSensorPosition();
        return encoderTicks;
    }

    public void setPower(double speedValue){
        intakeMasterDrive.set(ControlMode.PercentOutput, speedValue);
    }

    public void stop(){
        intakeMasterDrive.set(ControlMode.PercentOutput, 0);
    }
}