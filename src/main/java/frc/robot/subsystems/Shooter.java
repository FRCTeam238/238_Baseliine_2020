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
import frc.core238.wrappers.SendableWrapper;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {
    private final TalonSRX shooterMasterDive = RobotMap.Shooter.shooterMaster;

    public Shooter() {
        initTalons();
        initLiveWindow();
    }

    public void initTalons() {
        shooterMasterDive.configFactoryDefault();
    }

    @Override
    public void initDefaultCommand() { 
    }

    public void resetEncoder(){
        shooterMasterDive.setSelectedSensorPosition(0,0,0);
    }
    public double getEncoderTicks(){
        double encoderTicks = shooterMasterDive.getSelectedSensorPosition();
        return encoderTicks;
    }

    public void setPower(double speedValue){
        shooterMasterDive.set(ControlMode.PercentOutput, speedValue);
    }

    public double getPower(){
        double power = shooterMasterDive.getMotorOutputPercent();
        return power;
        
    }

    public void stop(){
        shooterMasterDive.set(ControlMode.PercentOutput, 0);
    }

    private void initLiveWindow(){
        SendableWrapper power = new SendableWrapper(builder -> {
        builder.addDoubleProperty("Power", this::getPower, null);
        });
        
        SendableWrapper encoderTicks = new SendableWrapper(builder -> {
        builder.addDoubleProperty("Ticks", this::getEncoderTicks, null);
        });
        
        addChild("Power", power);
        addChild("EncoderTicks", encoderTicks);
    }
    
}
