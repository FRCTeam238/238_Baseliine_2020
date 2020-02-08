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
public class Turret extends Subsystem {

    //TODO: temporary TalonSRX number for bench testing; CHANGE IT
    private final TalonSRX turretMasterDrive = RobotMap.FeederDevices.feederTalon;//TurretDevices.turretTalon;

    final double kF = 1e-4;
    final double kP = 0;
    final double kI = 0;
    final double kD = 0;

    public Turret() {
        initTalons();
    }

    @Override
    protected void initDefaultCommand() {
        // TODO Auto-generated method stub

    }
    
    public void initTalons() {
        turretMasterDrive.configFactoryDefault();
        turretMasterDrive.selectProfileSlot(0, 0);
        turretMasterDrive.config_kF(0, kF);
        turretMasterDrive.config_kP(0, kP);
        turretMasterDrive.config_kI(0, kI);
        turretMasterDrive.config_kD(0, kD);
    }

    private void resetEncoder() {
        turretMasterDrive.setSelectedSensorPosition(0, 0, 0);
    }

    private double getEncoderTicks() {
        double encoderTicks = turretMasterDrive.getSelectedSensorPosition();
        return encoderTicks;
    }

    public void setPosition(double angle) {
        //TODO: convert angle to position(ticks)
        double position = angle;
        turretMasterDrive.set(ControlMode.Position,  position);
    }

    public void setTurnVelocity(double magnitude){
        turretMasterDrive.set(ControlMode.PercentOutput, magnitude);
    }

    public double getPosition() {
        double position = turretMasterDrive.getSelectedSensorPosition();
        return position;
    }

    public void stop(){
        double currentPosition = getPosition();
        setPosition(currentPosition);
    }

    public void neutral(){
        turretMasterDrive.set(ControlMode.PercentOutput, 0);
    }

}
