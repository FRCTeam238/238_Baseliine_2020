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
    private final TalonSRX turretMasterDrive = RobotMap.TurretDevices.turretTalon;

    public Turret() {
    }

    @Override
    protected void initDefaultCommand() {
        // TODO Auto-generated method stub

    }
    
    public void initTalons() {
        turretMasterDrive.configFactoryDefault();
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
