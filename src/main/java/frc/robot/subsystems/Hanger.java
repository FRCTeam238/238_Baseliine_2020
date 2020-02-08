/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.lang.reflect.Field;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.FieldConstants;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Hanger extends Subsystem {

    //private final TalonSRX hangerMasterDrive = RobotMap.HangerDevices.hangerTalon;
    double hangHeight = FieldConstants.hangHeight;
    double ticksPerInch;
	public Object resetEncoder;

    public Hanger() {
    }

    @Override
    protected void initDefaultCommand() {
        // TODO Auto-generated method stub

    }

    public void resetEncoder() {
        //hangerMasterDrive.setSelectedSensorPosition(0, 0, 0);
    }

    private double getPosition() {
        double encoderTicks = 111;
        //double encoderTicks = hangerMasterDrive.getSelectedSensorPosition();
        return encoderTicks;
    }

    public void hang(){
        //hangerMasterDrive.set(ControlMode.Position, inchesToTicks(hangHeight));
    }

    private double inchesToTicks(double inches){
        double ticks = inches * ticksPerInch;
        return ticks;
    }

    private double ticksToInches(double ticks){
        double inches = ticks / ticksPerInch;
        return inches;
    }

    private double getHeight(){
        double height = ticksToInches(getPosition());
        return height;
    }
}
