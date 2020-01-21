/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Hanger extends Subsystem {
    private final TalonSRX turretMasterDrive = RobotMap.HangerDevices.hangerTalon;

    public Hanger() {
    }

    @Override
    protected void initDefaultCommand() {
        // TODO Auto-generated method stub

    }

    private void resetEncoder() {
        turretMasterDrive.setSelectedSensorPosition(0, 0, 0);
    }

    private double getEncoderTicks() {
        double encoderTicks = turretMasterDrive.getSelectedSensorPosition();
        return encoderTicks;
    }

}
