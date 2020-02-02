/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
    private final TalonSRX intakeMasterDrive = RobotMap.IntakeDevices.INTAKE_MASTER_TALON;
    private final int forwarChannel = RobotMap.IntakeDevices.FORWARD_CHANNEL;
    private final int reverseChannel = RobotMap.IntakeDevices.REVERSE_CHANNEL;
    private DoubleSolenoid solenoid;

    private final double INTAKEPOWER = 0.5;

    public Intake() {
        initTalons();
        solenoid = new DoubleSolenoid(forwarChannel, reverseChannel);
    }

    @Override 
    public void initDefaultCommand() {
    }

    public void initTalons() {
        intakeMasterDrive.configFactoryDefault();
    }

    public void resetEncoder(){
        intakeMasterDrive.setSelectedSensorPosition(0,0,0);
    }

    public double getEncoderTicks(){
        double encoderTicks = intakeMasterDrive.getSelectedSensorPosition();
        return encoderTicks;
    }

    private void setPower(double speedValue){
        intakeMasterDrive.set(ControlMode.PercentOutput, speedValue);
    }

    public void in() {
        setPower(INTAKEPOWER);
    }

    public void out() {
        setPower(-INTAKEPOWER);
    }

    public void stop(){
        setPower(0);
    }

    public void extendIntake() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retractIntake() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

}
