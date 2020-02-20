/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
    private final VictorSPX intakeMasterDrive = RobotMap.IntakeDevices.IntakeVictor;//IntakeDevices.INTAKE_MASTER_TALON;
    private final int forwarChannel = RobotMap.IntakeDevices.FORWARD_CHANNEL;
    private final int reverseChannel = RobotMap.IntakeDevices.REVERSE_CHANNEL;
    private DoubleSolenoid solenoid;

    private final double INTAKEPOWER = 0.5;

    public Intake() {
        initTalons();
        solenoid = new DoubleSolenoid(forwarChannel, reverseChannel);
        //solenoid = RobotMap.IntakeDevices.intakeSolenoid;
    }

    @Override 
    public void initDefaultCommand() {

    }

    public void initTalons() {
        intakeMasterDrive.configFactoryDefault();
        intakeMasterDrive.setInverted(true);
    }

    private void setPower(double speedValue){
        intakeMasterDrive.set(ControlMode.PercentOutput, speedValue);
    }

    public void in(double speed) {
        setPower(speed);
    }

    public void out(double speed) {
        setPower(speed);
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

    public DoubleSolenoid.Value getDirection() {
        return solenoid.get();
    }

    public BooleanSupplier isOutsideLimits(GenericHID controller, int axis){
        return () -> -0.2 >= controller.getRawAxis(axis) || controller.getRawAxis(axis) >= 0.2;
    }

}
