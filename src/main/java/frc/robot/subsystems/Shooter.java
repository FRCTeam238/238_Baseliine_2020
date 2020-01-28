/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.core238.wrappers.SendableWrapper;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {
    private final CANSparkMax shooterMasterDrive = RobotMap.ShooterDevices.shooterMaster;
    //private CANSparkMax shooterFollowerDrive;
    private final int followerID = 44;
    //NOT THE REAL ID
    private CANPIDController shooterPID;
    private CANEncoder shooterEncoder;

    private double kP = 1e-3;
    private double kI = 0;
    private double kD = 0;
    private double kIZ = 0;
    private double kFF = 1.85e-3;
    private double kMinOutput = 0;
    private double kMaxOutput = 12;

    private double desiredSpeedPID = 0;
    private double desiredPositionPID = 0;

    /*
     * private double integral = 0; private double derivative
     *  = 0; private double
     * previousError = 0; private double desiredSpeed = 0; private double
     * encoderTicks; private double previousEncoderTicks; private double error;
     * private double speed;
     */

    public Shooter() {
        initSparkMax();
        initLiveWindow();
    }

    public void initSparkMax() {
        shooterMasterDrive.restoreFactoryDefaults();
        //shooterFollowerDrive = new CANSparkMax(followerID, MotorType.kBrushless);
        //shooterFollowerDrive.restoreFactoryDefaults();
        //shooterFollowerDrive.follow(shooterMasterDrive);
        shooterPID = shooterMasterDrive.getPIDController();
        shooterEncoder = shooterMasterDrive.getEncoder();
        shooterPID.setP(kP);
        shooterPID.setI(kI);
        shooterPID.setD(kD);
        shooterPID.setIZone(kIZ);
        shooterPID.setFF(kFF);
        shooterPID.setOutputRange(kMinOutput, kMaxOutput);
    }

    @Override
    public void initDefaultCommand() {
    }

    private void resetEncoder() {
        shooterEncoder.setPosition(0);
    }

    private double getEncoderTicks() {
        //GIVES IN ROTATIONS
        double encoderTicks = shooterEncoder.getPosition();
        return encoderTicks;
    }

    private void pid_loop() {

        /*
         * previousEncoderTicks = encoderTicks; encoderTicks = getEncoderTicks(); speed
         * = (encoderTicks - previousEncoderTicks)/; //assuming that this loops 50 times
         * per second, 0.02 seconds for looop error = speed - desiredSpeed; integral +=
         * error*0.02; //continually adds up derivative = (error - previousError)/0.02
         */

    }

    public void setSpeed(double speedValue) {
        desiredSpeedPID = speedValue;
        shooterPID.setReference(speedValue, ControlType.kVelocity);
    }

    public void simpleSetSpeed(double speedValue){
        shooterMasterDrive.set(speedValue);
    }


    //Changed from percent to raw
    public double getPower() {
        double power = shooterMasterDrive.getOutputCurrent();
        return power;
    }

    public double getSpeed() {
        double speed = shooterEncoder.getVelocity();
        return speed;
    }

    public double getDesiredSpeed(){
        return desiredSpeedPID;
    }

    public void setPosition(double desiredPosition){
        shooterEncoder.setPosition(0);
        shooterPID.setReference(desiredPosition, ControlType.kPosition);
        desiredPositionPID = desiredPosition;
    }

    public double getPosition(){
        double position = shooterEncoder.getPosition();
        return position;
    }

    public double getDesiredPosition(){
        return desiredPositionPID;
    }


    public void neutral() {
        //shooterMasterDrive.set(ControlMode.PercentOutput, 0);
        shooterPID.setReference(0, ControlType.kVoltage);
        desiredSpeedPID = 0;
    }

    private void initLiveWindow() {
        SendableWrapper power = new SendableWrapper(builder -> {
            builder.addDoubleProperty("Power", this::getPower, null);
        });

        SendableWrapper speed = new SendableWrapper(builder -> {
            builder.addDoubleProperty("Speed", this::getSpeed, null);
        });

        SendableWrapper desiredSpeed = new SendableWrapper(builder -> {
            builder.addDoubleProperty("DesiredSpeed", this::getDesiredSpeed, null);
        });

        SendableWrapper position = new SendableWrapper(builder -> {
            builder.addDoubleProperty("Position", this::getPosition, null);
        });

        SendableWrapper desiredPosition = new SendableWrapper(builder -> {
            builder.addDoubleProperty("DesiredPosition", this::getDesiredPosition, null);
        });


        addChild("Power", power);
        addChild("Speed", speed);
        addChild("DesiredSpeed", desiredSpeed);
        addChild("Position", position);
        addChild("DesiredPosition", desiredPosition);
    }

    private List<SendableWrapper> _sendables = new ArrayList<>();
    private void addChild(String name, SendableWrapper wrapper){
      _sendables.add(wrapper);
      addChild(name, (Sendable)wrapper); 
    }

}
