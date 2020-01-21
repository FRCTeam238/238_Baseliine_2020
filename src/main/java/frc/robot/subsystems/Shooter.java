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

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.core238.wrappers.SendableWrapper;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {
    private final TalonSRX shooterMasterDrive = RobotMap.ShooterDevices.shooterMaster;

    private final double kP = 1;
    private final double kI = 1;
    private final double kD = 1;
    /*
     * private double integral = 0; private double derivative = 0; private double
     * previousError = 0; private double desiredSpeed = 0; private double
     * encoderTicks; private double previousEncoderTicks; private double error;
     * private double speed;
     */

    public Shooter() {
        initTalons();
        initLiveWindow();
    }

    public void initTalons() {
        shooterMasterDrive.configFactoryDefault();
    }

    @Override
    public void initDefaultCommand() {
    }

    private void resetEncoder() {
        shooterMasterDrive.setSelectedSensorPosition(0, 0, 0);
    }

    private double getEncoderTicks() {
        double encoderTicks = shooterMasterDrive.getSelectedSensorPosition();
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
        shooterMasterDrive.set(ControlMode.Velocity, speedValue);
    }

    public double getPower() {
        double power = shooterMasterDrive.getMotorOutputPercent();
        return power;
    }

    public double getSpeed() {
        double speed = shooterMasterDrive.getSelectedSensorVelocity();
        return speed;
    }

    public double getDesiredSpeed() {
        double wantedSpeed = shooterMasterDrive.getClosedLoopTarget();
        return wantedSpeed;
    }

    public double getSpeedError() {
        double currentSpeed = getSpeed();
        double wantedSpeed = getDesiredSpeed();
        double currentError = wantedSpeed - currentSpeed;
        return currentError;
    }

    public void stop() {
        shooterMasterDrive.set(ControlMode.PercentOutput, 0);
    }

    private void initLiveWindow() {
        SendableWrapper power = new SendableWrapper(builder -> {
            builder.addDoubleProperty("Power", this::getPower, null);
        });

        SendableWrapper speed = new SendableWrapper(builder -> {
            builder.addDoubleProperty("Speed", this::getSpeed, null);
        });

        SendableWrapper desiredSpeed = new SendableWrapper(builder -> {
            builder.addDoubleProperty("Desired Speed", this::getDesiredSpeed, null);
        });

        SendableWrapper speedError = new SendableWrapper(builder -> {
            builder.addDoubleProperty("Speed Error", this::getSpeedError, null);
        });

        SendableWrapper encoderTicks = new SendableWrapper(builder -> {
            builder.addDoubleProperty("Ticks", this::getEncoderTicks, null);
        });

        addChild("Power", power);
        addChild("Speed", speed);
        addChild("Desired Speed", desiredSpeed);
        addChild("Speed Error", speedError);
        addChild("EncoderTicks", encoderTicks);
    }

    private List<SendableWrapper> _sendables = new ArrayList<>();
    private void addChild(String name, SendableWrapper wrapper){
      _sendables.add(wrapper);
      addChild(name, (Sendable)wrapper);
    }

}
