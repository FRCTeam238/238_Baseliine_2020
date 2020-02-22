/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.core238.Logger;
import frc.core238.wrappers.SendableWrapper;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {
    private final CANSparkMax shooterMasterDrive = RobotMap.ShooterDevices.shooterMaster;
    //private final CANSparkMax shooterMasterDrive = RobotMap.ShooterDevices.shooterMaster;
    private CANSparkMax shooterFollowerDrive = RobotMap.ShooterDevices.shooterFollower;
    //private final int followerID = 17;
    //NOT THE REAL ID
    private CANPIDController shooterPID;
    private CANEncoder shooterEncoder;

    private double kP = 0.0008;//0.00005;
    private double kI = 0;
    private double kD = 0.01;
    private double kIZ = 0;
    private double kFF = 1.95e-4;
    private double kMinOutput = 0;
    private double kMaxOutput = 12;

    private double desiredSpeedPID = 0;
    private double desiredPositionPID = 0;

    public double shootTimePerBall = 3;

    public HashMap<Integer, Integer> distanceToShootMap = new HashMap<Integer, Integer>();

    /*
     * private double integral = 0; private double derivative
     *  = 0; private double
     * previousError = 0; private double desiredSpeed = 0; private double
     * encoderTicks; private double previousEncoderTicks; private double error;
     * private double speed;
     */

    public Shooter() {
        initSparkMax();
        // initLiveWindow();
        populateSpeedMap(distanceToShootMap);
    }

    public void initSparkMax() {
        shooterMasterDrive.restoreFactoryDefaults();
        shooterMasterDrive.setInverted(true);
        shooterFollowerDrive.restoreFactoryDefaults();
        shooterFollowerDrive.follow(shooterMasterDrive, true);
        shooterMasterDrive.setIdleMode(IdleMode.kCoast);
        shooterFollowerDrive.setIdleMode(IdleMode.kCoast);
        shooterPID = shooterMasterDrive.getPIDController();
        shooterEncoder = shooterMasterDrive.getEncoder();
        resetEncoder();
        shooterMasterDrive.setSmartCurrentLimit(40);
        shooterFollowerDrive.setSmartCurrentLimit(40);
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

    public void setSpeed(double speedValue) {
        desiredSpeedPID = speedValue;
        shooterMasterDrive.getPIDController().setReference(desiredSpeedPID, ControlType.kVelocity);
    }

    public void setPower(double power){
        if(Math.abs(power) > 0.15){
            double sign = ( power / (Math.abs(power)));
            power = sign*0.15;
        }
        shooterMasterDrive.set(power);
    }

    public boolean isAtSpeed() {

        //get current speed
        //eval against desiredSpeedPID
        // allow for range
        boolean inRange = false;
        double tolerance = 200;
        double currentSpeed = getSpeed();
        double speedDifference = currentSpeed - desiredSpeedPID;
        if(Math.abs(speedDifference) < tolerance){
            inRange = true;
        }else{
            inRange = false;
        }
        Logger.Debug("Shooter wheel in range: " + inRange);
        return inRange;
    }

    public void simpleSetSpeed(double speedValue){
        //shooterMasterDrive.set(speedValue);
    }


    //Changed from percent to raw
    public double getCurrent() {
        double power = shooterMasterDrive.getOutputCurrent();
        return power;
    }

    public double getPower(){
        double power = shooterMasterDrive.get();
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
        shooterPID.setReference(0, ControlType.kVoltage);
        desiredSpeedPID = 0;
    }

    private void initLiveWindow() {
        SendableWrapper power = new SendableWrapper(builder -> {
            builder.addDoubleProperty("Power", this::getCurrent, null);
        });

        SendableWrapper speed = new SendableWrapper(builder -> {
            builder.addDoubleProperty("Speed", this::getSpeed, null);
        });

        SendableWrapper desiredSpeed = new SendableWrapper(builder -> {
            builder.addDoubleProperty("DesiredSpeed", this::getDesiredSpeed, null);
        });

        SendableWrapper actualPower = new SendableWrapper(builder -> {
            builder.addDoubleProperty("actualPower", this::getPower, null);
        });

        addChild("Power", power);
        addChild("Speed", speed);
        addChild("DesiredSpeed", desiredSpeed);
        addChild("Actual power", actualPower);
    }

    private List<SendableWrapper> _sendables = new ArrayList<>();
    private void addChild(String name, SendableWrapper wrapper){
      _sendables.add(wrapper);
      addChild(name, (Sendable)wrapper); 
    }

    private void populateSpeedMap(HashMap map){
        // Format: (distance, speed)
        // Distance is in inches, speed is in controller-side RPM
        map.put(80, 5000);
        map.put(96, 3600);
        map.put(112, 3600);
        map.put(128, 3620);
        map.put(144, 3665);
        map.put(160, 3750);
        map.put(176, 3840);
        map.put(192, 3950);
        map.put(208, 4000);
        map.put(224, 4075);
        map.put(240, 4100);
        map.put(256, 4150);
        map.put(272, 4230);
        map.put(288, 4300);
        map.put(304, 4415);
        map.put(320, 4575);
        map.put(336, 4700);
        map.put(352, 4900);
        map.put(368, 5000);
        map.put(384, 5100);
        map.put(400, 5270);
    }

    public int readSpeedMap(int distance){
        int neededRPMS = 0;
        int distanceRemainder = distance % 16;
        if(distanceRemainder == 0){
            if(distanceToShootMap.containsKey(distance)){
                neededRPMS = distanceToShootMap.get(distance);
            }else{
                Logger.Debug("Shooter.java line 231 - key not found");
            }
            
        }else{
            if(distanceRemainder <= 8){
                if(distanceToShootMap.containsKey(distance - distanceRemainder)){
                    neededRPMS = distanceToShootMap.get(distance - distanceRemainder);
                }else{
                    Logger.Debug("Shooter.java line 239 - key not found");
                }
            }else{
                if(distanceToShootMap.containsKey(distance + (16 - distanceRemainder))){
                    neededRPMS = distanceToShootMap.get(distance + (16 - distanceRemainder));
                }else{
                    Logger.Debug("Shooter.java line 245 - key not found");
                }
            }
        }
        return neededRPMS;
    }
}
