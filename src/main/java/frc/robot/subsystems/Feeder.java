/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.core238.wrappers.SendableWrapper;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Feeder extends Subsystem {
    public final TalonSRX feederMasterDrive = RobotMap.IntakeDevices.INTAKE_MASTER_TALON;//FeederDevices.feederTalon;
    public final DigitalInput firstDetector = new DigitalInput(0);
    public final DigitalInput secondDetector = new DigitalInput(1);
    //TODO: change FEEDER_OUTPUT to reasonable value;
    private final double FEEDER_OUTPUT = 0.5;
    private final double STOP_FEEDER_OUTPUT = 0;
    private int heldBallsNumber = 0;

    public Feeder() {
        initLiveWindow();
    }

    @Override
    protected void initDefaultCommand() {
        // TODO Auto-generated method stub
    }

    public void start() {
        feederMasterDrive.set(ControlMode.PercentOutput, FEEDER_OUTPUT);
    }

    /*if(firstDetector == broken){
     turn on
    }

    if(secondDetector == broken){
        secondbroken = true
    } else {
        if(secondBroken == true){
            turn off
            secondBroken = false
        }
    }
    


    */

    public void stop(){
        feederMasterDrive.set(ControlMode.PercentOutput, STOP_FEEDER_OUTPUT);
    }
// public BooleanSupplier getSensor1Triggered (){

//     boolean test = firstDetector.get();

//     BooleanSupplier testBS = new BooleanSupplier(){
    
//         @Override
//         public boolean getAsBoolean() {
//             // TODO Auto-generated method stub
//             return test;
//         }
//     };
//     return testBS;
// }
    // public void feederLogicLoop(){
    //     boolean lastStateBroken = false;
    //     boolean secondSensorBroken = false;
    //     boolean firstSensorBroken = false;
    //     //firstSensorBroken = firstDetector.get();
    //     //secondSensorBroken = secondDetector.get();
    //     if(firstSensorBroken == true){
    //         start();
    //     }
    //     if(secondSensorBroken == false && lastStateBroken == true){
    //         heldBallsNumber++;
    //         stop();
    //     }
    //     //activate LEDs here
    //     if(secondSensorBroken == true){
    //         lastStateBroken = true;
    //     }
    // }
    //for testing moved to feedercommand

    private double getMotorOutput(){
        double motorOutput = feederMasterDrive.getSelectedSensorPosition();
        return motorOutput;
    }

    private void initLiveWindow() {
        SendableWrapper motor = new SendableWrapper(builder -> {
            builder.addDoubleProperty("Motor", this::getMotorOutput, null);
        });
        
        addChild("Motor", motor);
    }

    private List<SendableWrapper> _sendables = new ArrayList<>();
    private void addChild(String name, SendableWrapper wrapper){
      _sendables.add(wrapper);
      addChild(name, (Sendable)wrapper);
    }
    
}
