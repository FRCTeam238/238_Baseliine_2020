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
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.core238.wrappers.SendableWrapper;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Feeder extends Subsystem {
    public final TalonSRX feederMasterDrive = RobotMap.FeederDevices.feederTalon;
    public final DigitalOutput firstDetector = new DigitalOutput(255);
    //TODO: change FEEDER_OUTPUT to reasonable value;
    private final double FEEDER_OUTPUT = 0.5;
    private final double STOP_FEEDER_OUTPUT = 0;
    
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

    public void stop(){
        feederMasterDrive.set(ControlMode.PercentOutput, STOP_FEEDER_OUTPUT);
    }

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
