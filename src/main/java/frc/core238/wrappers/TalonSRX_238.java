/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.core238.wrappers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.time.StopWatch;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimEnum;

/**
 * Add your docs here.
 */
public class TalonSRX_238 extends TalonSRX {

    private SimDevice _simDevice;
    private SimBoolean _simConnected;
    private SimEnum _simMode;
    private SimDouble _simOutput;
    private SimDouble _simTicks;
    private SimDouble _simDistance;

    public TalonSRX_238(int deviceNumber) {
        super(deviceNumber);
        // simulation
        _simDevice = SimDevice.create("TalonSRX", deviceNumber);
        if (_simDevice != null) {
            _simConnected = _simDevice.createBoolean("Connected", false, true);
            _simMode = _simDevice
                    .createEnum(
                            "ControlMode", false, new String[] { "PercentOutput", "Position", "Velocity", "Current",
                                    "Follower", "MotionProfile", "MotionMagic", "MotionProfileArc" },
                            ControlMode.Current.value);
            _simOutput = _simDevice.createDouble("Output", false, 0);
            _simTicks = _simDevice.createDouble("EncoderTicks", false, 0);
            _simDistance = _simDevice.createDouble("Distance", false, 0);
        }
    }

    private static double _ticksPerInch = 194;
    private void setTicks(double value){
        _simTicks.set(value);
        _simDistance.set(value / _ticksPerInch);
    }

    @Override
    public void set(ControlMode mode, double outputValue) {
        if (_simConnected.get()) {
            double currentOutput = _simOutput.get();

            _simMode.set(mode.value);
            _simOutput.set(outputValue);

            if (currentOutput != outputValue){
                if (outputValue == 0) {
                    if (_stopWatch != null) {
                        double elapsed = _stopWatch.getDuration();
                        _encoderTicks += getScaledTicks(elapsed, currentOutput);
                        _stopWatch = null;
                    }
                } else {
                    if (_stopWatch == null){
                        _stopWatch = new StopWatch();
                        _stopWatch.start();
                    } else {
                        double elapsed = _stopWatch.getDuration();
                        _stopWatch = new StopWatch();
                        _stopWatch.start();
                        _encoderTicks += getScaledTicks(elapsed, currentOutput);
                    }
                }
                setTicks(_encoderTicks);
            }

            return;
        }
        super.set(mode, outputValue);
    }

    private double getScaledTicks(double elapsed, double output){
        return (maxTicksPerSecond*output)*(elapsed);
    }

    private int _encoderTicks = 0;
    private static double maxTicksPerSecond = 100.0;
    private StopWatch _stopWatch = null;

    @Override
    public int getSelectedSensorPosition(int pidIdx){
        if (_simConnected.get()){
            if (_stopWatch != null){
                double elapsed = _stopWatch.getDuration();
                _stopWatch = new StopWatch();
                _stopWatch.start();
                _encoderTicks += getScaledTicks(elapsed, _simOutput.get());
            }
            setTicks(_encoderTicks);
            return _encoderTicks;
        }
        return super.getSelectedSensorPosition(pidIdx);
    }

    @Override
    public ErrorCode setSelectedSensorPosition(int position, int pidIdx, int timeout){
        if (_simConnected.get()){
            _encoderTicks = position;
            setTicks(_encoderTicks);
            return ErrorCode.OK;
        }
        return super.setSelectedSensorPosition(position, pidIdx, timeout);
    }

    @Override
    public double getMotorOutputPercent() {
        return _simOutput.get();
    }

    public static TalonSRX create(int deviceNumber, Boolean isSimulation) {
        return isSimulation ? new TalonSRX_238(deviceNumber) : new TalonSRX(deviceNumber);
    }
}
