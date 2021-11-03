package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Hanger2021 is the subsystem that controls the hanger which has a talon and 2 solenoids 
 * the talon is the motor wich controls the elevator and the solenoids fire the "claw" and "stopper"
 */
public class Hanger2021 extends Subsystem {
    private final DoubleSolenoid brakeSolenoid = RobotMap.ClimberDevices.brakeSolenoid; // RECHECK CHANNEL NUMBER
    private final DoubleSolenoid deploySolenoid = RobotMap.ClimberDevices.deploySolenoid; // RECHECK CHANNEL NUMBER
    private final TalonSRX climberTalon = RobotMap.ClimberDevices.climberTalon;
    public final DigitalInput topSwitch = RobotMap.ClimberDevices.topSwitch;
    public final DigitalInput bottomSwitch = RobotMap.ClimberDevices.bottomSwitch;

    public Hanger2021() {
        initTalon();
    }

    private void initTalon() {
        climberTalon.configFactoryDefault();
        climberTalon.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    protected void initDefaultCommand() {
        // TODO Auto-generated method stub

    }

    // getHeight()

    public void deployClawSolenoid() {
        deploySolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retractClawSolenoid() {
        deploySolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void retractBrake() {
        brakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void deployBrake() {
        climberTalon.set(ControlMode.PercentOutput, 0);
        brakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void controlTalon(double value) {
        climberTalon.set(ControlMode.PercentOutput, value);
    }

}