/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.core238.Logger;
import frc.core238.wrappers.SendableWrapper;
import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;
import frc.robot.commands.drivetrainparameters.DriverJoysticks;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {

  // private LiveWindow lw = LiveWindow.getInstance();

  public final static double TICKS_PER_INCH = 194;
  private final static double ANGLE_KP = 3;
  private final double kV = 0.00434;// 0.00455
  private final double kA = 00.00434 * 0.15;
  private final double vSetpoint = 0.078;// 0.078

  private final TalonSRX rightMasterDrive = RobotMap.DrivetrainControllers.RightMaster;
  private final TalonSRX leftMasterDrive = RobotMap.DrivetrainControllers.LeftMaster;

  private final TalonSRX leftDriveFollower1 = RobotMap.DrivetrainControllers.LeftFollower1;
  private final TalonSRX rightDriveFollower1 = RobotMap.DrivetrainControllers.RightFollower1;

  public Drivetrain() {
    initTalons();
    initLiveWindow();
  }

  @Override
  public void initDefaultCommand() {
    DriverJoysticks myDriverJoysticks = new DriverJoysticks();
    TankDrive tankDriveCommand = new TankDrive(myDriverJoysticks, this);
    setDefaultCommand(tankDriveCommand);
  }

  public void drive(double left, double right) {
    leftMasterDrive.set(ControlMode.PercentOutput, left);
    rightMasterDrive.set(ControlMode.PercentOutput, right);
  }

  public void drive(double left, double right, double desiredAngle) {

    if (desiredAngle == 0) {
      drive(left, right);
    } else {
      if (Math.abs(desiredAngle) > (360.0 - 0.0) / 2.0D) {
        desiredAngle = desiredAngle > 0.0D ? desiredAngle - 360.0 + 0.0 : desiredAngle + 360.0 - 0.0;
      }

      double angleVelocityAddend = desiredAngle * ANGLE_KP;
      angleVelocityAddend = Math.min(50, Math.max(angleVelocityAddend, -50));

      accelerate(left + angleVelocityAddend, right - angleVelocityAddend, left, right);
    }
  }

  // method to accelerate rather than set straigt power
  public void accelerate(double leftSpeed, double rightSpeed, double leftAccel, double rightAccel) {

    /*
     * the joystick value is multiplied by a target RPM so the robot works with the
     * velocity tuning code
     */

    double leftWantedVoltage = 0;
    leftWantedVoltage += kV * leftSpeed;
    leftWantedVoltage += kA * leftAccel;

    leftWantedVoltage += leftWantedVoltage > 0 ? vSetpoint : -vSetpoint;

    double rightWantedVoltage = 0;
    rightWantedVoltage += kV * rightSpeed;
    rightWantedVoltage += kA * rightAccel;
    rightWantedVoltage += rightWantedVoltage > 0 ? vSetpoint : -vSetpoint;

    leftMasterDrive.config_kF(0, TALON_F_VALUE_LEFT * 10.0, 0);
    rightMasterDrive.config_kF(0, TALON_F_VALUE_RIGHT * 10.0, 0);

    leftMasterDrive.set(ControlMode.Velocity, (-leftSpeed) * TICKS_PER_INCH / 10.0);
    rightMasterDrive.set(ControlMode.Velocity, (-rightSpeed) * TICKS_PER_INCH / 10.0);
    // Logger.Log("DriveTrain.driveSpeedAccel() LEFT Speed = " + -leftSpeed + "RIGHT
    // Speed = " + -rightSpeed);
    // Logger.Log("Drive Accel RIGHT Speed:" + -rightSpeed);
    // convert to inches/second
    // Logger.Log("DriveTrain() : driveSpeed() : RIGHT SPEED IS ="
    // + leftFrontDrive.getSelectedSensorVelocity(0) /
    // CrusaderCommon.DRIVE_FORWARD_ENCODER_TICKS_PER_INCH);
    // Logger.Log("DriveTrain() : driveSpeed() : RIGHT ERROR IS =" +
    // rightFrontDrive.getClosedLoopError(0));

  }

  public void stop() {
    leftMasterDrive.set(ControlMode.PercentOutput, 0);
    rightMasterDrive.set(ControlMode.PercentOutput, 0);
  }

  public void initTalons() {
    rightMasterDrive.configFactoryDefault();
    leftMasterDrive.configFactoryDefault();
    leftDriveFollower1.configFactoryDefault();
    rightDriveFollower1.configFactoryDefault();

    // var leftDriveFollower2 = RobotMap.DrivetrainControllers.LeftFollower2;

    leftDriveFollower1.follow(leftMasterDrive);
    // leftDriveFollower2.follow(leftMasterDrive);

    // leftMasterDrive.setInverted(false);

    leftMasterDrive.setNeutralMode(NeutralMode.Brake);
    leftDriveFollower1.setNeutralMode(NeutralMode.Brake);
    // leftDriveFollower2.setNeutralMode(NeutralMode.Brake);

    // var rightDriveFollower2 = RobotMap.DrivetrainControllers.RightFollower2;

    rightDriveFollower1.follow(rightMasterDrive);
    // rightDriveFollower2.follow(rightMasterDrive);

    rightMasterDrive.setInverted(true);
    rightDriveFollower1.setInverted(true);

    rightMasterDrive.setNeutralMode(NeutralMode.Brake);
    rightDriveFollower1.setNeutralMode(NeutralMode.Brake);
    // rightDriveFollower2.setNeutralMode(NeutralMode.Brake);

    rightMasterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 0);
    leftMasterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 0);

    leftMasterDrive.config_kF(0, TALON_F_VALUE_LEFT, 0);
    rightMasterDrive.config_kF(0, TALON_F_VALUE_RIGHT, 0);

    leftMasterDrive.configOpenloopRamp(0.1, 100);
    rightMasterDrive.configOpenloopRamp(0.1, 100);

    leftMasterDrive.configClosedloopRamp(0.1, 100);
    rightMasterDrive.configClosedloopRamp(0.1, 100);

    leftMasterDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    rightMasterDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

    configTalon(leftMasterDrive);
    configTalon(rightMasterDrive);

    Logger.Debug("initTalons Is Sucessful!");
  }

  /**
   * configTalon is used to configure the master talons for velocity tuning so
   * they can be set to go to a specific velocity rather than just use a voltage
   * percentage This can be found in the CTRE Talon SRX Software Reference Manual
   * Section 12.4: Velocity Closed-Loop Walkthrough Java
   */
  private void configTalon(TalonSRX talon) {
    /*
     * This sets the voltage range the talon can use; should be set at +12.0f and
     * -12.0f
     */
    // talon.configNominalOutputVoltage(+0.0f, -0.0f);
    // talon.configPeakOutputVoltage(+12.0f, -12.0f);

    /*
     * This sets the FPID values to correct error in the motor's velocity
     */
    // talon.setProfile(CrusaderCommon.TALON_NO_VALUE);
    // .3113);
    talon.config_kP(0, TALON_P_VALUE, 0); // .8);//064543);
    talon.config_kI(0, TALON_NO_VALUE, 0);
    talon.config_kD(0, TALON_D_VALUE, 0);

    talon.set(ControlMode.Velocity, 0);

  }

  private final static double TALON_F_VALUE_LEFT = 0.00455;// 0.0725 old autonomous
  private final static double TALON_F_VALUE_RIGHT = 0.00455;// 0.0735 old autonomous
  private final static double TALON_P_VALUE = 0.2;// 0.5
  private final static double TALON_D_VALUE = 0;
  private final static int TALON_NO_VALUE = 0;

  /**
   * Resets the encoders by setting them to 0
   */
  public void resetEncoders() {
    leftMasterDrive.setSelectedSensorPosition(0, 0, 0);
    rightMasterDrive.setSelectedSensorPosition(0, 0, 0);
  }

  public double getLeftEncoderTicks() {
    double leftTicks = leftMasterDrive.getSelectedSensorPosition(0);
    Logger.Trace("LEFT TICKS: " + leftTicks);
    return leftTicks;
  }

  // return distance travelled in inches
  public double leftDistanceTravelled() {
    double leftTicks = getLeftEncoderTicks();
    double leftDistanceTravelled = leftTicks / TICKS_PER_INCH;
    Logger.Trace("LEFT TICKS: " + leftTicks + "  Left Distance Travelled  " + leftDistanceTravelled);
    return leftDistanceTravelled;
  }

  public double getRightEncoderTicks() {
    double rightTicks = rightMasterDrive.getSelectedSensorPosition(0);
    Logger.Trace("RIGHT TICKS: " + rightTicks);
    return rightTicks;
  }

  // return distance travelled in inches
  public double rightDistanceTravelled() {
    double rightTicks = getRightEncoderTicks();
    double rightDistanceTravelled = rightTicks / TICKS_PER_INCH;
    Logger.Trace("RIGHT TICKS: " + rightTicks);
    return rightDistanceTravelled;
  }

  private void initLiveWindow() {
    SendableWrapper leftEncoder = new SendableWrapper(builder -> {
      builder.addDoubleProperty("Ticks", this::getLeftEncoderTicks, null);
    });

    SendableWrapper rightEncoder = new SendableWrapper(builder -> {
      builder.addDoubleProperty("Ticks", this::getRightEncoderTicks, null);
    });

    SendableWrapper rigthSpeedController = new SendableWrapper(builder -> {
      builder.setSmartDashboardType("Speed Controller");
      builder.addDoubleProperty("Value", () -> rightMasterDrive.getMotorOutputPercent(), null);
    });

    addChild("Left Encoder", leftEncoder);
    addChild("Right Encoder", rightEncoder);
    addChild("Right Speed Controller", rigthSpeedController);
  }

  private List<SendableWrapper> _sendables = new ArrayList<>();
  private void addChild(String name, SendableWrapper wrapper){
    _sendables.add(wrapper);
    addChild(name, (Sendable)wrapper);
  }

}
