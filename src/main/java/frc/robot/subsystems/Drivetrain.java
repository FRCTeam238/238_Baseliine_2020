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

  public final static double TICKS_PER_INCH = 217.299;
  private final static double ANGLE_KP = 3;
  private final double kV = 0.00434;// 0.00455
  private final double kA = 00.00434 * 0.15;
  private final double vSetpoint = 0.078;// 0.078

  private final static TalonSRX rightMasterDrive = RobotMap.DrivetrainControllers.RightMaster;
  private final static TalonSRX leftMasterDrive = RobotMap.DrivetrainControllers.LeftMaster;

  private final TalonSRX leftDriveFollower1 = RobotMap.DrivetrainControllers.LeftFollower1;
  private final TalonSRX rightDriveFollower1 = RobotMap.DrivetrainControllers.RightFollower1;

  public static double kP = 0.5;
  public static double kI = 1;
  public static double kD = 1;
  public static double kF = 1;
  public static int kIzone = 100;
  public static int kPIDLoopIdx = 0;
  public static int kTimeoutMs = 30;
  public static double rampRate = 0.1;

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

  public void driveWithTicks(double distance) {
    double ticks = distance * TICKS_PER_INCH;
    CTRE_PID.moveToPosition(rightMasterDrive, ticks);
    CTRE_PID.moveToPosition(leftMasterDrive, ticks);
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

    leftMasterDrive.setInverted(true);
    leftDriveFollower1.setInverted(true);

    leftDriveFollower1.follow(leftMasterDrive);
    // leftDriveFollower2.follow(leftMasterDrive);

    leftMasterDrive.setNeutralMode(NeutralMode.Brake);
    leftDriveFollower1.setNeutralMode(NeutralMode.Brake);
    // leftDriveFollower2.setNeutralMode(NeutralMode.Brake);

    // var rightDriveFollower2 = RobotMap.DrivetrainControllers.RightFollower2;

    rightDriveFollower1.follow(rightMasterDrive);
    // rightDriveFollower2.follow(rightMasterDrive);

    rightMasterDrive.setNeutralMode(NeutralMode.Brake);
    rightDriveFollower1.setNeutralMode(NeutralMode.Brake);
    // rightDriveFollower2.setNeutralMode(NeutralMode.Brake);

    rightMasterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 0);
    leftMasterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 0);

    CTRE_PID.initTalonPID(kP, kI, kD, kF, kIzone, leftMasterDrive, kTimeoutMs, kPIDLoopIdx, rampRate);
    CTRE_PID.initTalonPID(kP, kI, kD, kF, kIzone, rightMasterDrive, kTimeoutMs, kPIDLoopIdx, rampRate);

    Logger.Debug("initTalons Is Sucessful!");
  }

  public static void resetEncoders() {
    CTRE_PID.zeroEncoders(kPIDLoopIdx, kTimeoutMs, rightMasterDrive);
    CTRE_PID.zeroEncoders(kPIDLoopIdx, kTimeoutMs, leftMasterDrive);
  }
  public double getLeftEncoderTicks() {
    double leftTicks = CTRE_PID.getTicks(leftMasterDrive);
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
    double rightTicks = CTRE_PID.getTicks(rightMasterDrive);
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

    SendableWrapper leftSpeedController = new SendableWrapper(builder -> {
      builder.setSmartDashboardType("Speed Controller");
      builder.addDoubleProperty("Value", () -> leftMasterDrive.getMotorOutputPercent(), null);
    });

    SendableWrapper rightSpeedController = new SendableWrapper(builder -> {
      builder.setSmartDashboardType("Speed Controller");
      builder.addDoubleProperty("Value", () -> rightMasterDrive.getMotorOutputPercent(), null);
    });

    /*
    SendableWrapper leftInches = new SendableWrapper(builder -> {
      builder.addDoubleProperty("Inches", this::leftDistanceTravelled, null);
    });
    */

    addChild("Left Encoder", leftEncoder);
    addChild("Right Encoder", rightEncoder);
    addChild("Left Speed Controller", leftSpeedController);
    addChild("Right Speed Controller", rightSpeedController);
    //addChild("Left Inches", leftInches);
  }

  private List<SendableWrapper> _sendables = new ArrayList<>();
  private void addChild(String name, SendableWrapper wrapper){
    _sendables.add(wrapper);
    addChild(name, (Sendable)wrapper);
  }

}
