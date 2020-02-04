/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.core238.wrappers.AxisButton;
import frc.core238.wrappers.AxisButton.Axis;
import frc.robot.commands.DriveStraightPID;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.IntakeExtendRetractCommand;
import frc.robot.commands.PrepareToShoot;
import frc.robot.commands.RotatePanelNTimesBySensorCommand;
import frc.robot.commands.RotateToColorCommand;
import frc.robot.commands.IntakeInOutCommand;
import frc.robot.commands.VisionDrive;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public Joystick leftStick = RobotMap.Joysticks.driverStickLeft;
  public Joystick rightStick = RobotMap.Joysticks.driverStickRight;
  public Joystick operatorController = RobotMap.Joysticks.operatorController;
  public Drivetrain drivetrain;

  //public XboxController xboxController = RobotMap.XBoxController.xboxController;

  public OI(){


    VisionDrive visionDrive = new VisionDrive();
    DriveStraightPID driveTenFeetPID = new DriveStraightPID(-48);
    PrepareToShoot prepareToShoot = new PrepareToShoot();
    RotateToColorCommand rotateToColor = new RotateToColorCommand();
    RotatePanelNTimesBySensorCommand positionControl = new RotatePanelNTimesBySensorCommand(6);
    FeederCommand feedToShooter = new FeederCommand();
    
    JoystickButton visionTrackButton = new JoystickButton(leftStick, RobotMap.Buttons.visionTrack);

    JoystickButton driveTenFeetButton = new JoystickButton(leftStick, RobotMap.Buttons.driveTenFeet);

    JoystickButton climbButton = new JoystickButton(operatorController, RobotMap.Buttons.climb);

    JoystickButton deployManipulatorButton = new JoystickButton(operatorController, RobotMap.Buttons.deployManipulator);
    JoystickButton rotationControlButton = new JoystickButton(operatorController, RobotMap.Buttons.rotationControl);
    JoystickButton positionControlButton = new JoystickButton(operatorController, RobotMap.Buttons.positionControl);

    JoystickButton spinUpShooterButton = new JoystickButton(operatorController, RobotMap.Buttons.spinUpShooter);
    JoystickButton shootButton = new JoystickButton(operatorController, RobotMap.Buttons.shoot);

    driveTenFeetButton.whenPressed(driveTenFeetPID);
    visionTrackButton.whileHeld(visionDrive);
    spinUpShooterButton.whileHeld(prepareToShoot);
    rotationControlButton.whenPressed(rotateToColor);
    positionControlButton.whenPressed(positionControl);
    shootButton.whileHeld(feedToShooter);

    //up is negitive and down is positive
    IntakeInOutCommand intakeIn = new IntakeInOutCommand(true);
    AxisButton intakeInButton = new AxisButton(operatorController, Hand.kRight, Axis.Y, -0.5);
    intakeInButton.whileHeld(intakeIn);

    IntakeInOutCommand intakeOut = new IntakeInOutCommand(false);
    AxisButton intakeOutButton = new AxisButton(operatorController, Hand.kRight, Axis.Y, 0.5);
    intakeOutButton.whileHeld(intakeOut);

    JoystickButton extendIntakeJoystick = new JoystickButton(operatorController, XboxController.Button.kBumperLeft.value);
    extendIntakeJoystick.whenPressed(new IntakeExtendRetractCommand(true));

    JoystickButton retractIntakeJoystick = new JoystickButton(operatorController, XboxController.Button.kBumperRight.value);
    retractIntakeJoystick.whenPressed(new IntakeExtendRetractCommand(false));
  }
}
