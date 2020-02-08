/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.core238.wrappers.AxisButton;
import frc.core238.wrappers.AxisButton.Axis;
import frc.core238.wrappers.InstantTrigger;
import frc.robot.commands.DeployPanelManipulatorCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.HangCommand;
import frc.robot.commands.IntakeExtendRetractCommand;
import frc.robot.commands.IntakeInOutCommand;
import frc.robot.commands.RotatePanelNTimesBySensorCommand;
import frc.robot.commands.RotateToColorCommand;
import frc.robot.subsystems.Drivetrain;

//TODO: do we need the vision???????????? 
//import frc.robot.commands.VisionDrive;
//import frc.robot.commands.DriveStraightPID;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public Joystick leftStick = RobotMap.Joysticks.driverStickLeft;
  public Joystick rightStick = RobotMap.Joysticks.driverStickRight;
  public XboxController operatorController = RobotMap.Joysticks.operatorController;
  public Drivetrain drivetrain;

  public OI(){

    //TODO: do we need the vision????????????
    //VisionDrive visionDrive = new VisionDrive();
    //DriveStraightPID driveTenFeetPID = new DriveStraightPID(-48);
    //JoystickButton visionTrackButton = new JoystickButton(leftStick, RobotMap.Buttons.visionTrack); button:1
    //JoystickButton driveTenFeetButton = new JoystickButton(leftStick, RobotMap.Buttons.driveTenFeet); button: 6
    //driveTenFeetButton.whenPressed(driveTenFeetPID);
    //visionTrackButton.whileHeld(visionDrive);

    JoystickButton climbButton = new JoystickButton(operatorController, XboxController.Button.kY.value);
    climbButton.whenPressed(new HangCommand());

    JoystickButton deployPanelManipulatorButton = new JoystickButton(operatorController, XboxController.Button.kBumperLeft.value);
    deployPanelManipulatorButton.whenPressed(new DeployPanelManipulatorCommand());

    //TODO: Make a proper button mapping
    JoystickButton spinToProperColor = new JoystickButton(operatorController, XboxController.Button.kBumperLeft.value);
    spinToProperColor.whenPressed(new RotateToColorCommand());

    JoystickButton rotatePanelNTimesBySensor = new JoystickButton(operatorController, XboxController.Button.kA.value);
    rotatePanelNTimesBySensor.whenPressed(new RotatePanelNTimesBySensorCommand(FieldConstants.numberOfTimesToRotatePanelManipulator));

    JoystickButton spinUpShooterButton = new JoystickButton(operatorController, XboxController.Axis.kLeftTrigger.value);
    //spinUpShooterButton.whileHeld(new PrepareToShoot());

    JoystickButton shootButton = new JoystickButton(operatorController, XboxController.Axis.kRightTrigger.value);
    shootButton.whileHeld(new FeederCommand());

    //InstantTrigger spinIntakeOut = new InstantTrigger(Robot.intake.isOutsideLimits(operatorController, 1));
    //spinIntakeOut.whenActive(new IntakeInOutCommand(operatorController, XboxController.Axis.kLeftY.value));

    //up is negitive and down is positive
    //IntakeInOutCommand intakeInOutCommand = new IntakeInOutCommand(false); //()-> operatorController.getY(Hand.kRight));
    //Robot.intake.setDefaultCommand(intakeInOutCommand);
    // AxisButton intakeIn = new AxisButton(operatorController, Hand.kRight, Axis.Y, 0.2);
    // intakeIn.whenPressed(new IntakeInOutCommand(operatorController.getRawAxis(XboxController.Axis.kLeftY.value)));

    // AxisButton intakeOut = new AxisButton(operatorController, Hand.kRight, Axis.Y, -0.2);
    // intakeOut.whenPressed(new IntakeInOutCommand(operatorController.getRawAxis(XboxController.Axis.kLeftY.`value)));

    //JoystickButton extendIntakeJoystick = new JoystickButton(operatorController, XboxController.Button.kBumperLeft.value);
    ///extendIntakeJoystick.whenPressed(new IntakeExtendRetractCommand(true));

    JoystickButton retractIntakeJoystick = new JoystickButton(operatorController, XboxController.Button.kBumperRight.value);
    retractIntakeJoystick.whenPressed(new IntakeExtendRetractCommand());

    Robot.intake.setDefaultCommand(new IntakeInOutCommand(RobotMap.Joysticks.operatorController, XboxController.Axis.kLeftY.value));

  //  InstantTrigger t = new InstantTrigger(() -> Robot.feeder.getSensor1Triggered());
  
  }

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
