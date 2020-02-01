/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.DriveStraightPID;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.IntakeBallCommand;
import frc.robot.commands.PrepareToShoot;
import frc.robot.commands.RotatePanelNTimesBySensorCommand;
import frc.robot.commands.RotateToColorCommand;
import frc.robot.commands.VisionDrive;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public Joystick leftStick = RobotMap.Joysticks.driverStickLeft;
  public Joystick rightStick = RobotMap.Joysticks.driverStickRight;
  public Joystick controller = RobotMap.Joysticks.operatorController;
  public Drivetrain drivetrain;

  public OI(){

    VisionDrive visionDrive = new VisionDrive();
    DriveStraightPID driveTenFeetPID = new DriveStraightPID(-48);
    PrepareToShoot prepareToShoot = new PrepareToShoot();
    RotateToColorCommand rotateToColor = new RotateToColorCommand();
    RotatePanelNTimesBySensorCommand positionControl = new RotatePanelNTimesBySensorCommand(6);
    IntakeBallCommand intakeBall = new IntakeBallCommand();
    FeederCommand feedToShooter = new FeederCommand();
    
    JoystickButton visionTrackButton = new JoystickButton(leftStick, RobotMap.Buttons.visionTrack);

    JoystickButton driveTenFeetButton = new JoystickButton(leftStick, RobotMap.Buttons.driveTenFeet);

    JoystickButton climbButton = new JoystickButton(controller, RobotMap.Buttons.climb);

    JoystickButton deployManipulatorButton = new JoystickButton(controller, RobotMap.Buttons.deployManipulator);
    JoystickButton rotationControlButton = new JoystickButton(controller, RobotMap.Buttons.rotationControl);
    JoystickButton positionControlButton = new JoystickButton(controller, RobotMap.Buttons.positionControl);

    JoystickButton spinUpShooterButton = new JoystickButton(controller, RobotMap.Buttons.spinUpShooter);
    JoystickButton shootButton = new JoystickButton(controller, RobotMap.Buttons.shoot);

    JoystickButton intakeButton = new JoystickButton(controller, RobotMap.Buttons.intake);

    driveTenFeetButton.whenPressed(driveTenFeetPID);
    visionTrackButton.whileHeld(visionDrive);
    spinUpShooterButton.whileHeld(prepareToShoot);
    rotationControlButton.whenPressed(rotateToColor);
    positionControlButton.whenPressed(positionControl);
    intakeButton.whenPressed(intakeBall);
    shootButton.whileHeld(feedToShooter);
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
