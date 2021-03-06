/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Locale;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class PanelManipulator extends Subsystem {

  private TalonSRX talon;
  private static ColorSensorV3 sensor;
  private I2C.Port i2cPort = I2C.Port.kOnboard;
  private static ColorMatch match;

  public static Color kBlueTarget;
  public static Color kRedTarget;
  public static Color kYellowTarget;
  public static Color kGreenTarget;

  private static Color[] colorList; // {"G","R","Y","B"} <-- order of colors;

  public static DoubleSolenoid solenoid;

  @Override
  public void initDefaultCommand() {
    SmartDashboard.putData("Panel man", this);
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public PanelManipulator() {
    solenoid = RobotMap.PanelManipulatorDevices.panelSolenoid;
    // SmartDashboard.putString("Current Color", "Yellow");
    // SmartDashboard.putString("Assigned Color", "No!!!!");
  }

  public void initSensor() {
    sensor = new ColorSensorV3(i2cPort);

    defineColors();
  }

  public static void defineColors() {
    match = new ColorMatch();

    kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    colorList = new Color[] { kGreenTarget, kRedTarget, kYellowTarget, kBlueTarget };

    match.addColorMatch(kBlueTarget);
    match.addColorMatch(kGreenTarget);
    match.addColorMatch(kRedTarget);
    match.addColorMatch(kYellowTarget);
    match.addColorMatch(Color.kBlack);
  }

  public Color getColor() {
    return sensor.getColor();
  }

  public Color getMatchedColor() {
    ColorMatchResult matchResult = match.matchClosestColor(getColor());
    return matchResult.color;
  }

  public static Color toColor(String color) {
    switch (color.toLowerCase(Locale.ROOT)) {
    case ("green"):
      return Color.kGreen;
    case ("red"):
      return Color.kRed;
    case ("blue"):
      return Color.kBlue;
    case ("yellow"):
      return Color.kYellow;
    default:
      return Color.kBlack;
    }
  }

  public void postColor() {
    ColorMatchResult matchResult = match.matchClosestColor(getColor());
    String colorString;
    if (matchResult.color == kBlueTarget) {
      colorString = "Blue";
    } else if (matchResult.color == kRedTarget) {
      colorString = "Red";
    } else if (matchResult.color == kGreenTarget) {
      colorString = "Green";
    } else if (matchResult.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    SmartDashboard.putNumber("Confidence", matchResult.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }

  public double setPosition(double position) {
    talon.set(ControlMode.Position, position);
    return position;
  }

  public void rotate() {
    talon.set(ControlMode.PercentOutput, RobotMap.PanelManipulatorDevices.power);
  }

  public void stop() {
    talon.set(ControlMode.PercentOutput, 0);   
  }

  public void extend() {
    solenoid.set(Value.kForward);
  }

  public void retract() {
    solenoid.set(Value.kReverse);
  }

}
