/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Add your docs here.
 */
public class PanelManipulator extends Subsystem {

  TalonSRX talon;
  static ColorSensorV3 sensor;
  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorMatch match;

  Color kBlueTarget;
  Color kRedTarget;
  Color kYellowTarget;
  Color kGreenTarget;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public PanelManipulator() {
  }

  public void initSensor(){
    sensor = new ColorSensorV3(i2cPort);
    match = new ColorMatch();
    defineColors();
  }

  public void defineColors(){
    kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    match.addColorMatch(kBlueTarget);
    match.addColorMatch(kGreenTarget);
    match.addColorMatch(kRedTarget);
    match.addColorMatch(kYellowTarget);
    match.addColorMatch(Color.kBlack);
  }

  public Color getColor(){
    return sensor.getColor();
  }

  /** Checks whether a color is detected.
   *  @param color Color to check for - "green", "red", "blue", or "yellow" */
  public boolean isColor(String color){
    switch(color.toLowerCase(Locale.ROOT)){
      case("green") :
        return getColor() == Color.kGreen;
      case("red") :
        return getColor() == Color.kRed;
      case("blue") :
        return getColor() == Color.kBlue;
      case("yellow") :
        return getColor() == Color.kYellow;
      default :
        return false;
    }
  }

  public void postColor(){
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

  public static String[] colorList = new String[]{"G","R","Y","B"};
  public static double colorSensing(String currentColor, String desiredColor) {
    int colorRepeat;
    int colorCount;
    int i;
    for (colorCount = 0; colorCount < 4; colorCount++) {
      if (colorList[colorCount] == currentColor) {
        i = colorCount;
        break;
      }
    }
    for (i = colorCount; i <= 7; i++) {
     // currentColor = colorList.get(0);
     if (i < 3){
     currentColor = colorList[i];
     }
     if(i>3){
       colorRepeat = i-4;
       currentColor = colorList[colorRepeat];
     }
      
     if(currentColor == desiredColor){
       i=i+1;
      return i;
     }
     
    }
    return -1;
  }

}
