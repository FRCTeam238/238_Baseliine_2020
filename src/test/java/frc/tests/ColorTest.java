/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.tests;

import static org.junit.Assert.assertEquals;

import org.junit.Assert;
import org.junit.Test;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.PanelManipulator;

/**
 * Add your docs here.
 */
public class ColorTest {
    @Test
    public void ColorLess() {
        Color startColor = Color.kRed;
        Color endColor = Color.kGreen;

        double distanceToColor = PanelManipulator.colorSensing(startColor, endColor);
        
        //Assert.assertEquals("Distance to Color: ", 3, distanceToColor, 0);
    }

    @Test
    public void ColorMore() {
        Color startColor = Color.kGreen;
        Color endColor = Color.kBlue;

        double distanceToColor = PanelManipulator.colorSensing(startColor, endColor);
        
        Assert.assertEquals("Distance to Color: ", 3, distanceToColor, 0);
    }

    @Test
    public void ColorSame() {
        Color startColor = Color.kRed;
        Color endColor = Color.kRed;

        double distanceToColor = PanelManipulator.colorSensing(startColor, endColor);
        
        Assert.assertEquals("Distance to Color: ", 0, distanceToColor, 0);
    }

    // tripe AAA = 1)arrange, 2)act, 3)assert
}
