/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.tests;

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
        PanelManipulator.defineColors();
        Color startColor = PanelManipulator.kRedTarget;
        Color endColor = PanelManipulator.kGreenTarget;

        double distanceToColor = PanelManipulator.getDistanceToColor(startColor, endColor);
        
        Assert.assertEquals("Distance to Color: ", 3, distanceToColor, 0);
    }

    @Test
    public void ColorMore() {
        PanelManipulator.defineColors();
        Color startColor = PanelManipulator.kGreenTarget;
        Color endColor = PanelManipulator.kBlueTarget;
        double distanceToColor = PanelManipulator.getDistanceToColor(startColor, endColor);
        
        Assert.assertEquals("Distance to Color: ", 3, distanceToColor, 0);
    }

    @Test
    public void ColorSame() {
        Color startColor = Color.kRed;
        Color endColor = Color.kRed;

        double distanceToColor = PanelManipulator.getDistanceToColor(startColor, endColor);
        
        Assert.assertEquals("Distance to Color: ", 0, distanceToColor, 0);
    }

    // tripe AAA = 1)arrange, 2)act, 3)assert
}
