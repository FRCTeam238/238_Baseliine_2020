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

import frc.robot.subsystems.PanelManipulator;

/**
 * Add your docs here.
 */
public class ColorTest {
    @Test
    public void ColorTestOne() {
        String startColor = "R";
        String endColor = "G";

        double distanceToColor = PanelManipulator.colorSensing(startColor, endColor);
        
        Assert.assertEquals("Distance to Color: ", 3, distanceToColor, 0);
    }

    // tripe AAA = 1)arrange, 2)act, 3)assert
}
