/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.tests;

import org.junit.Assert;
import org.junit.Test;

import frc.robot.subsystems.NavigationBoard;

/**
 * Add your docs here.
 */
public class GetAbsoluteYawTest {

    private NavigationBoard navboard;
    private double newYaw = 0;

    @Test
    public void GetAbsoluteYaw() {
        double currentYaw = 1;
        double previousYaw = 0;
        double newYawExpected = 1;
    
        previousYaw = currentYaw;
		newYaw = previousYaw;
        
        Assert.assertEquals("Absolute Yaw", newYawExpected, newYaw, 0);
    }
}
