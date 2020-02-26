/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.tests;

import org.junit.Assert;
import org.junit.Test;

import frc.robot.commands.TurretToTargetCommand;

/**
 * Add your docs here.
 */
public class TurretToTargetTest {

    @Test
    public void greaterAngleOfDriveTrain() {
        double yaw = 1;
        double turretAngle = 0;
        double expectedOutput = 89;
        double function = TurretToTargetCommand.greaterDrivetrain(yaw, turretAngle);
        System.out.println("Yaw: " + yaw);
        System.out.println("turret angle: " + turretAngle);
        System.out.println("expected output: " + expectedOutput);
        System.out.println("actual output: " + function);
        Assert.assertEquals("Turret Pos", expectedOutput, function, 0);
    }

    @Test
    public void lesserAngleOfDriveTrain() {
        double yaw = -90;
        double turretAngle = 0;
        double expectedOutput = 90;
        double function = TurretToTargetCommand.lesserDrivetrain(yaw, turretAngle);
        System.out.println("Yaw: " + yaw);
        System.out.println("turret angle: " + turretAngle);
        System.out.println("expected output: " + expectedOutput);
        System.out.println("actual output: " + function);
        Assert.assertEquals("Turret Pos", expectedOutput, function, 0);
    }

    @Test
    public void zeroYawDrivetrain() {
        double yaw = -90;
        double turretAngle = 0;
        double expectedOutput = 90;
        double function = TurretToTargetCommand.zeroDrivetrain(yaw, turretAngle);
        System.out.println("Yaw: " + yaw);
        System.out.println("turret angle: " + turretAngle);
        System.out.println("expected output: " + expectedOutput);
        System.out.println("actual output: " + function);
        Assert.assertEquals("Turret Pos", expectedOutput, function, 0);
    }
    
}
