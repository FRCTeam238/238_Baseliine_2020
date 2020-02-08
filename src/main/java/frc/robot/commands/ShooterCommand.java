/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.TurnTurretByVision;
import frc.robot.commands.PrepareToShoot;
import frc.robot.commands.IsAtSpeedCommand;
import frc.robot.commands.FeederCommand;

public class ShooterCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  Shooter theShooter = Robot.shooter;
  Feeder theFeeder = Robot.feeder;
  Turret theTurret = Robot.turret;

  public ShooterCommand() {
    requires(theFeeder);
    requires(theTurret);
    requires(theShooter);

    CommandGroup TargetingDistance = new CommandGroup();
    TargetingDistance.addParallel(new TurnTurretByVision());
    TargetingDistance.addParallel(new PrepareToShoot(150));

    CommandGroup FireBalls = new CommandGroup();
    FireBalls.addSequential(new IsAtSpeedCommand());
    FireBalls.addSequential(new FeederCommand());

    addSequential(TargetingDistance);
    addSequential(FireBalls);
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
