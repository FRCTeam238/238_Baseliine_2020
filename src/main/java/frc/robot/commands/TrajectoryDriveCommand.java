/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainTrajectoryWrapper;

@AutonomousModeAnnotation(parameterNames = { "TrajectoryName" })
public class TrajectoryDriveCommand extends CommandGroup implements IAutonomousCommand {

  private DrivetrainTrajectoryWrapper drivetrain = Robot.drivetrain;
  private boolean isAutonomousMode = false;
  private String trajectoryName;

  /**
   * Add your docs here.
   */
  /*public TrajectoryDriveCommand(Trajectory trajectory) {
    requires(Robot.drivetrain);
    addSequential(drivetrain.createCommandForTrajectory(trajectory));
  } */

  public TrajectoryDriveCommand() {
    requires(Robot.drivetrain);
  }

  @Override
  public boolean getIsAutonomousMode() {
    return isAutonomousMode;
  }

  @Override
  public void setIsAutonomousMode(boolean isAutonomousMode) {
    this.isAutonomousMode = isAutonomousMode;
  }

  @Override
  public void setParameters(List<String> parameters) {
    trajectoryName = parameters.get(0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Trajectory trajectory;
    try {
      trajectory = TrajectoryUtil
          .fromPathweaverJson(Paths.get("/home/lvuser/deploy/" + trajectoryName + ".wpilib.json"));
      addSequential(drivetrain.createCommandForTrajectory(trajectory));
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /*public static TrajectoryDriveCommand getExampleCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DrivetrainTrajectoryWrapper.DriveTrainConstants.kS,
            DrivetrainTrajectoryWrapper.DriveTrainConstants.kV, DrivetrainTrajectoryWrapper.DriveTrainConstants.kA),
        DrivetrainTrajectoryWrapper.DriveTrainConstants.DRIVE_KINEMATICS, 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DrivetrainTrajectoryWrapper.DriveTrainConstants.DRIVE_KINEMATICS)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);
    return new TrajectoryDriveCommand(exampleTrajectory);
  } */

  public static class AutoConstants {
    public static double kMaxSpeedMetersPerSecond = 1;
    public static double kMaxAccelerationMetersPerSecondSquared = 0.1;
  }
}