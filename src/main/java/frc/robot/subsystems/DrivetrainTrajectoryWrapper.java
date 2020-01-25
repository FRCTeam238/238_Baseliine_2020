/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Robot;
import frc.robot.TrajectoryConstants;
import frc.robot.commands.RamseteCommand;

/**
 * pathweaver stuff
 * https://github.com/STMARobotics/frc-7028-2020/blob/master/src/main/java/frc/robot/subsystems/DriveTrainSubsystem.java
 */
public class DrivetrainTrajectoryWrapper extends Drivetrain {

    public static final class DriveTrainConstants {

        public static final int SENSOR_UNITS_PER_ROTATION = 4096;
        public static final double WHEEL_DIAMETER_INCHES = 6d;
        public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
        public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;

        public static final double TRACK_WIDTH_METERS = 0.555625;
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

        /** Voltage needed to overcome the motor’s static friction. kS */
        public static final double kS = 0.829;

        /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
        public static final double kV = 3.04;

        /** Voltage needed to induce a given acceleration in the motor shaft. kA */
        public static final double kA = 0.676;

        public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(kS, kV, kA);

        public static final double CLOSED_LOOP_RAMP = .2;
        public static final double OPEN_LOOP_RAMP = .25;
    }

    public static final int SENSOR_UNITS_PER_ROTATION = 4096;
    public static final double WHEEL_DIAMETER_INCHES = 6d;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;

    private final NavigationBoard gyro = Robot.navigationBoard;
    private final DifferentialDriveOdometry differentialDriveOdometry;
    private Pose2d savedPose;

    public DrivetrainTrajectoryWrapper() {
        super();
        differentialDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    }

    @Override
    public void periodic() {
        differentialDriveOdometry.update(Rotation2d.fromDegrees(getHeading()), stepsToMeters(getLeftEncoderTicks()),
                stepsToMeters(getRightEncoderTicks()));
        SmartDashboard.putString("Pose", differentialDriveOdometry.getPoseMeters().toString());
    }

    public void resetOdometry() {
        resetEncoders();
        gyro.zeroYaw();
        savedPose = new Pose2d(0, 0, Rotation2d.fromDegrees(getHeading()));
        differentialDriveOdometry.resetPosition(savedPose, Rotation2d.fromDegrees(getHeading()));
    }

    private double getHeading() {
        return gyro.getHeading();
    }

    /**
     * Creates a command to follow a Trajectory on the drivetrain.
     * 
     * @param trajectory trajectory to follow
     * @return command that will run the trajectory
     */
    public Command createCommandForTrajectory(Trajectory trajectory) {
        CommandGroup cg = new CommandGroup();

        cg.addSequential(new RamseteCommand(trajectory, this::getCurrentPose,
                new RamseteController(TrajectoryConstants.RAMSETE_B, TrajectoryConstants.RAMSETE_ZETA),
                DriveTrainConstants.DRIVE_KINEMATICS, this::tankDriveVelocity, this));
        cg.addSequential(new InstantCommand(this::stop));

        return cg;
    }

    /**
     * Controls the left and right side of the drive using Talon SRX closed-loop
     * velocity.
     * 
     * @param leftVelocity  left velocity
     * @param rightVelocity right velocity
     */
    public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
        var leftAccel = (leftVelocity - stepsPerDecisecToMetersPerSec(leftMasterDrive.getSelectedSensorVelocity()))
                / 20;
        var rightAccel = (rightVelocity - stepsPerDecisecToMetersPerSec(rightMasterDrive.getSelectedSensorVelocity()))
                / 20;

        var leftFeedForwardVolts = DriveTrainConstants.FEED_FORWARD.calculate(leftVelocity, leftAccel);
        var rightFeedForwardVolts = DriveTrainConstants.FEED_FORWARD.calculate(rightVelocity, rightAccel);

        leftMasterDrive.set(ControlMode.Velocity, metersPerSecToStepsPerDecisec(leftVelocity),
                DemandType.ArbitraryFeedForward, leftFeedForwardVolts / 12);
        rightMasterDrive.set(ControlMode.Velocity, metersPerSecToStepsPerDecisec(rightVelocity),
                DemandType.ArbitraryFeedForward, rightFeedForwardVolts / 12);
    }

    /**
     * Converts from encoder steps to meters.
     * 
     * @param steps encoder steps to convert
     * @return meters
     */
    public static double stepsToMeters(double steps) {
        return (WHEEL_CIRCUMFERENCE_METERS / SENSOR_UNITS_PER_ROTATION) * steps;
    }

    /**
     * Converts from meters to encoder units.
     * 
     * @param meters meters
     * @return encoder units
     */
    public static double metersToSteps(double meters) {
        return (meters / WHEEL_CIRCUMFERENCE_METERS) * SENSOR_UNITS_PER_ROTATION;
    }

    /**
     * Converts from encoder units per 100 milliseconds to meters per second.
     * 
     * @param stepsPerDecisec steps per decisecond
     * @return meters per second
     */

    public static double stepsPerDecisecToMetersPerSec(int stepsPerDecisec) {
        return stepsToMeters(stepsPerDecisec * 10);
    }

    /**
     * Convers from meters per second to encoder units per 100 milliseconds.
     * 
     * @param metersPerSec meters per second
     * @return encoder units per decisecond
     */
    public static double metersPerSecToStepsPerDecisec(double metersPerSec) {
        return metersToSteps(metersPerSec) * .1d;
    }

    public Pose2d getCurrentPose() {
        return differentialDriveOdometry.getPoseMeters();
    }

}

