package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

public class RamseteControl {
    private static final double kVolts = 0.19519; // aka kS (static friction)
    private static final double kVoltsSecondsPerMeter = 1.8922; // aka kV (direct impact on voltage)
    private static final double kVoltsSecondsSquaredPerMeter = 0.4371; // aka kA (direct impact of current)
    private static final double kPDriveVelocity = 2.6586; // aka kP (PID control for velocity error)
    private static final double kTrackWidthMeters = 0.79108;

    private static final double maxSpeedInMetersPerSecond = 1.5;
    private static final double maxAccellInMetersPerSecond = 2;
    private static final double maxVoltage = 10; // not direct voltage, but what's set to the motor controller.

    private static final double kRameseteB = 2;
    private static final double kRamesetZeta = 0.7; // Magic numbers provided by tutorial.

    /**
     * A simplified generator for RamseteCommands, to avoid having all of _everything_ above
     * need to be available to other classes.  Uses the default maxSpeed and maxAcceleration.
     * @param startPosition Initial position, including heading.
     * @param translations List of Translation2d describing motions.
     * @param finalPosition Final position, including heading.  Should be the mathematical sum of all input translations from the startPosition.
     * @return The assembled Ramsete Command.
     */
    public static RamseteCommand generateRamseteCommand(final Pose2d startPosition, final List<Translation2d> translations, final Pose2d finalPosition)
    {
        final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kVolts, kVoltsSecondsPerMeter, kVoltsSecondsSquaredPerMeter);
        final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
        final DifferentialDriveVoltageConstraint autoVoltConstraint =
                new DifferentialDriveVoltageConstraint(feedForward, driveKinematics, maxVoltage);

        final TrajectoryConfig driveTrajectoryConfig = new TrajectoryConfig(maxSpeedInMetersPerSecond, maxAccellInMetersPerSecond)
                .setKinematics(driveKinematics)
                .addConstraint(autoVoltConstraint);
        final  Trajectory inputTraj = TrajectoryGenerator.generateTrajectory(
                startPosition,
                translations,
                finalPosition,
                driveTrajectoryConfig);

       return new RamseteCommand(
                inputTraj,
                DriveSubsystem.getInstance()::getPose,
                new RamseteController(kRameseteB, kRamesetZeta),
                feedForward,
                driveKinematics,
                DriveSubsystem.getInstance()::getWheelSpeeds,
                new PIDController(kPDriveVelocity, 0, 0),
                new PIDController(kPDriveVelocity, 0, 0),
                DriveSubsystem.getInstance()::tankDriveSetVolts,
                DriveSubsystem.getInstance());
    }

    public static RamseteCommand generateRamseteBackupCommand(final Pose2d startPosition, final List<Translation2d> translations, final Pose2d finalPosition)
    {
        final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kVolts, kVoltsSecondsPerMeter, kVoltsSecondsSquaredPerMeter);
        final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
        final DifferentialDriveVoltageConstraint autoVoltConstraint =
                new DifferentialDriveVoltageConstraint(feedForward, driveKinematics, maxVoltage);

        final TrajectoryConfig driveTrajectoryConfig = new TrajectoryConfig(maxSpeedInMetersPerSecond, maxAccellInMetersPerSecond)
                .setKinematics(driveKinematics)
                .addConstraint(autoVoltConstraint)
                .setReversed(true);
        final  Trajectory inputTraj = TrajectoryGenerator.generateTrajectory(
                startPosition,
                translations,
                finalPosition,
                driveTrajectoryConfig);

        return new RamseteCommand(
                inputTraj,
                DriveSubsystem.getInstance()::getPose,
                new RamseteController(kRameseteB, kRamesetZeta),
                feedForward,
                driveKinematics,
                DriveSubsystem.getInstance()::getWheelSpeeds,
                new PIDController(kPDriveVelocity, 0, 0),
                new PIDController(kPDriveVelocity, 0, 0),
                DriveSubsystem.getInstance()::tankDriveSetVolts,
                DriveSubsystem.getInstance());
    }

    public static RamseteCommand generateRamseteStopCommand(final Pose2d startPosition, final List<Translation2d> translations, final Pose2d finalPosition)
    {
        final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kVolts, kVoltsSecondsPerMeter, kVoltsSecondsSquaredPerMeter);
        final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
        final DifferentialDriveVoltageConstraint autoVoltConstraint =
                new DifferentialDriveVoltageConstraint(feedForward, driveKinematics, maxVoltage);

        final TrajectoryConfig driveTrajectoryConfig = new TrajectoryConfig(maxSpeedInMetersPerSecond, maxAccellInMetersPerSecond)
                .setKinematics(driveKinematics)
                .addConstraint(autoVoltConstraint)
                .setEndVelocity(0.);
        final  Trajectory inputTraj = TrajectoryGenerator.generateTrajectory(
                startPosition,
                translations,
                finalPosition,
                driveTrajectoryConfig);

        return new RamseteCommand(
                inputTraj,
                DriveSubsystem.getInstance()::getPose,
                new RamseteController(kRameseteB, kRamesetZeta),
                feedForward,
                driveKinematics,
                DriveSubsystem.getInstance()::getWheelSpeeds,
                new PIDController(kPDriveVelocity, 0, 0),
                new PIDController(kPDriveVelocity, 0, 0),
                DriveSubsystem.getInstance()::tankDriveSetVolts,
                DriveSubsystem.getInstance());
    }
}
