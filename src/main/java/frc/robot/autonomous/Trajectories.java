package frc.robot.autonomous;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

import java.util.Arrays;
import java.util.List;

public class Trajectories {
    // Blue assumes that looking up is 0 on the gyro
    // Red assumes that looking down is 0 on the gyro


    private static final List<Trajectory> trajectories = Arrays.asList(
            TrajectoryGenerator.generateTrajectory(
                    new Pose2d(-0.05, -2.16, Rotation2d.fromDegrees(0)),
                    Arrays.asList(
                            new Translation2d(-0.65, -3.82),
                            new Translation2d(-3.14, -2.20)),
                    new Pose2d(-7.19, -2.91, Rotation2d.fromDegrees(-59)),
                    new TrajectoryConfig(1, 5)
            ),
            TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0.35, -1.71, Rotation2d.fromDegrees(0)),
                    Arrays.asList(
                            new Translation2d(-1.57, -1.07),
                            new Translation2d(-3.25, -3.87)
                    ),
                    new Pose2d(-5.43, -6.17, Rotation2d.fromDegrees(64)),
                    new TrajectoryConfig(1, 5)
            )
    );

    private Trajectories() {

    }

    public static Trajectory getTrajectory(int trajectory) {
        return trajectories.get(trajectory);
    }
}
