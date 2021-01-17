package frc.robot.utils.trajectory;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

public class TestFile {
    Trajectory trajectory;
    public TestFile(Drivetrain drivetrain) {
        TrajectoryConfig config = drivetrain.getConfig();

        trajectory = TrajectoryGenerator.generateTrajectory( List.of(
            new Pose2d(0.0, 0.0, new Rotation2d()),
            new Pose2d(4.93, 2.17, new Rotation2d()),
            new Pose2d(6.92, 2.17, new Rotation2d()),
            new Pose2d(8.6, 2.17, new Rotation2d())),
            config
        );
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

}