package frc.robot.utils.trajectory;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

public class CircleTest {
    Trajectory trajectory;
    public CircleTest(Drivetrain drivetrain) {
        TrajectoryConfig config = drivetrain.getConfig();

        trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.0, 0.0, new Rotation2d()),
            List.of(
                new Translation2d(/*4.35*/2, 0.16)
            ),
            new Pose2d(6.3, 0.16, new Rotation2d()),
            config
        );
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

}