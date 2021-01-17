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

        trajectory = TrajectoryGenerator.generateTrajectory( List.of(
            new Pose2d(0.0, 0.0, new Rotation2d()),
            new Pose2d(3.0 * 1.153, 0.0, new Rotation2d())
            //new Pose2d(3.0, 1.0, new Rotation2d(Math.PI / 2))
            ),
            config
        );
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

}