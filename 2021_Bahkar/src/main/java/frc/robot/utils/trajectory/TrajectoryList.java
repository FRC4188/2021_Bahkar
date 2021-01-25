package frc.robot.utils.trajectory;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public final class TrajectoryList {
    public static final class TestAuto {
        public CSPSwerveTrajectory motionOne = new CSPSwerveTrajectory(
            List.of(
                new Pose2d(0.0, 0.0, new Rotation2d()),
                new Pose2d(1.5, 0.0, new Rotation2d()),
                new Pose2d(2.5, -1.0, new Rotation2d()),
                new Pose2d(3.25, 0.0, new Rotation2d(1.0, 0.0)),
                new Pose2d(0.0, 0.0, new Rotation2d(-Math.PI))
            ),
            List.of(
                0.0,
                0.5,
                1.0
            ),
            List.of(
                0.0,
                90.0,
                180.0
            ));
    }
}