package frc.robot.utils.trajectory;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import frc.robot.Constants;

public final class WaypointsList {
    public static final class Testing {
        public static final Pose2d INIT_POSE = new Pose2d(0, 0, new Rotation2d(Math.PI/2));

        public static final Waypoints partA = new Waypoints(
            List.of(
                INIT_POSE,
                new Pose2d(0.0, 1.0, new Rotation2d(Math.PI/2))
            ),
            new TrajectoryConfig(
                Constants.drive.auto.MAX_VELOCITY, Constants.drive.auto.MAX_ACCEL)
                .addConstraint(new CentripetalAccelerationConstraint(Constants.drive.auto.MAX_CACCEL)));

        public static final Waypoints partB = new Waypoints(
            List.of(
                new Pose2d(7.0, -1.7, new Rotation2d()),
                new Pose2d(10.0, 0.0, new Rotation2d())
            ),
            new TrajectoryConfig(
                Constants.drive.auto.MAX_VELOCITY, Constants.drive.auto.MAX_ACCEL)
                .addConstraint(new CentripetalAccelerationConstraint(Constants.drive.auto.MAX_CACCEL)));
    }

    public static final class tuning {
        public static final Pose2d INIT_POSE = new Pose2d();

        public static final Waypoints stuff = new Waypoints(
            List.of(
                INIT_POSE,
                new Pose2d(1.0, 0.0, new Rotation2d()),
                new Pose2d(1.5, 1.0, new Rotation2d(Math.PI)),
                new Pose2d(0.0, 0.0, new Rotation2d(Math.PI))
            ),
            new TrajectoryConfig(Constants.drive.auto.MAX_VELOCITY, Constants.drive.auto.MAX_ACCEL)
        );

        public static final Waypoints stuff2 = new Waypoints(
            List.of(
                new Pose2d(1.0, 0.0, new Rotation2d(Math.PI / 2.0)),
                new Pose2d(1.0, 1.0, new Rotation2d(Math.PI / 2.0))
            ),
            new TrajectoryConfig(Constants.drive.auto.MAX_VELOCITY, Constants.drive.auto.MAX_ACCEL)
        );
    }

    public static final class TrenchRun {
        public static final Pose2d INIT_POSE = new Pose2d();

        public static final Waypoints PATH = new Waypoints(
            List.of(
                INIT_POSE,
                new Pose2d(2.12, -2.77, new Rotation2d()),
                new Pose2d(3.97, -2.77, new Rotation2d()),
                new Pose2d(5.19, -2.77, new Rotation2d()),
                new Pose2d(9.31, 0.0, new Rotation2d())
            ), new TrajectoryConfig(2.0, 1.0)
            .addConstraint(new CentripetalAccelerationConstraint(1.0)
            ));
    }
}