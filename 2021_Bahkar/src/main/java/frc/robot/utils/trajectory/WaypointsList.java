package frc.robot.utils.trajectory;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public final class WaypointsList {
    public static final class Testing {
        public static final Waypoints partA = new Waypoints(
            List.of(
                new Pose2d(),
                new Pose2d(3.0, -1.7, new Rotation2d()),
                new Pose2d(7.0, -1.7, new Rotation2d())
            ));

        public static final Waypoints partB = new Waypoints(
            List.of(
                new Pose2d(7.0, -1.7, new Rotation2d()),
                new Pose2d(10.0, 0.0, new Rotation2d())
            ));
    }

    public static final class LeftEightBall {

        public static final Waypoints DOWN_TRENCH = new Waypoints(
            List.of(
                new Pose2d(),
                new Pose2d(2.0, 0.75, new Rotation2d()),
                new Pose2d(7.0, 0.75, new Rotation2d())
            ));
        
        public static final Waypoints TO_SHOOT = new Waypoints(
            List.of(
                new Pose2d(7.0, 0.75, new Rotation2d()),
                new Pose2d(3.0, -0.5, new Rotation2d())
            ));
    }
}