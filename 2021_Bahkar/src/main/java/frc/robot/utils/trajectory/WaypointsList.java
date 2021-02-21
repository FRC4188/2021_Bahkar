package frc.robot.utils.trajectory;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public final class WaypointsList {
    public static final class Testing {
        public static final Waypoints motionOne = new Waypoints(
            List.of(
                new Pose2d(),
                new Pose2d(3.0, -1.7, new Rotation2d()),
                new Pose2d(7.0, -1.7, new Rotation2d())
            ), false);
    }
}