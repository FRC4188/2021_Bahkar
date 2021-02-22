package frc.robot.utils.trajectory;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public final class TrajectoryList {

    public static final class TestAuto {
        public static CSPSwerveTrajectory motionOne = new CSPSwerveTrajectory(
            new Waypoints[]{
                WaypointsList.Testing.partA,
                WaypointsList.Testing.partB
            },
            new Rotation2d[]{
                new Rotation2d(Math.PI),
                new Rotation2d()
            }
        );
    }

    public static final class LeftEightBall {
        public static CSPSwerveTrajectory PICK_UP = new CSPSwerveTrajectory(
            new Waypoints[] {
                WaypointsList.LeftEightBall.DOWN_TRENCH,
                WaypointsList.LeftEightBall.TO_SHOOT
            }, 
            new Rotation2d[]{
                new Rotation2d(),
                new Rotation2d()
            }
            );
    }
}