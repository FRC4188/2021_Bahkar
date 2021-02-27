package frc.robot.utils.trajectory;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public final class TrajectoryList {

    public static final class TestAuto {
        public static CSPSwerveTrajectory motionOne = new CSPSwerveTrajectory(
            new Waypoints[]{
                WaypointsList.Testing.partA,
                WaypointsList.Testing.partB
            },
            new ArrayList<Rotation2d>(List.of(
                new Rotation2d(Math.PI),
                new Rotation2d()
            ))
        );
    }

    public static final class TuningAuto {
        public static CSPSwerveTrajectory meter = new CSPSwerveTrajectory(
            new Waypoints[] {
                WaypointsList.tuning.stuff,
                WaypointsList.tuning.stuff2
            },
            new ArrayList<Rotation2d>(List.of(
                new Rotation2d(),
                new Rotation2d(-Math.PI/  2.0)
            ))
        );
    }
}