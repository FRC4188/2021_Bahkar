package frc.robot.utils;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;

public final class Trajectories {

    public static final class Testing {
        public static final Trajectory ONE_METER = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(), new Pose2d(1.0, 0.0, new Rotation2d())), new TrajectoryConfig(1.0, 2.0));

        public static final Trajectory CURVE = TrajectoryGenerator.generateTrajectory(new Pose2d(),
                List.of(new Translation2d(0.125, Math.cos(Math.asin(-0.875))),
                        new Translation2d(0.25, Math.cos(Math.asin(-0.75))),
                        new Translation2d(0.5, Math.cos(Math.asin(-0.5))),
                        new Translation2d(0.75, Math.cos(Math.asin(-0.25)))),
                new Pose2d(1.0, 1.0, new Rotation2d(Math.PI / 2.0)), new TrajectoryConfig(1.0, 2.0).addConstraint(new CentripetalAccelerationConstraint(2.0)));
    }
}