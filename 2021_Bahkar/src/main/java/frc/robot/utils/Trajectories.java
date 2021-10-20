package frc.robot.utils;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.subsystems.drive.Swerve;

public final class Trajectories {

        public static class trench6M {
        
                public static Rotation2d[] headings = {
                    new Rotation2d()
                };
        
                private static double adjustment = -0.15;
        
                public static Trajectory down = TrajectoryGenerator.generateTrajectory(
                    List.of(
                        new Pose2d(),
                        new Pose2d(1.78, 1.63 + adjustment, new Rotation2d()),
                        new Pose2d(5.5, 1.63 + adjustment, new Rotation2d())
                    ),
                    new TrajectoryConfig(4.0, 3.0)
                        .addConstraint(new CentripetalAccelerationConstraint(3.0))
                        .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
                );
        
                public static Trajectory back = TrajectoryGenerator.generateTrajectory(
                    List.of(
                        new Pose2d(5.5, 1.63 + adjustment, new Rotation2d(-Math.PI)),
                        new Pose2d(5.0, 1.63 + adjustment, new Rotation2d(-Math.PI)),
                        new Pose2d(0.0, 0.0, new Rotation2d(-Math.PI))
                    ),
                new TrajectoryConfig(4.0, 3.0)
                        .addConstraint(new CentripetalAccelerationConstraint(3.0))
                        .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
                );
            }

        public static class trench8M {
        
                public static Rotation2d[] headings = {
                    new Rotation2d(),
                    new Rotation2d(Math.PI / 8.0),
                    new Rotation2d(-Math.PI)
                };
        
                private static double adjustment = -0.15;
        
                public static Trajectory down = TrajectoryGenerator.generateTrajectory(
                    List.of(
                        new Pose2d(),
                        new Pose2d(1.78, 1.63 + adjustment, new Rotation2d()),
                        new Pose2d(5.5, 1.63 + adjustment, new Rotation2d())
                    ),
                    new TrajectoryConfig(4.0, 3.0)
                        .addConstraint(new CentripetalAccelerationConstraint(3.0))
                        .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
                        .setEndVelocity(0.5)
                );
        
                public static Trajectory ball1 = TrajectoryGenerator.generateTrajectory(
                    List.of(
                        new Pose2d(5.5, 1.63 + adjustment, new Rotation2d()),
                        new Pose2d(7.0, 1.53 + adjustment, new Rotation2d())
                    ),
                    new TrajectoryConfig(4.0, 3.0)
                        .addConstraint(new CentripetalAccelerationConstraint(3.0))
                        .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
                        .setStartVelocity(0.5)
                );
        
                public static Trajectory ball2 = TrajectoryGenerator.generateTrajectory(
                    List.of(
                        new Pose2d(7.0, 1.53 + adjustment, new Rotation2d(-Math.PI / 2.0)),
                        new Pose2d(7.0, 1.7 + adjustment, new Rotation2d(-Math.PI / 2.0))
                    ),
                    new TrajectoryConfig(4.0, 3.0)
                        .addConstraint(new CentripetalAccelerationConstraint(3.0))
                        .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
                );
        
                public static Trajectory back = TrajectoryGenerator.generateTrajectory(
                    List.of(
                        new Pose2d(5.0, 1.7 + adjustment, new Rotation2d(-Math.PI)),
                        new Pose2d(5.0, 1.63 + adjustment, new Rotation2d(-Math.PI)),
                        new Pose2d(0.0, 0.0, new Rotation2d(-Math.PI))
                    ),
                new TrajectoryConfig(4.0, 3.0)
                        .addConstraint(new CentripetalAccelerationConstraint(3.0))
                        .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
                );
            }

            public static class trench6L {
        
                public static Rotation2d[] headings = {
                    new Rotation2d()
                };
        
                private static double adjustment = -1.63;
        
                public static Trajectory down = TrajectoryGenerator.generateTrajectory(
                    List.of(
                        new Pose2d(),
                        new Pose2d(1.78, 1.63 + adjustment, new Rotation2d()),
                        new Pose2d(5.5, 1.63 + adjustment, new Rotation2d())
                    ),
                    new TrajectoryConfig(4.0, 3.0)
                        .addConstraint(new CentripetalAccelerationConstraint(3.0))
                        .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
                );
        
                public static Trajectory back = TrajectoryGenerator.generateTrajectory(
                    List.of(
                        new Pose2d(5.5, 1.63 + adjustment, new Rotation2d(-Math.PI)),
                        new Pose2d(5.0, 1.63 + adjustment, new Rotation2d(-Math.PI)),
                        new Pose2d(0.0, 0.0, new Rotation2d(-Math.PI))
                    ),
                new TrajectoryConfig(4.0, 3.0)
                        .addConstraint(new CentripetalAccelerationConstraint(3.0))
                        .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
                );
            }

        public static class trench8L {
        
                public static Rotation2d[] headings = {
                    new Rotation2d(),
                    new Rotation2d(Math.PI / 8.0),
                    new Rotation2d(-Math.PI)
                };
        
                private static double adjustment = -1.63;
        
                public static Trajectory down = TrajectoryGenerator.generateTrajectory(
                    List.of(
                        new Pose2d(),
                        new Pose2d(1.78, 1.63 + adjustment, new Rotation2d()),
                        new Pose2d(5.5, 1.63 + adjustment, new Rotation2d())
                    ),
                    new TrajectoryConfig(4.0, 3.0)
                        .addConstraint(new CentripetalAccelerationConstraint(3.0))
                        .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
                        .setEndVelocity(0.5)
                );
        
                public static Trajectory ball1 = TrajectoryGenerator.generateTrajectory(
                    List.of(
                        new Pose2d(5.5, 1.63 + adjustment, new Rotation2d()),
                        new Pose2d(7.0, 1.53 + adjustment, new Rotation2d())
                    ),
                    new TrajectoryConfig(4.0, 3.0)
                        .addConstraint(new CentripetalAccelerationConstraint(3.0))
                        .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
                        .setStartVelocity(0.5)
                );
        
                public static Trajectory ball2 = TrajectoryGenerator.generateTrajectory(
                    List.of(
                        new Pose2d(7.0, 1.53 + adjustment, new Rotation2d(-Math.PI / 2.0)),
                        new Pose2d(7.0, 1.7 + adjustment, new Rotation2d(-Math.PI / 2.0))
                    ),
                    new TrajectoryConfig(4.0, 3.0)
                        .addConstraint(new CentripetalAccelerationConstraint(3.0))
                        .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
                );
        
                public static Trajectory back = TrajectoryGenerator.generateTrajectory(
                    List.of(
                        new Pose2d(5.0, 1.7 + adjustment, new Rotation2d(-Math.PI)),
                        new Pose2d(5.0, 1.63 + adjustment, new Rotation2d(-Math.PI)),
                        new Pose2d(0.0, 0.0, new Rotation2d(-Math.PI))
                    ),
                new TrajectoryConfig(4.0, 3.0)
                        .addConstraint(new CentripetalAccelerationConstraint(3.0))
                        .addConstraint(new SwerveDriveKinematicsConstraint(Swerve.getInstance().getKinematics(), 4.0))
                );
            }
}