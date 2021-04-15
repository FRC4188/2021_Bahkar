package frc.robot.commands.drive.trajectorycontrol;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.trajectory.Waypoints;

public class FollowTrajectory extends SwerveControllerCommand {

    /**
     * Command controlling swerve drivetrain in trajectory following.
     * @param drivetrain Drivetrain object.
     * @param trajectory Trajectory to be followed.
     */
    public FollowTrajectory(Drivetrain drivetrain, Trajectory trajectory, Rotation2d angle) {
        super(trajectory, drivetrain::getPose, Constants.drive.KINEMATICS,
        new PIDController(5.2, 0.0, 0.0), new PIDController(-5.2, 0.0, 0.0),
        new ProfiledPIDController(0.08, 0.0, 0.02, new Constraints(Constants.drive.MAX_RADIANS, Constants.drive.MAX_RADIANS * 2)),
        () -> angle, drivetrain::setModuleStates, drivetrain);
    }

    /**
     * Command controlling swerve drivetrain in trajectory following.
     * @param drivetrain Drivetrain object.
     * @param waypoints Waypoint object for trajectory to be followed.
     */
    public FollowTrajectory(Drivetrain drivetrain, Waypoints waypoints) {
        this(drivetrain, waypoints, new Rotation2d());
    }

    public FollowTrajectory(Drivetrain drivetrain, Waypoints waypoints, Rotation2d angle) {
        this(drivetrain,
        TrajectoryGenerator.generateTrajectory(waypoints.getPoses(), drivetrain.getConfig()),
        angle);
    }
}