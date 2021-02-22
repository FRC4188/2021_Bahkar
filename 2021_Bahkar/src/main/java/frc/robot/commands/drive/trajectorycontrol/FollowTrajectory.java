package frc.robot.commands.drive.trajectorycontrol;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
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
    public FollowTrajectory(Drivetrain drivetrain, Trajectory trajectory) {
        super(
            trajectory,
            drivetrain::getPose,
            drivetrain.getKinematics(),
            new PIDController(1.0, 0.0, 0.0),
            new PIDController(1.0, 0.0, 0.0),
            new ProfiledPIDController(1.0, 0.0, 0.0, new Constraints(Constants.Drive.Auto.MAX_VELOCITY, Constants.Drive.Auto.MAX_ACCEL)),
            drivetrain::setModuleStates,
            drivetrain);
    }

    /**
     * Command controlling swerve drivetrain in trajectory following.
     * @param drivetrain Drivetrain object.
     * @param waypoints Waypoint object for trajectory to be followed.
     */
    public FollowTrajectory(Drivetrain drivetrain, Waypoints waypoints) {
        this(drivetrain,
        TrajectoryGenerator.generateTrajectory(waypoints.getPoses(), drivetrain.getConfig()));
    }
}