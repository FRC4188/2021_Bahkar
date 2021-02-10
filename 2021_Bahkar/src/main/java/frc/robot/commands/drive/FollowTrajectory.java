package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.trajectory.Waypoints;

public class FollowTrajectory extends SwerveControllerCommand {

    public FollowTrajectory(Drivetrain drivetrain, Waypoints waypoints) {
        super(
            TrajectoryGenerator.generateTrajectory(waypoints.getPoses(), drivetrain.getConfig().setReversed(waypoints.isReversed())),
            drivetrain::getPose,
            drivetrain.getKinematics(),
            new PIDController(1.0, 0.0, 0.0),
            new PIDController(1.0, 0.0, 0.0),
            new ProfiledPIDController(1.0, 0.0, 0.0, new Constraints(Constants.Drive.Auto.MAX_VELOCITY, Constants.Drive.Auto.MAX_ACCEL)),
            drivetrain::setModuleStates,
            drivetrain);
    }

}