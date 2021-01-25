package frc.robot.utils.trajectory;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class CSPSwerveTrajectory {
    Trajectory trajectory;
    List<Double> times;
    List<Double> angles;
    
        


    public CSPSwerveTrajectory(Trajectory trajectory, List<Double> times, List<Double> angles) {
        this.trajectory = trajectory;
        this.times = times;
        this.angles = angles;
    }

    public CSPSwerveTrajectory(List<Pose2d> poses, List<Double> times, List<Double> angles) {
        TrajectoryConfig config = new TrajectoryConfig(Constants.Drive.Auto.MAX_VELOCITY, Constants.Drive.Auto.MAX_ACCEL)
        .addConstraint(
            new CentripetalAccelerationConstraint(Constants.Drive.Auto.MAX_CACCEL));
        
        this.trajectory = TrajectoryGenerator.generateTrajectory(poses, config);
        this.times = times;
        this.angles = angles;
    }

    /**
     * @return the trajectory
     */
    public Trajectory getTrajectory() {
        return trajectory;
    }

    public List<Double> getTimes() {
        return times;
    }

    public List<Double> getAngles() {
        return angles;
    }
}