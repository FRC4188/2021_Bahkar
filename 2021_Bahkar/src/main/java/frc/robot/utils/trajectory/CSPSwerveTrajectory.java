package frc.robot.utils.trajectory;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import frc.robot.Constants;

public class CSPSwerveTrajectory {
    List<Trajectory> trajectories;
    List<Rotation2d> angles;
    
    public CSPSwerveTrajectory(Waypoints[] waypointsArray, Rotation2d[] angleArray) {
        CentripetalAccelerationConstraint centrip = new CentripetalAccelerationConstraint(Constants.drive.auto.MAX_CACCEL);
        TrajectoryConfig config = new TrajectoryConfig(Constants.drive.auto.MAX_VELOCITY, Constants.drive.auto.MAX_ACCEL).addConstraint(centrip);

        for (Waypoints path : waypointsArray) {
            trajectories.add(TrajectoryGenerator.generateTrajectory(path.getPoses(), config));
        }
        for (Rotation2d angle : angleArray) {
            angles.add(angle);
        }
    }

    /**
     * @return the trajectory
     */
    public Trajectory getTrajectory(int index) {
        return trajectories.get(index);
    }

    public Rotation2d getAngle(int index) {
        return angles.get(index);
    }

    public int getLength() {
        return trajectories.size();
    }
}