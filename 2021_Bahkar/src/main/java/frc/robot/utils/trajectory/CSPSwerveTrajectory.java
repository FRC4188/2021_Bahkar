package frc.robot.utils.trajectory;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

public class CSPSwerveTrajectory {
    ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();
    ArrayList<Rotation2d> angles;
    
    public CSPSwerveTrajectory(Waypoints[] waypointsArray, ArrayList<Rotation2d> angleArray) {

        for (Waypoints path : waypointsArray) {
            trajectories.add(TrajectoryGenerator.generateTrajectory(path.getPoses(), path.getConfig()));
        }

        angles = angleArray;
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