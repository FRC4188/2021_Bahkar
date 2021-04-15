package frc.robot.utils.trajectory;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;

import java.util.List;

/**
 * Class representing list of Pose2d objects and associated information for
 * trajectory generation.
 */
public class Waypoints {

    private final List<Pose2d> poses;
    private final TrajectoryConfig config;

    /**
     * Constructs a Waypoints object.
     * @param poses - poses to store in object.
     * @param config The TrajectoryConfig of the path once it is generated.
     */
    public Waypoints(List<Pose2d> poses, TrajectoryConfig config) {
        this.poses = poses;
        this.config = config;
    }

    /**
     * Returns the list of Pose2d objects associated with the object.
     */
    public List<Pose2d> getPoses() {
        return poses;
    }

    public TrajectoryConfig getConfig() {
        return config;
    }
}
