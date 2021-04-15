package frc.robot.utils.trajectory;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;

import java.util.List;

/**
 * Class representing list of Pose2d objects and associated information for
 * trajectory generation.
 */
public class CubicWaypoints {

    private final Pose2d start;
    private final List<Translation2d> interior;
    private final Pose2d end;
    private final TrajectoryConfig config;

    /**
     * Constructs a CubicWaypoints object.
     * @param start The Pose2d of the first point in the trajectory.
     * @param interior Translation2d objects defining interior waypoints to pass through.
     * @param end The Pose2d of the last point in the trajectory.
     * @param config The TrajectoryConfig of the path once it is generated.
     */
    public CubicWaypoints(Pose2d start, List<Translation2d> interior, Pose2d end, TrajectoryConfig config) {
        this.start = start;
        this.interior = interior;
        this.end = end;
        this.config = config;
    }

    /**
     * Returns the Pose2d object of the first position on the path.
     */
    public Pose2d getStart() {
        return start;
    }

    /**
     * Returns the list of Translation2d objects associated with the interior points of the path.
     */
    public List<Translation2d> getInterior() {
        return interior;
    }

    /**
     * Returns the Pose2d object of the last position on the path.
     */
    public Pose2d getEnd() {
        return end;
    }

    /**
     * Returns the config for the trajectory to be generated.
     */
    public TrajectoryConfig getConfig() {
        return config;
    }
}
