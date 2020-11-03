package frc.robot.utils.trajectory;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import java.util.ArrayList;

public class TwoMeterTest {

    Trajectory trajectory;

    public TwoMeterTest(TrajectoryConfig config) {
        var start = new Pose2d(0.0, 0.0,
        Rotation2d.fromDegrees(0.0));

        var interiorWaypoints = new ArrayList<Translation2d>();

        var end = new Pose2d(2.0, 0.0,
        Rotation2d.fromDegrees(90.0));

        var trajectory = TrajectoryGenerator.generateTrajectory(
            start,
            interiorWaypoints,
            end,
            config);
        
        this.trajectory = trajectory;
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

}