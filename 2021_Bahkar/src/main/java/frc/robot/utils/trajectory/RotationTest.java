package frc.robot.utils.trajectory;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.Iterator;

public class RotationTest {

    Trajectory trajectory;

    Iterator<Double> angIterator = null;
    Iterator<Double> timeIterator = null;

    public RotationTest(TrajectoryConfig config) {
        var start = new Pose2d(Constants.STARTING_X, Constants.STARTING_Y,
        Rotation2d.fromDegrees(0.0));

        var interiorWaypoints = new ArrayList<Translation2d>();

        var end = new Pose2d(0.0, 3.0,
        Rotation2d.fromDegrees(0.0));

        var trajectory = TrajectoryGenerator.generateTrajectory(
            start,
            interiorWaypoints,
            end,
            config);
        
        this.trajectory = trajectory;

        double totalTime = trajectory.getTotalTimeSeconds();

        ArrayList<Double> angles = new ArrayList<Double>();
        angles.add(0.0);
        angles.add(90.0);
        angles.add(180.0);
        angles.add(-90.0);
        angles.add(0.0);

        this.angIterator = angles.iterator();

        ArrayList<Double> times = new ArrayList<Double>();
        times.add(0.0);
        times.add((totalTime/4) - (90/180));
        times.add((2 * totalTime / 4) - (90/180));
        times.add(( 3 * totalTime / 4) - (90/180));
        times.add((4 * totalTime / 4) - (90/180));

        this.timeIterator = times.iterator();
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

    public Iterator<Double> getAngles() {
        return angIterator;
    }

    public Iterator<Double> getTimes() {
        return timeIterator;
    }
}