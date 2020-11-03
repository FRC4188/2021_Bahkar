package frc.robot.utils.trajectory;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import java.util.ArrayList;

public class SpontaneousTrajectory {

    Trajectory trajectory;

    public SpontaneousTrajectory(Pose2d startpose, TrajectoryConfig config) {
        var interiorWaypoints = new ArrayList<Translation2d>();

        double X = startpose.getTranslation().getX();
        double Y = startpose.getTranslation().getY();

        if (X < 3.1) {
            if (Y > 4.6 && Y < 9.55) {
                interiorWaypoints.add(new Translation2d(3.1, 4.25));
                interiorWaypoints.add(new Translation2d(7.51, 6.0));
                interiorWaypoints.add(new Translation2d(7.51, 6.5));
                interiorWaypoints.add(new Translation2d(7.51, 8.0));
            } else if (Y <= 4.6) {
                interiorWaypoints.add(new Translation2d(7.51, 6.0));
                interiorWaypoints.add(new Translation2d(7.51, 6.5));
                interiorWaypoints.add(new Translation2d(7.51, 8.0));
            }
        } else {
            if (Y < 7.2) {
                interiorWaypoints.add(new Translation2d(7.51, 6.0));
                interiorWaypoints.add(new Translation2d(7.51, 6.5));
                interiorWaypoints.add(new Translation2d(7.51, 8.0));
            }
        }

        var end = new Pose2d(5.8, 12.8,
        Rotation2d.fromDegrees(0.0));

        var trajectory = TrajectoryGenerator.generateTrajectory(
            startpose,
            interiorWaypoints,
            end,
            config);
        
        this.trajectory = trajectory;
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

}