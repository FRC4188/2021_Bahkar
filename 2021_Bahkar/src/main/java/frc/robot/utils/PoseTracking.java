package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Turret;

public class PoseTracking {
    Pose2d pose;
    Drivetrain drivetrain;
    Sensors sensors;
    Turret turret;

    //Create initial odometry
    private SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(new Rotation2d(), new Pose2d(), Constants.drive.KINEMATICS,
    VecBuilder.fill(0.5, 0.5, 0.5),
    VecBuilder.fill(0.2),
    VecBuilder.fill(0.05, 0.05, 0.05));
    
    public PoseTracking(Drivetrain drivetrain, Turret turret, Sensors sensors, Pose2d initPose) {
        setPose(pose);

        this.drivetrain = drivetrain;
        this.sensors = sensors;
        this.turret = turret;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public void update() {
        odometry.update(sensors.getRotation2d(), drivetrain.getModuleStates());
        if(sensors.getTurretHasTarget()) odometry.addVisionMeasurement(sensors.getVisionPose(turret.getPosition()), Timer.getFPGATimestamp() - 0.02);
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }
}