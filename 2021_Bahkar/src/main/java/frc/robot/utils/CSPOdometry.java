package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

public class CSPOdometry {

    private Integral x_integral;
    private Integral y_integral;

    public CSPOdometry(Pose2d start) {
        x_integral = new Integral(start.getX());
        y_integral = new Integral(start.getY());
    }

    public void update(Rotation2d gyro, ChassisSpeeds speeds) {
        double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double heading = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond) + gyro.getRadians();

        x_integral.sample(Math.cos(heading) * speed);
        y_integral.sample(Math.sin(heading) * speed);
    }

    public Pose2d getPose(Rotation2d gyro) {
        return new Pose2d(x_integral.getValue(), y_integral.getValue(), gyro);
    }
    
}
