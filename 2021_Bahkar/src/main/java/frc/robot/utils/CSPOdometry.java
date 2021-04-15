package frc.robot.utils;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

/**
 * Class to track field position using distance traveled and heading.
 */
public class CSPOdometry {

    // Integral and derivative declarations.
    private Integral x_integral;
    private Integral y_integral;
    private Integral heading_integral;
    private Derivative heading_derivative;

    /**
     * Constructs a new CSPOdometry object.
     * @param start The initial position of the robot.
     */
    public CSPOdometry(Pose2d start) {
        x_integral = new Integral(start.getX());
        y_integral = new Integral(start.getY());
        heading_integral = new Integral(start.getRotation().getRadians());
        heading_derivative = new Derivative(start.getRotation().getDegrees());
    }

    /**
     * Update the odometry's estimated position.
     * @param gyro Rotation of the robot as recorded by the gyro.
     * @param speeds ChassisSpeeds of the robot.
     */
    public void update(Rotation2d gyro, ChassisSpeeds speeds) {

        // Find the rate of the gyro and if it's low enough, use the ChassisSpeed rotation rate and integrate.
        double rate = heading_derivative.getRate(gyro.getDegrees());
        heading_integral.sample(Math.abs(rate) < 1.0 ? speeds.omegaRadiansPerSecond : Math.toRadians(rate));

        // Calculate the speed and heading of the robot's motion based on ChassisSpeeds and gyro.
        double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double heading = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond) + normalizeAngle(heading_integral.getValue());

        // Update the integral objects with transformed headings.
        x_integral.sample(Math.cos(heading) * speed);
        y_integral.sample(Math.sin(heading) * speed);
    }

    /** Returns the integrated position of the robot and its heading. */
    public Pose2d getPose() {
        return new Pose2d(x_integral.getValue(), y_integral.getValue(), new Rotation2d(normalizeAngle(heading_integral.getValue())));
    }

    /** Set a new position for the robot. */
    public void setPose(Pose2d pose) {
        x_integral = new Integral(pose.getX());
        y_integral = new Integral(pose.getY());
        heading_integral = new Integral(pose.getRotation().getRadians());
    }
    
    /** Convert an angle which may be outside of [-180, 180] to its corresponding angle within [-180, 180]. */
    private static double normalizeAngle(double angle) {
        return (angle + 180.0) % 360 - 180;
    }
}
