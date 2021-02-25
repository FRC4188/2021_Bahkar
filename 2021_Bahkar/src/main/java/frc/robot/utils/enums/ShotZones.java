package frc.robot.utils.enums;

import frc.robot.Constants;

public enum ShotZones {
    CLOSE(Constants.shooter.CLOSE_SHOOTING_RANGE, Constants.shooter.CLOSE_SHOOTING_VEL),
    MID(Constants.shooter.MID_SHOOTING_RANGE, Constants.shooter.MID_SHOOTING_VEL),
    FAR(Constants.shooter.FAR_SHOOTING_RANGE, Constants.shooter.FAR_SHOOTING_VEL),
    HAILMARY(Constants.shooter.HAIL_MARY_RANGE, Constants.shooter.HAIL_MARY_VEL),
    OUTSIDE(new double[]{Constants.shooter.MAX_DISTANCE, 99.0}, Constants.shooter.IDLE_VEL);

    private double[] range = new double[2];
    private double velocity;

    private ShotZones (double[] range, double velocity) {
        this.range = range;
        this.velocity = velocity;
    }

    public double getVelocity() {
        return velocity;
    }

    public boolean inRange(double distance) {
        return range[0] < distance && distance < range[1];
    }
}