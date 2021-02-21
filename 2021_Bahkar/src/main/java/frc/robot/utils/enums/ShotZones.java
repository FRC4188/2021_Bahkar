package frc.robot.utils.enums;

import frc.robot.Constants;

public enum ShotZones {
    CLOSE(Constants.Shooter.CLOSE_SHOOTING_RANGE, Constants.Shooter.CLOSE_SHOOTING_VEL),
    MID(Constants.Shooter.MID_SHOOTING_RANGE, Constants.Shooter.MID_SHOOTING_VEL),
    FAR(Constants.Shooter.FAR_SHOOTING_RANGE, Constants.Shooter.FAR_SHOOTING_VEL),
    HAILMARY(Constants.Shooter.HAIL_MARY_RANGE, Constants.Shooter.HAIL_MARY_VEL),
    OUTSIDE(new double[]{Constants.Shooter.MAX_DISTANCE, 99.0}, Constants.Shooter.IDLE_VEL);

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