package frc.robot.utils;

import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.Sensors;;
public class CSPMath {

    public static double remainder(double a, double b) {
        return Math.IEEEremainder(a, b);
    }

    public static double minChange(double a, double b, double wrap) {
        return remainder((a-b), wrap);
    }

    public static double minDist(double a, double b, double wrap) {
        return Math.abs(minChange(a, b, wrap));
    }
/**
     * Calculates the horizontal velocity component from range of a projectile formula
     * @param distanceToTarget
     * @param vy the vertical velocity
     * @return the horizontal velocity in ft/s
     */
    public double getVx(double distanceToTarget, double vy) {
        return ((2 * ACCEL_GRAVITY * (distanceToTarget + THREE_POINT_DEPTH)) / (vy + Math.sqrt(Math.pow(vy, 2) + (2 * ACCEL_GRAVITY * ROBOT_HEIGHT))));
    }

    /**
     * Calculates the vertical velocity component from maximum height of a projectile formula
     * @return the vertical velocity in ft/s
     */
    public double getVy() {
        return Math.sqrt(2 * ACCEL_GRAVITY * (GOAL_HEIGHT - ROBOT_HEIGHT));
    }

    /**
     * Calculates the launch angle from vertical and horizontal velocity formulas
     * @param vx the horizontal velocity
     * @param vy the vertical velocity
     * @return the launch angle in degrees
     */
    public double getLaunchAngle(double vx, double vy) {
        return Math.toDegrees(Math.atan(vy / vx));
    }

    /**
     * Calculates the velocity from 
     * @param vx the horizontal velocity
     * @param vy the vertical velocity
     * @param launchAngle the launch angle in degrees
     * @return the velocity in m/s or -1 if velocity equations are not equal
     */
    public double getVelocity(double vx, double vy, double launchAngle) {
        double result;

        if (Math.round(vy / Math.sin(Math.toRadians(launchAngle))) == Math.round((vx / Math.cos(Math.toRadians(launchAngle))))) {
            result = vy / (Math.sin(Math.toRadians(launchAngle)));
        } else
            result = -1;
        return result;
    }

    /**
     * Evaluates either parabola when given x
     * @param input x or horizontal distance
     * @param topParabola true for top parabola equation, false for bottom parabola equation
     * @return the ouput depending on either top or bottom parabola equation
     */
    public double projectileEquations(double input, boolean topParabola) {
        double vy = getVy();
        double vx = getVx(distanceToTarget, vy);
        double velocity = getVelocity(vx, vy, getLaunchAngle(vx, vy));
        double launchAngle = getLaunchAngle(vx, vy);

        double output = -1;

        double topParabolaOutput = (-(((0.5 * ACCEL_GRAVITY) * Math.pow(input, 2)) / (Math.pow(velocity, 2) * Math.pow(Math.cos(Math.toRadians(launchAngle)), 2))))
                + (Math.tan((Math.toRadians((launchAngle)))) * input) + (ROBOT_HEIGHT + (POWER_CELL_DIAMETER / 2));
        double bottomParabolaOutput = (-(((0.5 * ACCEL_GRAVITY) * Math.pow(input, 2)) / (Math.pow(velocity, 2) * Math.pow(Math.cos(Math.toRadians(launchAngle)), 2))))
                + (Math.tan((Math.toRadians((launchAngle)))) * input) + (ROBOT_HEIGHT - (POWER_CELL_DIAMETER / 2));

        if (topParabola) {
            output = topParabolaOutput;
        } else {
            output = bottomParabolaOutput;
        }
        return output;
    }

    /**
     * Tells whether it is possible to clear the outer port and make a three-point shot with distance to
     * target
     * @return true if it can make it, false if it cannot
     */
    public boolean trajectoryTest() {
        boolean isClearOuter = true;
        boolean isClearInner = true;

        if (projectileEquations(distanceToTarget, true) >= (GOAL_HEIGHT + (PORT_HEIGHT / 2))
                || projectileEquations(distanceToTarget, false) <= (GOAL_HEIGHT - (PORT_HEIGHT / 2))) {isClearOuter = false;}
        if (projectileEquations((distanceToTarget + THREE_POINT_DEPTH),true) >= (GOAL_HEIGHT + (INNER_PORT_DIAMETER / 2))
                || projectileEquations((distanceToTarget + THREE_POINT_DEPTH), false) <= (GOAL_HEIGHT - (INNER_PORT_DIAMETER / 2))) {isClearInner = false;}

        return isClearOuter && isClearInner;
    }
    public static void main(String[] args) {
        
    }
}