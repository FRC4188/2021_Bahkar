package frc.robot.utils;

import frc.robot.Constants;

public final class CSPMath {

    public static double remainder(double a, double b) {
        return Math.IEEEremainder(a, b);
    }

    public static double minChange(double a, double b, double wrap) {
        return remainder((a-b), wrap);
    }

    public static double minDist(double a, double b, double wrap) {
        return Math.abs(minChange(a, b, wrap));
    }

    public static double findHypotenuse(double a, double b) {
        return Math.sqrt(Math.pow(a, 2.0) + Math.pow(b, 2.0));
    }

    public static class Shooter {

        public static double velToRPM(double velocity) {
            return (120.0 * velocity) / ((Constants.Shooter.MAIN_WHEEL_CIRCUMFERENCE / Constants.Shooter.MAIN_WHEEL_RATIO) + (Constants.Shooter.AUX_WHEEL_CIRCUMFERENCE / Constants.Shooter.AUX_WHEEL_RATIO));
        }

        public static double rpmToVel(double rpm) {
            return (rpm * ((Constants.Shooter.MAIN_WHEEL_CIRCUMFERENCE / Constants.Shooter.MAIN_WHEEL_RATIO) + (Constants.Shooter.AUX_WHEEL_CIRCUMFERENCE / Constants.Shooter.AUX_WHEEL_RATIO))) / 120.0;
        }

        public static double closeFormulaAngle(double distance) {
            return 6.57143 * Math.pow(distance - 2.40652, 2.0) + 55.6397;
        }

        public static double midFormulaAngle(double distance) {
            return 1.80714 * Math.pow(distance - 3.32036, 2.0) + 39.1587;
        }

        public static double farFormulaAngle(double distance) {
            return 0.707293 * Math.pow(distance - 5.80819, 2.0) + 26.8696;
        }
    }

    /**
     * The math class for the shooter's hood
     */
    public static class Hood {

        /**
         * Converts a release angle to the set position for the hood servos.
         * @param angle Angle to be converted. degrees.
         * @return Servo set position in range [0.0, 1.0].
         */
        public static double angleToSet(double angle) {
            return (-Constants.Hood.PIVOT_CIRCUMFERENCE * (angle - Constants.Hood.INITIAL_ANGLE)) / (360.0 * Constants.Hood.SERVO_STROKE);
        }

        /**
         * Converts a servo set position to a release angle.
         * @param position The set position of the hood servos in range [0.0, 1.0].
         * @return Release angle. degrees.
         */
        public static double setToAngle(double position) {
            return Constants.Hood.INITIAL_ANGLE - (360.0 * Constants.Hood.SERVO_STROKE * position) / Constants.Hood.PIVOT_CIRCUMFERENCE;
        }
    }
}