package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

public final class Constants {

    public static final class Physics {
        public static final double ACCEL_GRAVITY = 9.81; //Meters per second.
    }

    public static final class RobotSpecs {
        public static final double MIN_VOLTS = 6.0; //Minimum available battery volts before things get really bad.
        public static final double MID_VOLTS = 8.0; //Battery voltage where we start getting a little bit worried.

        public static final double A_LENGTH = 0.59055; // Axel length (Meters).
        public static final double A_WIDTH = 0.48895; // Axel width (Meters).
        public static final double A_CROSSLENGTH = Math.sqrt(Math.pow(A_LENGTH, 2) + Math.pow(A_WIDTH, 2)); // Axel crosslength (Meters).

        public static final double FIVEFIFTY_MAX_TEMP = 50.0; //Maximum preferred temperature of a neo550 (Celsius.)
        public static final double FALCON_ENCODER_TICKS = 2048; //Counts per revolution of the Falcon 500 motor.
        public static final double FALCON_MAX_TEMP = 50.0; //Max temperature of Falcon 500 (Celsius).
    }

    public static final class Field {
        public static final double GOAL_HEIGHT = Units.feetToMeters(8.1875); //Height of the goal from the ground (Meters).
        public static final double THREE_POINT_DEPTH = Units.feetToMeters(2 + (5.25/12.0)); //Depth of the 3 point goal inside the 2 point goal (Meters).
        public static final double PORT_HEIGHT = 2.5; //ft
        public static final double PORT_SIDE_LENGTH = Units.feetToMeters(PORT_HEIGHT / (2 * Math.sin(Math.toRadians(60)))); //Side length of the port (Meters).
        public static final double OFFSET_LIMIT = Math.toDegrees(Math.atan(PORT_SIDE_LENGTH / (THREE_POINT_DEPTH * 2))); //Limit for the skew against the 3-point goal (Degrees).
        public static final double POWER_CELL_DIAMETER = Units.feetToMeters(7.0 / 12.0); //Diameter of the power cell (Meters).
        public static final double INNER_PORT_DIAMETER = Units.feetToMeters(13.0 / 12.0); //Diameter of the inner port (Meters).
    }

    public static final class Drive {
        public static final double GEARING = 6.54545454545; //Gear ratio of the drive motor.
        public static final double WHEEL_DIAMETER = 0.1016;//Units.inchesToMeters(4); //Diameter of the drive wheels (Meters).
        public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER; //Circumfrence of the drive wheels (Meters).
        public static final double ROTATIONS_PER_METER = 1.0 / WHEEL_CIRCUMFRENCE; //Rotations per meter of the drive wheels.
        public static final double COUNTS_PER_ROTATION = GEARING * RobotSpecs.FALCON_ENCODER_TICKS; //Encoder counts per revolution of the drive wheel.
        public static final double COUNTS_PER_METER = ROTATIONS_PER_METER * COUNTS_PER_ROTATION; //Encoder ticks per meter of the drive wheels.

        public static final double ANGLE_GEARING = 12.0;
        public static final double ANGLE_TICKS_PER_DEGREE = (ANGLE_GEARING * RobotSpecs.FALCON_ENCODER_TICKS) / 360;

        public static final double MAX_VOLTS = 12.0; //Maximum voltage allowed in the drivetrain.
        public static final double MAX_VELOCITY = 5.0; //Maximum velocity allowed in the drivetrain (Meters per Second).
        public static final double MAX_ACCEL = 10.0; //Maximum acceleration of the drivetrain in (Meters per Second Squared).
        public static final double MAX_CACCEL = 8.0; //Maximum centripital acceleration of the robot (Meters per Second Squared).
        public static final double MAX_RADIANS = 3 * Math.PI; //Maximum rotational velocity (Radians per Second).
    }

    public static final class Turret {
        public static final double LIMELIGHT_HEIGHT = 1.0; //Height fromt the ground of the limelight (Meters).
        public static final double MOUNTING_ANGLE = 0.0; //Verticle angle which the limelight sits at on the turret (Degrees).

        public static final double MAX_VELOCITY = 11000; // rpm
        public static final double MAX_ACCELERATION = 22000; // rpm / sec
        public static final double GEAR_RATIO = 300; // angular velocity will be divided by this amount
        public static final double ENCODER_TO_DEGREES = 360.0 / GEAR_RATIO; // degrees
        public static final double MAX_ANG = 370; //Farthest bound of the turret's roation in degrees.
        public static final double MIN_ANG = -10; //Other farthers bound of the turret's rotation in degrees.
    }

    public static final class Shooter {
        public static final double SHOOTER_HEIGHT = Units.feetToMeters(1.5);
        public static final double WHEEL_DIAMETER = Units.feetToMeters(4.0 / 12);   

        public static final double kP = 0.3;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double MAX_VELOCITY = 21300.0;
        public static final double RAMP_RATE = 0.25;
    }
}
