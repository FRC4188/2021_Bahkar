/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class Robot {
            public static final double MIN_VOLTS = 6.0; //Minimum available battery volts before things get really bad.
            public static final double MID_VOLTS = 8.0; //Battery voltage where we start getting a little bit worried.

            public static final double A_LENGTH = 0.59055; // Axel length (Meters).
            public static final double A_WIDTH = 0.48895; // Axel width (Meters).
            public final double A_CROSSLENGTH = Math.sqrt(Math.pow(A_LENGTH, 2) + Math.pow(A_WIDTH, 2));

            public static final double FIVEFIFTY_MAX_TEMP = 50.0; //Maximum preferred temperature of a neo550 (Celsius.)
            public static final double FALCON_ENCODER_TICKS = 2048; //Counts per revolution of the Falcon 500 motor.
            public static final double FALCON_MAX_TEMP = 50.0; //Max temperature of Falcon 500 (Celsius).
        }

        public final class Drive {
        public static final double DRIVE_GEARING = 6.54545454545; //Gear ratio of the drive motor.
        public static final double WHEEL_DIAMETER = 0.1016;//Units.inchesToMeters(4); //Diameter of the drive wheels (Meters).
        public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER; //Circumfrence of the drive wheels (Meters).
        public static final double DRIVE_ROTATIONS_PER_METER = 1.0 / WHEEL_CIRCUMFRENCE; //Rotations per meter of the drive wheels.
        public static final double DRIVE_COUNTS_PER_ROTATION = DRIVE_GEARING * Robot.FALCON_ENCODER_TICKS; //Encoder counts per revolution of the drive wheel.
        public static final double DRIVE_COUNTS_PER_METER = DRIVE_ROTATIONS_PER_METER * DRIVE_COUNTS_PER_ROTATION; //Encoder ticks per meter of the drive wheels.

        public static final double ANGLE_GEARING = 12.0;
        public static final double ANGLE_TICKS_PER_DEGREE = (ANGLE_GEARING * Robot.FALCON_ENCODER_TICKS) / 360;

        public static final double MAX_VOLTS = 12.0; //Maximum voltage allowed in the drivetrain.
        public static final double MAX_VELOCITY = 5.0; //Maximum velocity allowed in the drivetrain (Meters per Second).
        public static final double MAX_ACCEL = 10.0; //Maximum acceleration of the drivetrain in (Meters per Second Squared).
        public static final double MAX_CACCEL = 8.0; //Maximum centripital acceleration of the robot (Meters per Second Squared).
        public static final double MAX_RADIANS = 3 * Math.PI; //Maximum rotational velocity (Radians per Second).

        public final class Auto {
            public static final double MAX_VELOCITY = 1.0; //Maximum velocity allowed in the drivetrain (Meters per Second).
            public static final double MAX_ACCEL = 3.0; //Maximum acceleration of the drivetrain in (Meters per Second Squared).
            public static final double MAX_CACCEL = 5.0; //Maximum centripital acceleration of the robot (Meters per Second Squared).
        }
    }

    public final static class Field {
        public final static double GOAL_HEIGHT = Units.feetToMeters(8.1875); //Height of the goal from the ground (Meters).
        public final static double THREE_POINT_DEPTH = Units.feetToMeters(2 + (5.25/12.0)); //Depth of the 3 point goal inside the 2 point goal (Meters).
        public final static double PORT_SIDE_LENGTH = Units.feetToMeters(2.5 / (2 * Math.sin(Math.toRadians(60)))); //Side length of the port (Meters).
        public final static double OFFSET_LIMIT = Math.toDegrees(Math.atan(PORT_SIDE_LENGTH / (THREE_POINT_DEPTH * 2))); //Limit for the skew against the 3-point goal (Degrees).
    }

    public final class Turret{
        public static final double LIMELIGHT_HEIGHT = 1.0; //Height fromt the ground of the limelight (Meters).
        public static final double MOUNTING_ANGLE = 0.0; //Verticle angle which the limelight sits at on the turret (Degrees).

        public static final double MAX_VELOCITY = 11000; // rpm
        public static final double MAX_ACCELERATION = 22000; // rpm per sec

        public static final double GEAR_RATIO = 300; // angular velocity will be divided by this amount
        public static final double ENCODER_TO_DEGREES = 360.0 / GEAR_RATIO; // degrees

        public static final double kP = 2.4e-2;
        public static final double kI = 3e-3;
        public static final double kD = 1e-2;

        public static final double POS_TOLERANCE = 0.1; // degrees
        public static final double VEL_TOLERANCE = 2.0; // degrees per second

        public static final double MAX_ANG = 370; //Farthest bound of the turret's roation in degrees.
        public static final double MIN_ANG = -10; //Other farthest bound of the turret's rotation in degrees.
    }

    public final class Shooter {
        public static final double MAX_VELOCITY = 6000;
        public static final double IDLE_VEL = 3500;
        public static final double SHOOTING_VEL = 4500;
        public static final double SHOOTING_TOLERANCE = 250;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 1023 / ((MAX_VELOCITY * Robot.FALCON_ENCODER_TICKS) / 600);

        public static final double RAMP_RATE = 1.5;
    }

    public final class Intake {
        public static final double FLOP_RATE = 0.5;
    }
}
