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
    public final class physics {
        public static final double ACCEL_GRAVITY = 9.8;
    }

    public final class robot {
            public static final double MIN_VOLTS = 6.0; //Minimum available battery volts before things get really bad.
            public static final double MID_VOLTS = 8.0; //Battery voltage where we start getting a little bit worried.

            public static final double A_LENGTH = 0.59055; // Axel length (Meters).
            public static final double A_WIDTH = 0.48895; // Axel width (Meters).
            public final double A_CROSSLENGTH = Math.sqrt(Math.pow(A_LENGTH, 2.0) + Math.pow(A_WIDTH, 2.0));

            public static final double FIVEFIFTY_MAX_TEMP = 50.0; //Maximum preferred temperature of a neo550 (Celsius.)
            public static final double FALCON_ENCODER_TICKS = 2048.0; //Counts per revolution of the Falcon 500 motor.
            public static final double FALCON_MAX_TEMP = 50.0; //Max temperature of Falcon 500 (Celsius).
            public static final double FALCON_MAX_VEL = 6380.0;
        }

    public final class drive {
        public static final double DRIVE_GEARING = 6.0; //Gear ratio of the drive motor.
        public static final double WHEEL_DIAMETER = 0.1016;//Units.inchesToMeters(4); //Diameter of the drive wheels (Meters).
        public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER; //Circumfrence of the drive wheels (Meters).
        public static final double DRIVE_ROTATIONS_PER_METER = 1.0 / WHEEL_CIRCUMFRENCE; //Rotations per meter of the drive wheels.
        public static final double DRIVE_COUNTS_PER_ROTATION = DRIVE_GEARING * robot.FALCON_ENCODER_TICKS; //Encoder counts per revolution of the drive wheel.
        public static final double DRIVE_COUNTS_PER_METER = DRIVE_ROTATIONS_PER_METER * DRIVE_COUNTS_PER_ROTATION; //Encoder ticks per meter of the drive wheels.

        public static final double ANGLE_GEARING = 12.0;
        public static final double ANGLE_TICKS_PER_DEGREE = (ANGLE_GEARING * robot.FALCON_ENCODER_TICKS) / 360.0;

        public static final double MAX_VOLTS = 12.0; //Maximum voltage allowed in the drivetrain.
        public static final double MAX_VELOCITY = 5.0; //Maximum velocity allowed in the drivetrain (Meters per Second).
        public static final double MAX_ACCEL = 10.0; //Maximum acceleration of the drivetrain in (Meters per Second Squared).
        public static final double MAX_CACCEL = 8.0; //Maximum centripital acceleration of the robot (Meters per Second Squared).
        public static final double MAX_RADIANS = 3.0 * Math.PI; //Maximum rotational velocity (Radians per Second).

        public final class auto {
            public static final double MAX_VELOCITY = 1.0; //Maximum velocity allowed in the drivetrain (Meters per Second).
            public static final double MAX_ACCEL = 3.0; //Maximum acceleration of the drivetrain in (Meters per Second Squared).
            public static final double MAX_CACCEL = 5.0; //Maximum centripital acceleration of the robot (Meters per Second Squared).
        }
    }

    public final static class field {
        public static final double GOAL_HEIGHT = Units.feetToMeters(8.0 + 2.25/12.0); //Height of the goal from the ground (Meters).
        public static final double PORT_HEIGHT = 2.5; //feet
        public static final double THREE_POINT_DEPTH = Units.feetToMeters(2.0 + (5.25/12.0)); //Depth of the 3 point goal inside the 2 point goal (Meters).
        public static final double PORT_SIDE_LENGTH = Units.feetToMeters(PORT_HEIGHT / (2.0 * Math.sin(Math.toRadians(60.0)))); //Side length of the port (Meters).
        public static final double OFFSET_LIMIT = Math.toDegrees(Math.atan(PORT_SIDE_LENGTH / (THREE_POINT_DEPTH * 2.0))); //Limit for the skew against the 3-point goal (Degrees).
        public static final double POWER_CELL_DIAMETER = Units.feetToMeters(7.0 / 12.0); //Diameter of the power cell (Meters).
        public static final double INNER_PORT_DIAMETER = Units.feetToMeters(13.0 / 12.0); //Diameter of the inner port (Meters).
        public static final double GOAL_Y_POS = 2.38;
    }

    public final class turret{
        public static final double LIMELIGHT_HEIGHT = 1.0; //Height fromt the ground of the limelight (Meters).
        public static final double MOUNTING_ANGLE = 0.0; //Verticle angle which the limelight sits at on the turret (Degrees).

        public static final double MAX_VELOCITY = 11000.0; // rpm
        public static final double MAX_ACCELERATION = 22000.0; // rpm per sec

        public static final double GEAR_RATIO = 300.0; // angular velocity will be divided by this amount
        public static final double ENCODER_TO_DEGREES = 360.0 / GEAR_RATIO; // degrees

        public static final double kP = 2.4e-2;
        public static final double kI = 3.0e-3;
        public static final double kD = 1.0e-2;

        public static final double POS_TOLERANCE = 0.1; // degrees
        public static final double VEL_TOLERANCE = 2.0; // degrees per second

        public static final double MAX_ANG = 370.0; //Farthest bound of the turret's roation in degrees.
        public static final double MIN_ANG = -10.0; //Other farthest bound of the turret's rotation in degrees.
    }

    public static final class shooter {
        public static final double MAX_VELOCITY = 5750.0;
        public static final double IDLE_VEL = 3500.0;

        public static final double MAX_DISTANCE = 12.2;

        public static final double HAIL_MARY_VEL = 5750.0;
        public static final double[] HAIL_MARY_RANGE = {8.5, MAX_DISTANCE};
        public static final double FAR_SHOOTING_VEL = 5000.0;
        public static final double[] FAR_SHOOTING_RANGE = {5.0, 9.0};
        public static final double MID_SHOOTING_VEL = 3750.0; 
        public static final double[] MID_SHOOTING_RANGE = {2.0, 5.5}; // Shoot with the mid velocity if the distance is between this and the short distance. in meters.
        public static final double CLOSE_SHOOTING_VEL = 3000.0;
        public static final double[] CLOSE_SHOOTING_RANGE = {0.5, 2.5}; // Shoot with the short velocity if the distance is less than this. In meters.

        public static final double SHOOTING_TOLERANCE = 250.0;

        public static final double MAIN_WHEEL_RADIUS = Units.inchesToMeters(2.0);
        public static final double MAIN_WHEEL_CIRCUMFERENCE = Math.PI * Math.pow(MAIN_WHEEL_RADIUS, 2.0);
        public static final double MAIN_WHEEL_RATIO = 1.0;
        public static final double AUX_WHEEL_RADIUS = Units.inchesToMeters(0.75);
        public static final double AUX_WHEEL_CIRCUMFERENCE = Math.PI * Math.pow(AUX_WHEEL_RADIUS, 2.0);
        public static final double AUX_WHEEL_RATIO = 4.0 / 3.0;

        public static final double kP = 0.325;
        public static final double kI = 0.0;
        public static final double kD = 0.55;
        public static final double kF = 1023.0 / ((robot.FALCON_MAX_VEL / 600.0) * robot.FALCON_ENCODER_TICKS);

        public static final double RAMP_RATE = 0.75;
        public static final double SHOOTER_HEIGHT = Units.feetToMeters(2.0);
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    }

    public static final class hood {
        public static final double SERVO_STROKE = 100.0; // milimeters
        public static final double AXEL_TO_SERVO = Math.cos(Math.toDegrees(16.7)) * 117.2972; // The length from the hood's pivot to the where the servo standoff is part of the line perpendicular to the arm. in milimeters.
        public static final double PIVOT_CIRCUMFERENCE = 2 * Math.PI * AXEL_TO_SERVO;
        public static final double INITIAL_ANGLE = 77.473085; // The angle the shooter starts at (Degrees).
        public static final double MIN_ANGLE = 26.475460; // The minimum angle the shooter can point to (Degrees).
    }

    public final class intake {
        public static final double FLOP_RATE = 0.5;
        public static final double RAMP_RATE = 0.5;
    }

    public final class lcdpanel {
        public static final int CLEAR_DISPLAY = 0x01;
        public static final int RETURN_HOME = 0x02;
        public static final int ENTRY_MODE_SET = 0x04;
        public static final int DISPLAY_CONTROL = 0x08;
        public static final int CURSOR_SHIFT = 0x10;
        public static final int FUNCTION_SET = 0x20;
        public static final int SETCGRAMADDR = 0x40;
        public static final int SETDDRAMADDR = 0x80;

        public static final int DISPLAY_ON = 0x04;
        public static final int DISPLAY_OFF = 0x00;
        public static final int CURSOR_ON = 0x02;
        public static final int CURSOR_OFF = 0x00;
        public static final int BLINK_ON = 0x01;
        public static final int BLINK_OFF = 0x00;

        // public static final int ENTRYRIGHT = 0x00;
        public static final int ENTRY_LEFT = 0x02;
        public static final int ENTRY_SHIFT_INCREMENT = 0x01;
        public static final int ENTRY_SHIFT_DECREMENT = 0x00;

        public static final int DISPLAY_MOVE = 0x08;
        public static final int CURSOR_MOVE = 0x00;
        public static final int MOVE_RIGHT = 0x04;
        public static final int MOVE_LEFT = 0x00;

        public static final int EIGHT_BIT_MODE = 0x10;
        public static final int FOUR_BIT_MODE = 0x00;
        public static final int TWO_LINE = 0x08;	//for 2 or 4 lines actually
        public static final int ONE_LINE = 0x00;
        public static final int FIVExTEN_DOTS = 0x04;	//seldom used!!
        public static final int FIVExEIGHT_DOTS = 0x00;	

        public static final int BACKLIGHT_ON = 0x08;
        public static final int BACKLIGHT_OFF = 0x00;

        public static final int ENABLE = 0b00000100; 
        public static final int READ_WRITE = 0b00000010;
        public static final int REGISTER_SELECT = 0b00000001; 
    }
}
