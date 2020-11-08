/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double ROBOT_MIN_VOLTS = 6.0;
    public static final double ROBOT_MID_VOLTS = 8.0;

    public static final double A_LENGTH = 0.59055; // Axel length in meters
    public static final double A_WIDTH = 0.48895; // Axel length in meters
    public static final double A_CROSSLENGTH = Math.sqrt(Math.pow(A_LENGTH, 2) + Math.pow(A_WIDTH, 2)); // Axel crosslength in meters

    public static final double FIVEFIFTY_MAX_TEMP = 50.0;
    public static final double FALCON_ENCODER_TICKS = 2048; //Counts per rotation of the Falcon 500 motor.
    public static final double FALCON_MAX_TEMP = 50.0; //Max temperature of Falcon 500 in degrees celsius.
    public static final double CANCODER_TICKS = 4096;
    
    public static final double DRIVE_GEARING = 0.0; //Gear ratio of the drive motor.
    public static final double WHEEL_DIAMETER = 4.0; //Diameter of the drive wheels.
    public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER; //Circumfrence of the drive wheels.
    public static final double DRIVE_ROTATIONS_PER_METER = DRIVE_GEARING / WHEEL_CIRCUMFRENCE; //Rotations per meter of the drive wheels.
    public static final double DRIVE_COUNTS_PER_METER = DRIVE_ROTATIONS_PER_METER *FALCON_ENCODER_TICKS; //Encoder ticks per meter of the drive wheels.

    public static final double ANGLE_GEARING = 0.0; //Gear ratio of the turning motor.
    public static final double ANGLE_RATIO = FALCON_ENCODER_TICKS * ANGLE_GEARING; //Encoder ticks in 360 degrees of turning.

    public static final double DRIVE_MAX_VOLTS = 12.0; //Maximum voltage allowed in the drivetrain.
    public static final double DRIVE_MAX_VELOCITY = 12.0; //Maximum velocity allowed in the drivetrain in meters per second.
    public static final double DRIVE_MAX_ACCEL = 8.0;
    public static final double DRIVE_MAX_CACCEL = 8.0;
    public static final double DRIVE_MAX_RADIANS = Math.PI; //Maximum rotation per second in radians.
    public static final double DRIVE_RAMP = 1.0; //Ramp rate of drivetrain motor power.
    public static final double DRIVEkP = 1.0; //P value for drivetrain pid loop.
    public static final double DRIVEkI = 0.0; //I value for the drivetrain pid loop.
    public static final double DRIVEkD = 0.0; //D value for the drivetrain pid loop.
    public static final double DRIVEkS = 0.0;
    public static final double DRIVEkV = 0.0;

    public static final double ANGLEkP = 1.0; //P value for the angle motor pid loop.
    public static final double ANGLEkI = 0.0; //I value for the angle motor pid loop.
    public static final double ANGLEkD = 0.0; //D value for the angle motor pid loop.

    public static final double STARTING_X = 0.0; //Starting X position of the robot.
    public static final double STARTING_Y = 0.0; //Starting Y position of the robot.

    public static final double GOAL_HEIGHT = 8.1875; //Height of the goal from the ground in feet.
    public static final double THREE_POINT_DEPTH = 2 + (5.25/12.0); //Depth of the 3 point goal inside the 2 point goal in feet.
    public static final double PORT_SIDE_LENGTH = 2.5 / (2 * Math.sin(60)); //Side length of the port in feet.
    public static final double OFFSET_LIMIT = Math.atan(PORT_SIDE_LENGTH / (THREE_POINT_DEPTH * 2)); //Limit for the skew against the 3-point goal where any farther would make the ball hit the wall.

    public static final double TURRET_LIMELIGHT_HEIGHT = 1.0; //Height fromt the ground of the limelight in feet.
    public static final double TURRET_MOUNTING_ANGLE = 0.0;

    public static final double TURRET_MAX_VELOCITY = 11000; // rpm
    public static final double TURRET_MAX_ACCELERATION = 22000; // rpm / sec
    public static final double TURRET_kP = 4e-5;
    public static final double TURRET_kI = 1e-6;
    public static final double TURRET_kD = 0;
    public static final double TURRET_kF = 1.0 / TURRET_MAX_VELOCITY;
    public static final double TURRET_kI_ZONE = 0;
    public static final double TURRET_GEAR_RATIO = 300; // angular velocity will be divided by this amount
    public static final double TURRET_ENCODER_TO_DEGREES = 360.0 / TURRET_GEAR_RATIO; // degrees
    public static final double TURRET_RAMP_RATE = 0.5; // seconds
    public static final double TURRET_MAX_ANG = 370;
    public static final double TURRET_MIN_ANG = -10;
}
