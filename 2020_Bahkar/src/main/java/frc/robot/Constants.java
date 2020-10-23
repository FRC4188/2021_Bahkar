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

    public final double A_LENGTH = 0.59055; // Axel length in meters
    public final double A_WIDTH = 0.48895; // Axel length in meters
    public final double A_CROSSLENGTH = Math.sqrt(Math.pow(A_LENGTH, 2) + Math.pow(A_WIDTH, 2)); // Axel crosslength in meters

    public final double FALCON_ENCODER_TICKS = 0;
    
    public final double DRIVE_GEARING = 0;
    public final double WHEEL_DIAMETER = 0;
    public final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER;
    public final double DRIVE_METERS_PER_ROTATION = DRIVE_GEARING * WHEEL_CIRCUMFRENCE;

    public final double ANGLE_GEARING = 0;
    public final double ANGLE_RATIO = FALCON_ENCODER_TICKS * ANGLE_GEARING;

    public final double DRIVE_MAX_VOLTS = 12; //Maximum voltage allowed in the drivetrain
    public final double DRIVEkP = 1; //P value for drivetrain pid loop
    public final double DRIVEkI = 0; //I value for the drivetrain pid loop
    public final double DRIVEkD = 0; //D value for the drivetrain pid loop

}
