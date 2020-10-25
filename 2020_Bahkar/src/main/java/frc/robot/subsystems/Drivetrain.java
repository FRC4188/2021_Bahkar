/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.CspController;
import frc.robot.utils.WheelDrive;

public class Drivetrain extends SubsystemBase {

  // device initialization
  private final TalonFX LFAngleMotor = new TalonFX(1);
  private final TalonFX LFSpeedMotor = new TalonFX(2);
  private final TalonFX RFAngleMotor = new TalonFX(3);
  private final TalonFX RFSpeedMotor = new TalonFX(4);
  private final TalonFX LRAngleMotor = new TalonFX(5);
  private final TalonFX LRSpeedMotor = new TalonFX(6);
  private final TalonFX RRAngleMotor = new TalonFX(7);
  private final TalonFX RRSpeedMotor = new TalonFX(8);
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  //Initialize WheelDrive objects
  WheelDrive LeftFront = new WheelDrive(LFAngleMotor, LFSpeedMotor);
  WheelDrive RightFront = new WheelDrive(RFAngleMotor, RFSpeedMotor);
  WheelDrive LeftRear = new WheelDrive(LRAngleMotor, LRSpeedMotor);
  WheelDrive RightRear = new WheelDrive(RRAngleMotor, RRSpeedMotor);

  //Put together swerve module positions relative to the center of the robot.
  Translation2d FrontLeftLocation = new Translation2d((Constants.A_LENGTH/2), (Constants.A_WIDTH/2));
  Translation2d FrontRightLocation = new Translation2d((Constants.A_LENGTH/2), -(Constants.A_WIDTH/2));
  Translation2d BackLeftLocation = new Translation2d(-(Constants.A_LENGTH/2), (Constants.A_WIDTH/2));
  Translation2d BackRightLocation = new Translation2d(-(Constants.A_LENGTH/2), -(Constants.A_WIDTH/2));

  //Create a kinematics withe the swerve module positions
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    FrontLeftLocation, FrontRightLocation, BackLeftLocation, BackRightLocation
  );

  //Initialize a ChassisSpeeds object and start it with default values
  ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(getGyro()));

  //Initialize a list of module states and assign the kinematic results to them
  SwerveModuleState[]  moduleStates = kinematics.toSwerveModuleStates(speeds);

  //Assign module states to modules
  SwerveModuleState frontLeft = moduleStates[0];
  SwerveModuleState frontRight = moduleStates[1];
  SwerveModuleState backLeft = moduleStates[2];
  SwerveModuleState backRight = moduleStates[3];

  //Create initial odometry
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics,
  Rotation2d.fromDegrees(getGyro()), new Pose2d(Constants.STARTING_Y, Constants.STARTING_X, new Rotation2d()));

  //Store odometry as a position on the field.
  Pose2d Position = odometry.update(Rotation2d.fromDegrees(getGyro()), frontLeft, frontRight, backLeft, backRight);

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    configSensors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    updateShuffleboard();
  }

  /**
   * Set properties of the motors.
   */
  private void configSensors() {
    //Select sensor for motors (integrated sensor).
    LFAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    LFSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    RFAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    RFSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    LRAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    LRSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    RRAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    RRSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    //Call PIDConfig method for each motor and set ramp rates.
    DrivePIDConfig(LFAngleMotor);
    AnglePIDConfig(LFSpeedMotor);
    LFSpeedMotor.configClosedloopRamp(Constants.DRIVE_RAMP);
    DrivePIDConfig(RFAngleMotor);
    AnglePIDConfig(RFSpeedMotor);
    RFSpeedMotor.configClosedloopRamp(Constants.DRIVE_RAMP);
    DrivePIDConfig(LRAngleMotor);
    AnglePIDConfig(LRSpeedMotor);
    LRSpeedMotor.configClosedloopRamp(Constants.DRIVE_RAMP);
    DrivePIDConfig(RRAngleMotor);
    AnglePIDConfig(RRSpeedMotor);
    RRSpeedMotor.configClosedloopRamp(Constants.DRIVE_RAMP);

    //Call reset methods.
    resetEncoders();
    resetGyro();
  }
  
  /**
   * Reset all encoders to position 0.
   */
  public void resetEncoders() {
    //Set encoder positions to 0
    LFAngleMotor.setSelectedSensorPosition(0);
    LFSpeedMotor.setSelectedSensorPosition(0);
    RFAngleMotor.setSelectedSensorPosition(0);
    RFSpeedMotor.setSelectedSensorPosition(0);
    LRAngleMotor.setSelectedSensorPosition(0);
    LRSpeedMotor.setSelectedSensorPosition(0);
    RRAngleMotor.setSelectedSensorPosition(0);
    RRSpeedMotor.setSelectedSensorPosition(0);
  }

  /**
   * Method to reset gyro to 0.
   */
  public void resetGyro() {
    //Reset the gyro.
    gyro.reset();
  }

  /**
   * Method to set P, I, and D terms for drive motor closed loop control.
   * @param motor Motor which is being configured.
   */
  public void DrivePIDConfig(TalonFX motor) {
    //Assign the values.
    motor.config_kP(0, Constants.DRIVEkP, 10);
    motor.config_kI(0, Constants.DRIVEkI, 10);
    motor.config_kD(0, Constants.DRIVEkD, 10);
  }

  /**
   * Set P, I, and D terms for Angle motor.
   * @param motor Motor which is being configured.
   */
  public void AnglePIDConfig(TalonFX motor) {
    //Assign the values.
    motor.config_kP(0, Constants.ANGLEkP, 10);
    motor.config_kI(0, Constants.ANGLEkI, 10);
    motor.config_kD(0, Constants.ANGLEkD, 10);
  }

  /**
   * Method for field oriented drive using kinematics
   * @param pilot CspController of the pilot
   */
  public void drive (CspController pilot) {
    //Convert controller input to M/S and Rad/S
    double Speed = (pilot.getY(Hand.kRight)) * Constants.DRIVE_MAX_VELOCITY;
    double Strafe = (pilot.getX(Hand.kRight)) * Constants.DRIVE_MAX_VELOCITY;
    double Rotation = (pilot.getX(Hand.kLeft)) * Constants.DRIVE_MAX_RADIANS;

    //Get a chassis speed and rotation from input.
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(Speed, Strafe, Rotation, Rotation2d.fromDegrees(getGyro()));

    //Fill the module states list with the new module states
    moduleStates = kinematics.toSwerveModuleStates(speeds);

    //Assign members of the module states list to variables
    frontLeft = moduleStates[0];
    frontRight = moduleStates[1];
    backLeft = moduleStates[2];
    backRight = moduleStates[3];

    //Using the WheelDrive objects, power the motors with the correct instructions.
    LeftFront.convertedDrive(frontLeft.speedMetersPerSecond, frontLeft.angle.getDegrees());
    RightFront.convertedDrive(frontRight.speedMetersPerSecond, frontRight.angle.getDegrees());
    LeftRear.convertedDrive(backLeft.speedMetersPerSecond,  backLeft.angle.getDegrees());
    RightRear.convertedDrive(backRight.speedMetersPerSecond, backLeft.angle.getDegrees());
  }

  /**
   * Method to update the odometry of the robot.
   */
  private void updateOdometry() {
    //Update odometry and assign it to the Position variable.
    Position = odometry.update(Rotation2d.fromDegrees(getGyro()), frontLeft, frontRight, backLeft, backRight);
  }

  /**
   * Publish value from the drivetrain to the Smart Dashboard.
   */
  private void updateShuffleboard() {
    SmartDashboard.putString("Odometry", Position.toString());
    SmartDashboard.putNumber("Left Front Drive temp.", getFrontLeftDriveTemp());
    SmartDashboard.putNumber("Left Front Angle temp.", getFrontLeftAngleTemp());
    SmartDashboard.putNumber("Right Front Drive temp.", getFrontRightDriveTemp());
    SmartDashboard.putNumber("Right Front Angle temp.", getFrontRightAngleTemp());
    SmartDashboard.putNumber("Left Rear Drive temp.", getRearLeftDriveTemp());
    SmartDashboard.putNumber("Left Rear Angle temp.", getRearLeftAngleTemp());
    SmartDashboard.putNumber("Right Rear Drive temp.", getRearRightDriveTemp());
    SmartDashboard.putNumber("Right Rear Angle temp.", getRearRightAngleTemp());
  }

  /**
   * Method to get Gyro position
   * @return Gyro angle in degrees as a double
   */
  public double getGyro() {
    // get the angle from the gyro object
    return gyro.getAngle();
  }

  public double getFrontLeftDriveTemp() {
    return LFSpeedMotor.getTemperature();
  }
  public double getFrontLeftAngleTemp() {
    return LFAngleMotor.getTemperature();
  }
  public double getFrontRightDriveTemp() {
    return RFSpeedMotor.getTemperature();
  }
  public double getFrontRightAngleTemp() {
    return RFAngleMotor.getTemperature();
  }
  public double getRearLeftDriveTemp() {
    return LRSpeedMotor.getTemperature();
  }
  public double getRearLeftAngleTemp() {
    return LRAngleMotor.getTemperature();
  }
  public double getRearRightDriveTemp() {
    return RRSpeedMotor.getTemperature();
  }
  public double getRearRightAngleTemp() {
    return RRAngleMotor.getTemperature();
  }
}