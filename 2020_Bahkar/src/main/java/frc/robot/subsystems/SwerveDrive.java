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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.WheelDrive;

public class SwerveDrive extends SubsystemBase {

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

  //Initialize WheelDrive objects for each module to make setting things easier.
  WheelDrive LeftFront = new WheelDrive(LFAngleMotor, LFSpeedMotor);
  WheelDrive RightFront = new WheelDrive(RFAngleMotor, RFSpeedMotor);
  WheelDrive LeftRear = new WheelDrive(LRAngleMotor, LRSpeedMotor);
  WheelDrive RightRear = new WheelDrive(RRAngleMotor, RRSpeedMotor);

  /**
   * Creates a new SwerveDrive.
   */
  public SwerveDrive() {
    //Call configSensors method.
    configSensors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
   * Method to drive based on controller input.
   * @param x1 Right hand X axis input.
   * @param y1 Right hand Y axis input.
   * @param x2 Left hand X axis input.
   */
  public void drive(double x1,double  y1,double x2) {
    y1 *= -1;

    //Start a basic kinematics calculation
    double a = x1 - x2 * (Constants.A_LENGTH / Constants.A_CROSSLENGTH);
    double b = x1 - x2 * (Constants.A_LENGTH / Constants.A_CROSSLENGTH);
    double c = y1 - x2 * (Constants.A_WIDTH / Constants.A_CROSSLENGTH);
    double d = y1 - x2 * (Constants.A_WIDTH / Constants.A_CROSSLENGTH);

    //Finish the basic kinematics calculation
    double RightRearVel = Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2));
    double LeftRearVel = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2));
    double RightFrontVel = Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2));
    double LeftFrontVel = Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2));
    double RightRearAngle = Math.atan2(a, d) / Math.PI;
    double LeftRearAngle = Math.atan2(a, c) / Math.PI;
    double RightFrontAngle = Math.atan2(b, d) / Math.PI;
    double LeftFrontAngle = Math.atan2(b, c) / Math.PI;

    //Use the WheelDrive objects to set the motors to the kinematic's results.
    LeftFront.drive(LeftFrontVel, LeftFrontAngle);
    RightFront.drive(RightFrontVel, RightFrontAngle);
    LeftRear.drive(LeftRearVel, LeftRearAngle);
    RightRear.drive(RightRearVel, RightRearAngle);
  }

  /**
   * Method for Field Oriented driving with controller input.
   * @param x1 Right hand X value.
   * @param y1 Right hand Y value.
   * @param angle Left hand angle.
   */
  public void FODrive(double x1, double  y1, double angle) {
    //Change angle based on robot angle and convert that to -1 to 1.
    angle = ((angle - getGyro()) / 180.0) -1.0;

    //Adjust x and y inputs for forward and strafe 
    double x = Math.cos(Math.atan(y1 / x1) + Math.toRadians(getGyro()));
    double y = Math.sin(Math.atan(y1 / x1) + Math.toRadians(getGyro()));

    //Send adjusted inputs to drive method.
    drive(x, y, angle);
  }
  
  /**
   * Returns current gyro angle.
   * @return Gyro angle as a double.
   */
  public double getGyro() {
    return gyro.getAngle();
  }
}
