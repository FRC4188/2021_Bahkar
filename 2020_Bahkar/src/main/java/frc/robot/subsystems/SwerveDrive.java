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

  WheelDrive LeftFront = new WheelDrive(LFAngleMotor, LFSpeedMotor);
  WheelDrive RightFront = new WheelDrive(RFAngleMotor, RFSpeedMotor);
  WheelDrive LeftRear = new WheelDrive(LRAngleMotor, LRSpeedMotor);
  WheelDrive RightRear = new WheelDrive(RRAngleMotor, RRSpeedMotor);

  /**
   * Creates a new SwerveDrive.
   */
  public SwerveDrive() {
    configSensors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void configSensors() {
    LFAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    LFSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    RFAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    RFSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    LRAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    LRSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    RRAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    RRSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    PIDConfig(LFAngleMotor);
    PIDConfig(LFSpeedMotor);
    LFSpeedMotor.configClosedloopRamp(Constants.DRIVE_RAMP);
    PIDConfig(RFAngleMotor);
    PIDConfig(RFSpeedMotor);
    RFSpeedMotor.configClosedloopRamp(Constants.DRIVE_RAMP);
    PIDConfig(LRAngleMotor);
    PIDConfig(LRSpeedMotor);
    LRSpeedMotor.configClosedloopRamp(Constants.DRIVE_RAMP);
    PIDConfig(RRAngleMotor);
    PIDConfig(RRSpeedMotor);
    RRSpeedMotor.configClosedloopRamp(Constants.DRIVE_RAMP);

    resetEncoders();
    resetGyro();
  }

  public void resetEncoders() {
    LFAngleMotor.setSelectedSensorPosition(0);
    LFSpeedMotor.setSelectedSensorPosition(0);
    RFAngleMotor.setSelectedSensorPosition(0);
    RFSpeedMotor.setSelectedSensorPosition(0);
    LRAngleMotor.setSelectedSensorPosition(0);
    LRSpeedMotor.setSelectedSensorPosition(0);
    RRAngleMotor.setSelectedSensorPosition(0);
    RRSpeedMotor.setSelectedSensorPosition(0);
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void PIDConfig(TalonFX motor) {
    motor.config_kP(0, Constants.DRIVEkP, 10);
    motor.config_kI(0, Constants.DRIVEkI, 10);
    motor.config_kD(0, Constants.DRIVEkD, 10);
  }
  
  public void drive(double x1,double  y1,double x2) {
    double a = x1 - x2 * (Constants.A_LENGTH / Constants.A_CROSSLENGTH);
    double b = x1 - x2 * (Constants.A_LENGTH / Constants.A_CROSSLENGTH);
    double c = y1 - x2 * (Constants.A_CROSSLENGTH / Constants.A_CROSSLENGTH);
    double d = y1 - x2 * (Constants.A_CROSSLENGTH / Constants.A_CROSSLENGTH);

    double RightRearVel = Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2));
    double LeftRearVel = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2));
    double RightFrontVel = Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2));
    double LeftFrontVel = Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2));

    double RightRearAngle = Math.atan2(a, d) / Math.PI;
    double LeftRearAngle = Math.atan2(a, c) / Math.PI;
    double RightFrontAngle = Math.atan2(b, d) / Math.PI;
    double LeftFrontAngle = Math.atan2(b, c) / Math.PI;

    LeftFront.drive(LeftFrontVel, LeftFrontAngle);
    RightFront.drive(RightFrontVel, RightFrontAngle);
    LeftRear.drive(LeftRearVel, LeftRearAngle);
    RightRear.drive(RightRearVel, RightRearAngle);
  }

  public void FODrive(double x1, double  y1, double angle) {
    angle = ((angle - getGyro()) / 180.0) - 1.0;
    double x = Math.cos(Math.atan(y1 / x1) + Math.toRadians(getGyro()));
    double y = Math.sin(Math.atan(y1 / x1) + Math.toRadians(getGyro()));

    drive(x, y, angle);
  }
  
  public double getGyro() {
    return gyro.getAngle();
  }
}
