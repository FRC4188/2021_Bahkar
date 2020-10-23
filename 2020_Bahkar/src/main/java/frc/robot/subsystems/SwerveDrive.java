/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.WheelDrive;

public class SwerveDrive extends SubsystemBase {

  private Constants C;

  // device initialization
  private final TalonFX LFAngleMotor = new TalonFX(1);
  private final TalonFX LFSpeedMotor = new TalonFX(2);
  private final TalonFX RFAngleMotor = new TalonFX(3);
  private final TalonFX RFSpeedMotor = new TalonFX(4);
  private final TalonFX LRAngleMotor = new TalonFX(5);
  private final TalonFX LRSpeedMotor = new TalonFX(6);
  private final TalonFX RRAngleMotor = new TalonFX(7);
  private final TalonFX RRSpeedMotor = new TalonFX(8);

  private final WheelDrive LeftFront = new WheelDrive(LFAngleMotor, LFSpeedMotor);
  private final WheelDrive RightFront = new WheelDrive(RFAngleMotor, RFSpeedMotor);
  private final WheelDrive LeftRear = new WheelDrive(LRAngleMotor, LRSpeedMotor);
  private final WheelDrive RightRear = new WheelDrive(RRAngleMotor, RRSpeedMotor);

  private final PIDController LFAnglePID = new PIDController(C.DRIVEkP, C.DRIVEkI, C.DRIVEkD);
  private final PIDController RFAnglePID = new PIDController(C.DRIVEkP, C.DRIVEkI, C.DRIVEkD);
  private final PIDController LRAnglePID = new PIDController(C.DRIVEkP, C.DRIVEkI, C.DRIVEkD);
  private final PIDController RRAnglePID = new PIDController(C.DRIVEkP, C.DRIVEkI, C.DRIVEkD);

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
    PIDConfig(RFAngleMotor);
    PIDConfig(RFSpeedMotor);
    PIDConfig(LRAngleMotor);
    PIDConfig(LRSpeedMotor);
    PIDConfig(RRAngleMotor);
    PIDConfig(RRSpeedMotor);

    resetEncoders();
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

  public void PIDConfig(TalonFX motor) {
    motor.config_kP(0, C.DRIVEkP, 10);
    motor.config_kI(0, C.DRIVEkI, 10);
    motor.config_kD(0, C.DRIVEkD, 10);
  }
  
  public void drive(double x1,double  y1,double x2) {
    double a = x1 - x2 * (C.A_LENGTH / C.A_CROSSLENGTH);
    double b = x1 - x2 * (C.A_LENGTH / C.A_CROSSLENGTH);
    double c = y1 - x2 * (C.A_CROSSLENGTH / C.A_CROSSLENGTH);
    double d = y1 - x2 * (C.A_CROSSLENGTH / C.A_CROSSLENGTH);

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
}
