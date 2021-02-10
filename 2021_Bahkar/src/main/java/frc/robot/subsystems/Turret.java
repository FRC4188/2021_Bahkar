/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  Sensors sensors;

  // Motor control components.
  CANSparkMax turretMotor = new CANSparkMax(22, MotorType.kBrushless);
  CANEncoder turretEncoder = new CANEncoder(turretMotor);
  ProfiledPIDController pid = new ProfiledPIDController(Constants.Turret.kP, Constants.Turret.kI, Constants.Turret.kD,
                              new Constraints(Constants.Turret.MAX_VELOCITY, Constants.Turret.MAX_ACCELERATION));

  /**
   * Creates a new Turret.
   */
  public Turret(Sensors sensors) {
    this.sensors = sensors;

    controllerInit();
    resetEncoders(); //added this to be in line with other subsystems, if stuff mess up here then remove 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
  * Configures gains for Spark closed loop controller.
  */
  private void controllerInit() {
    pid.setP(Constants.Turret.kP);
    pid.setI(Constants.Turret.kI);
    pid.setD(Constants.Turret.kD);

    turretMotor.setClosedLoopRampRate(0.0);
    turretMotor.setOpenLoopRampRate(0.5);

    turretMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
  * Resets turret encoder value to 0.
  */
  public void resetEncoders() {
    turretEncoder.setPosition(0.0);
  }

  /**
  * Sets turret motor to given percentage [-1.0, 1.0].
  * @param percent The goal percentage to set the turret motor to.
  */
  public void set(double percent) {
    turretMotor.set(percent);
  }

  /**
   * Turns turret to angle in degrees.
   * @param angle Angle for the turret to move to.
   */
  public void setAngle(double angle) {
      angle /= Constants.Turret.ENCODER_TO_DEGREES;
      turretMotor.set(pid.calculate(turretEncoder.getPosition(), angle));
  }

  /**
   * Method to track the limelight target
   * @param cont whether to continue tracking or stop.
   */
  public void trackTarget(boolean cont) {
    double angle = sensors.getTurretHorizontalAngle();
    double offset = sensors.getTurretOffset();
    double power = pid.calculate(angle - offset, 0.0);
    
    set(power);
  }
  
  /**
   * Returns turret encoder position in degrees.
   * @return Degrees of the turret's current rotation.
   */
  public double getPosition() {
    return turretEncoder.getPosition() * Constants.Turret.ENCODER_TO_DEGREES;
  }

  /**
   * Returns turret encoder velocity in degrees per second.
   * @return Velocity of the turret in degrees per second.
   */
  public double getVelocity() {
      return turretEncoder.getVelocity() * Constants.Turret.ENCODER_TO_DEGREES / 60.0;
  }

  /**
   * Returns the temperature of the turret motor.
   * @return Motor temperature in celcius.
   */
  public double getTemp() {
    return turretMotor.getMotorTemperature();
  }


  /**
   * Method to determine if the turret is aimed at the limelight target.
   * @return Whether the turret is correctly aimed.
   */
  public boolean getIsAimed() {
    double angle = sensors.getTurretHorizontalAngle() - sensors.getTurretOffset();
    boolean aimed = (Math.abs(angle - turretEncoder.getPosition()) < Constants.Turret.POS_TOLERANCE) ? true : false;
    return aimed;
  }
}
