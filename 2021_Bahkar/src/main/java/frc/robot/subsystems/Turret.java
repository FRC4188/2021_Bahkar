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

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  Sensors sensors;
  //Drivetrain drivetrain;

  // Motor control components.
  CANSparkMax turretMotor = new CANSparkMax(42, MotorType.kBrushless);
  CANEncoder turretEncoder = new CANEncoder(turretMotor);
  ProfiledPIDController pid = new ProfiledPIDController(Constants.Turret.kP, Constants.Turret.kI, Constants.Turret.kD,
                              new Constraints(Constants.Turret.MAX_VELOCITY, Constants.Turret.MAX_ACCELERATION));

  Notifier shuffle;

  /**
   * Creates a new Turret.
   */
  public Turret(Sensors sensors /*Drivetrain drivetrain*/) {
    this.sensors = sensors;

    motorInits();
    resetEncoders(); //added this to be in line with other subsystems, if stuff mess up here then remove 

    shuffle = new Notifier(() -> updateShuffleboard());

    //this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
  * Configures gains for Spark closed loop controller.
  */
  private void motorInits() {
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

  private void updateShuffleboard() {
    SmartDashboard.putBoolean("Is Aimed", isAimed());
    SmartDashboard.putNumber("Turret Angle", getPosition());
  }

  public void closeNotifier() {
    shuffle.close();
  }

  public void openNotifier() {
    shuffle.startPeriodic(0.1);
  }

  /**
  * Sets turret motor to given percentage [-1.0, 1.0]. Will not allow turret to spin past the software limit
  * @param percent The goal percentage to set the turret motor to.
  */
  public void set(double percent) {
    turretMotor.set(getPosition() < Constants.Turret.MAX_ANG && getPosition() > Constants.Turret.MIN_ANG ? percent : percent > 0.0 ? percent : 0.0);
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
    double angle = sensors.getTurretHasTarget() ? sensors.getTurretHorizontalAngle() : sensors.getTurretHorizontalAngle() /*getPosition() +
      (sensors.getFusedHeading() - Math.toDegrees(
      Math.atan2(drivetrain.getPose().getY(), drivetrain.getPose().getX() - Constants.Field.GOAL_X_POS)))*/;
    double offset = sensors.getTurretHasTarget() ? sensors.getTurretOffset() : 0.0;
    double power = pid.calculate(angle - offset, 0.0);
    
    set(cont ? power : 0.0);
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
  public boolean isAimed() {
    double angle = sensors.getTurretHorizontalAngle() - sensors.getTurretOffset();
    boolean aimed = (Math.abs(angle - turretEncoder.getPosition()) < Constants.Turret.POS_TOLERANCE) ? true : false;
    return aimed;
  }
}
