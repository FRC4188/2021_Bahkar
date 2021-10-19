/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.turret;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.Sensors;

public class Turret extends SubsystemBase {

  private static Turret instance;

  public synchronized static Turret getInstance() {
    if (instance == null) instance = new Turret();
    return instance;
  }

  Sensors sensors = Sensors.getInstance();
  Swerve drive = Swerve.getInstance();

  // Motor control components.
  CANSparkMax turretMotor = new CANSparkMax(42, MotorType.kBrushless);
  CANEncoder turretEncoder = turretMotor.getEncoder();
  PIDController pid = new PIDController(Constants.turret.kP, Constants.turret.kI, Constants.turret.kD);
  //ProfiledPIDController pid = new ProfiledPIDController(Constants.turret.kP, Constants.turret.kI, Constants.turret.kD,
                              //new Constraints(Constants.turret.MAX_VELOCITY, Constants.turret.MAX_ACCELERATION));

  Notifier shuffle;

  /**
   * Creates a new Turret.
   */
  private Turret() {
    CommandScheduler.getInstance().registerSubsystem(this);

    motorInits();
    resetEncoders(); //added this to be in line with other subsystems, if stuff mess up here then remove 

    shuffle = new Notifier(() -> updateShuffleboard());
    shuffle.startPeriodic(0.2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
  * Configures gains for Spark closed loop controller.
  */
  private void motorInits() {
    pid.setP(Constants.turret.kP);
    pid.setI(Constants.turret.kI);
    pid.setD(Constants.turret.kD);

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
    turretMotor.set(percent);
  }

  /**
   * Turns turret to angle in degrees.
   * @param angle Angle for the turret to move to.
   */
  public void setAngle(double angle) {
      angle /= Constants.turret.ENCODER_TO_DEGREES;
      turretMotor.set(pid.calculate(turretEncoder.getPosition(), angle));
  }

  /**
   * Method to track the limelight target
   * @param cont whether to continue tracking or stop.
   */
  public void trackTarget(boolean cont) {
    double angle = sensors.getTX();
    double power = pid.calculate(angle, 0.0) +
      Swerve.getInstance().getChassisSpeeds().omegaRadiansPerSecond / 10.0;;
    
    set(cont ? power : 0.0);
  }
  
  /**
   * Returns turret encoder position in degrees.
   * @return Degrees of the turret's current rotation.
   */
  public double getPosition() {
    return turretEncoder.getPosition() * Constants.turret.ENCODER_TO_DEGREES;
  }

  /**
   * Returns turret encoder velocity in degrees per second.
   * @return Velocity of the turret in degrees per second.
   */
  public double getVelocity() {
      return turretEncoder.getVelocity() * Constants.turret.ENCODER_TO_DEGREES / 60.0;
  }

  public double getTemperature() {
    return turretMotor.getMotorTemperature();
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
    double angle = sensors.getTX();
    boolean aimed = (Math.abs(angle) < Constants.turret.POS_TOLERANCE);
    return aimed;
  }
}