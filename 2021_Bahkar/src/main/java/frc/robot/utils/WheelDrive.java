package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class WheelDrive {

  //Unassigned motor objects.
  private TalonFX angleMotor;
  private TalonFX speedMotor;
  private CANCoder angleEncoder;

  private PIDController anglePID = new PIDController(0.00600, 0.0, 0.0);

  private double magOffset;
  private boolean right;

  public WheelDrive(TalonFX angleMotor, TalonFX speedMotor, CANCoder angleEncoder, double magOffset, boolean right, boolean back) {
    //Assign the motor objects.
    this.angleMotor = angleMotor;
    this.speedMotor = speedMotor;
    this.angleEncoder = angleEncoder;
    this.magOffset = magOffset;
    this.right = right;

    configMotors();
  }

    /**
   * Set properties of the motors.
   */
  private void configMotors() {
    speedMotor.configFactoryDefault();
    angleMotor.configFactoryDefault();

    speedMotor.setInverted(!right);

    //Select sensor for motors (integrated sensor).
    speedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    angleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

    speedMotor.setNeutralMode(NeutralMode.Brake);
    angleMotor.setNeutralMode(NeutralMode.Brake);

    //Call PIDConfig method for each motor and set ramp rates.
    PIDConfig();
    speedMotor.configClosedloopRamp(0.5);

    //Call reset methods.
    resetEncoders();
  }
  
  /**
   * Reset all encoders to position 0.
   */
  public void resetEncoders() {
    angleEncoder.configFactoryDefault();

    angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    angleEncoder.configMagnetOffset(magOffset);

    speedMotor.setSelectedSensorPosition(0);
    angleMotor.setSelectedSensorPosition(0);
  }

  /**
   * Method to set P, I, and D terms for drive motor closed loop control.
   * @param motor Motor which is being configured.
   */
  public void PIDConfig() {
    //Assign the values.
    speedMotor.config_kP(0, 0.23, 10);
    speedMotor.config_kI(0, 0.0, 10);
    speedMotor.config_kD(0, 0.0, 10);

    anglePID.enableContinuousInput(-180, 180);
  }

  public void convertedDrive(SwerveModuleState state) {
    //double desired = state.angle.getDegrees();
    double set = state.angle.getDegrees();
    double currentAngle = getAbsoluteAngle();

    //double set = CSPMath.minChange(desired, currentAngle, wrap) + currentAngle;

    angleMotor.set(ControlMode.PercentOutput, anglePID.calculate(currentAngle, set));

    double speed = (state.speedMetersPerSecond * (Constants.Drive.DRIVE_COUNTS_PER_METER / 10.0));

    //Convert M/S to ticks per 100ms and set motor to it.
    speedMotor.set(ControlMode.Velocity, speed);
  }

  /**
   * Return an updated module state for odometry purposes.
   * @return The updated module state with mesured rather than set values.
   */
  public SwerveModuleState updateModuleState() {
    double speed = (speedMotor.getSelectedSensorVelocity() * 10.0) / Constants.Drive.DRIVE_COUNTS_PER_METER;
    double angle = angleEncoder.getAbsolutePosition();

    return new SwerveModuleState(speed, new Rotation2d(Math.toRadians(angle)));
  }

  /**
   * Sets the invert for the speed motor.
   * @param invert True if inverted, false if not.
   */
  public void setInverted(boolean invert) {
    angleMotor.setInverted(invert);
  }

  /**
   * Runs just the angle motor to a specific angle.
   * @param angle Target angle.
   */
  public void setAngle(double angle) {
    angleMotor.set(ControlMode.Position, (angle * Constants.Drive.ANGLE_TICKS_PER_DEGREE));
  }

  /**
   * Runs just the velocity motor to a specific velocity.
   * @param speed Target Velocity.
   */
  public void setVelocity(double speed) {
    speedMotor.set(ControlMode.Velocity, ((speed * Constants.Drive.DRIVE_COUNTS_PER_METER) / 10.0));
  }

  /**
   * Adjust the PID gains for the angle motor.
   * @param kP P gain
   * @param kI I gain
   * @param kD D gain
   */
  public void setAnglePID(double kP, double kI, double kD) {
    anglePID.setPID(kP, kI, kD);
  }

  /**
   * Adjust the PID gains for the speed motor.
   * @param kP P gain
   * @param kI I gain
   * @param kD D gain
   */
  public void setSpeedPID(double kP, double kI, double kD) {
    speedMotor.config_kP(0, kP, 10);
    speedMotor.config_kI(0, kI, 10);
    speedMotor.config_kD(0, kD, 10);
  }

  /**
   * Measure the angle of the wheel where 0 points forward.
   * @return Absolute position (degrees).
   */
  public double getAbsoluteAngle() {
    return -angleEncoder.getAbsolutePosition();
  }

  /**
   * Returns the measured value where wrapping continues; can count 361 degrees but is reset every time the robot turns on.
   * @return The relative measurement (degrees).
   */
  public double getRelativeAngle() {
    return angleMotor.getSelectedSensorPosition();
  }

  /**
   * Returns the measured RPM of the speed motor.
   * @return Measured RPM.
   */
  public double getRPM() {
    return ((double) speedMotor.getSelectedSensorVelocity() * 10.0) / Constants.Robot.FALCON_ENCODER_TICKS;
  }

  /**
   * Returns the measured meters per second velocity of the speed motor.
   * @return Measured meters per second.
   */
  public double getMS() {
    return ((double) speedMotor.getSelectedSensorVelocity() * 10.0) / Constants.Drive.DRIVE_COUNTS_PER_METER;
  }
}
