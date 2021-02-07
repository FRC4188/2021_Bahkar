package frc.robot.utils.components;

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
import frc.robot.utils.CSPMath;

public class WheelDrive {

  //Unassigned motor objects.
  private TalonFX angleMotor;
  private TalonFX speedMotor;
  private CANCoder angleEncoder;

  private PIDController anglePID = new PIDController(0.00600, 0.0, 0.0);

  private double magOffset;
  private boolean right;

  double wrap = 360.0;


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

    angleEncoder.configSensorDirection(false);

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
    speedMotor.config_kP(0, 0.2, 10);
    speedMotor.config_kI(0, 0.0, 10);
    speedMotor.config_kD(0, 0.0, 10);
  }

  public void convertedDrive(SwerveModuleState state) {
    double set = state.angle.getDegrees();
    double currentAngle = getAbsoluteAngle();

    if (set < 0) set += 360;
    if (Math.abs(currentAngle - set) > Math.abs(currentAngle - (set - 180))) set -= 180;
    else if (Math.abs(currentAngle - set) < Math.abs(currentAngle - (set - 180))) set += 180;
    else if (currentAngle == set || currentAngle == set - 180 || currentAngle == set + 180) set = currentAngle;
    if (set > 180) set -= 360;
    
    set = 0.0;//CSPMath.minChange(set, currentAngle, wrap) + currentAngle;

    angleMotor.set(ControlMode.PercentOutput, anglePID.calculate(currentAngle, set));

    //Convert M/S to ticks per 100ms and set motor to it.
    speedMotor.set(ControlMode.Velocity, (state.speedMetersPerSecond * (Constants.Drive.DRIVE_COUNTS_PER_METER / 10.0)));
  }

  public SwerveModuleState updateModuleState() {
    double speed = (speedMotor.getSelectedSensorVelocity() * 10.0) / Constants.Drive.DRIVE_COUNTS_PER_METER;
    double angle = angleEncoder.getAbsolutePosition();

    return new SwerveModuleState(speed, new Rotation2d(Math.toRadians(angle)));
  }

  public void setInverted(boolean invert) {
    angleMotor.setInverted(invert);
  }

  public void setAngle(double angle) {
    angleMotor.set(ControlMode.Position, (angle * Constants.Drive.ANGLE_TICKS_PER_DEGREE));
  }

  public void setVelocity(double speed) {
    speedMotor.set(ControlMode.Velocity, ((speed * Constants.Drive.DRIVE_COUNTS_PER_METER) / 10.0));
  }

  public void setAnglePID(double kP, double kI, double kD) {
    anglePID.setPID(kP, kI, kD);
  }

  public void setSpeedPID(double kP, double kI, double kD) {
    speedMotor.config_kP(0, kP, 10);
    speedMotor.config_kI(0, kI, 10);
    speedMotor.config_kD(0, kD, 10);
  }

  public double getAbsoluteAngle() {
    return angleEncoder.getAbsolutePosition();
  }

  public double getRelativeAngle() {
    return angleEncoder.getPosition();
  }

  public double getRPM() { 
    return ((double) speedMotor.getSelectedSensorVelocity() * 10.0) / Constants.Robot.FALCON_ENCODER_TICKS;
  }

  public double getMS() {
    return ((double) speedMotor.getSelectedSensorVelocity() * 10.0) / Constants.Drive.DRIVE_COUNTS_PER_METER;
  }
}
