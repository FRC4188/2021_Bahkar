package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;

public class WheelDrive {

  //Unassigned motor objects.
  private TalonFX angleMotor;
  private TalonFX speedMotor;

  public WheelDrive(TalonFX angleMotor, TalonFX speedMotor) {
    //Assign the motor objects.
    this.angleMotor = angleMotor;
    this.speedMotor = speedMotor;
  }

  /**
   * Method to set SwerveModules based on speed input between -1 and 1 and angle input between 0 and 360.
   * @param speed Speed of the wheel between -1 and 1.
   * @param angle Angle for wheel to be turned to.
   */
  public void drive(double speed, double angle) {
    //Convert -1 to 1 input to an output in counts per 100ms and set the drive motor to that speed.
    speed *= (Constants.DRIVE_COUNTS_PER_METER * Constants.DRIVE_MAX_VELOCITY) / 10;
    speedMotor.set(ControlMode.Velocity, speed);

    //Find how for through the current rotation the wheel is.
    double position = 360 % (angleMotor.getSelectedSensorPosition() / Constants.ANGLE_RATIO);
    //Find how many rotations the wheel has completed 
    double rotationsIn = ((angleMotor.getSelectedSensorPosition() / Constants.ANGLE_RATIO) - position) / 360;

    //Create an empty value for the real position to set the motor to.
    double SetAngle;
    //Decide which possible set position is closest to the current position and set the motor to that position.
    if ((position - angle) <= 180 && (position - angle) >= -180) SetAngle = (rotationsIn * 360) + angle;
    else if ((position - angle) > 180) SetAngle = ((rotationsIn + 1) * 360) + angle;
    else if ((position - angle) < -180) SetAngle = ((rotationsIn - 1) * 360) + angle;
    else SetAngle = 0;

    //Convert position in degrees to position in ticks and set the motor to that position.
    SetAngle *= Constants.ANGLE_RATIO;
    angleMotor.set(ControlMode.Position, SetAngle);
  }

  /**
  * Method to set SwerveModules based on speed input of M/S and angle input between 0 and 360.
  * @param speed Speed of the wheel in M/S.
  * @param angle Angle for wheel to be turned to.
  */
  public void convertedDrive(double speed, double angle) {
    //Convert M/S to ticks per 100ms and set motor to it.
    speed *= Constants.DRIVE_COUNTS_PER_METER / 10;
    speedMotor.set(ControlMode.Velocity, speed);

    //Find position of wheel in current rotation.
    double position = 360 % (angleMotor.getSelectedSensorPosition() / Constants.ANGLE_RATIO);
    //Find number of rotations taken so far.
    double rotationsIn = (360 / (angleMotor.getSelectedSensorPosition() / Constants.ANGLE_RATIO)) - position;

    //Create an empty value to hold set point.
    double SetAngle;

    //Decide which possible set angle is the closest.
    if ((position - angle) <= 180 && (position - angle) >= -180) SetAngle = (rotationsIn * 360) + angle;
    else if ((position - angle) > 180) SetAngle = ((rotationsIn + 1) * 360) + angle;
    else if ((position - angle) < -180) SetAngle = ((rotationsIn - 1) * 360) + angle;
    else SetAngle = 0;

    //Convert angle to encoder ticks and set the motor to that position.
    SetAngle *= Constants.ANGLE_RATIO;
    angleMotor.set(ControlMode.Position, SetAngle);
  }
}
