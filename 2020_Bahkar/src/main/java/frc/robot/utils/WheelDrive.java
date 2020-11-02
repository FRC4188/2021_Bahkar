package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
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

    double currentAngle = angleMotor.getSelectedSensorPosition() / Constants.ANGLE_RATIO;

    double position = 360 % currentAngle;
    double rotIn = (currentAngle - position) / 360;

    position = (position > 180) ? (-360 + position) : position;
    position = (position < -180) ? (360 + position) : position;

    double diff = position - angle;

    double SetAngle;

    if (diff < 90) {
      SetAngle = diff + (rotIn * 360);
    } else if (position < -90) {
      SetAngle = diff + ((rotIn - 1) * 360);
    } else if (position > 90) {
      SetAngle = diff + ((rotIn + 1) * 360);
    } else {
      SetAngle = 0;
    }

    //Convert angle to encoder ticks and set the motor to that position.
    SetAngle *= Constants.ANGLE_RATIO;
    angleMotor.set(ControlMode.Position, SetAngle);
  }

  public SwerveModuleState updateModuleState(SwerveModuleState state) {
    double Speed = speedMotor.getSelectedSensorVelocity() / (Constants.DRIVE_COUNTS_PER_METER / 10);
    double Angle = angleMotor.getSelectedSensorPosition() / Constants.ANGLE_RATIO;

    double position = 360 % Angle;

    position = (position > 180) ? (-360 + position) : position;
    position = (position < -180) ? (360 + position) : position;

    state.speedMetersPerSecond = Speed;
    state.angle = Rotation2d.fromDegrees(Angle);

    return state;
  }
}
