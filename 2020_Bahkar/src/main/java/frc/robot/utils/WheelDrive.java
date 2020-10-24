package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;

public class WheelDrive {

  private TalonFX angleMotor;
  private TalonFX speedMotor;

  public WheelDrive(TalonFX angleMotor, TalonFX speedMotor) {
    this.angleMotor = angleMotor;
    this.speedMotor = speedMotor;
  }

  public void drive(double speed, double angle) {
    speed *= (Constants.DRIVE_METERS_PER_ROTATION / 60) * Constants.DRIVE_MAX_VELOCITY;
    speedMotor.set(ControlMode.Velocity, speed);

    double position = 360 % (angleMotor.getSelectedSensorPosition() / Constants.ANGLE_RATIO);
    double rotationsIn = (360 / (angleMotor.getSelectedSensorPosition() / Constants.ANGLE_RATIO)) - position;

    double SetAngle;
    if ((position - angle) <= 180 && (position - angle) >= -180) SetAngle = (rotationsIn * 360) + angle;
    else if ((position - angle) > 180) SetAngle = ((rotationsIn + 1) * 360) + angle;
    else if ((position - angle) < -180) SetAngle = ((rotationsIn - 1) * 360) + angle;
    else SetAngle = 0;

    SetAngle *= Constants.ANGLE_GEARING * Constants.FALCON_ENCODER_TICKS;
    angleMotor.set(ControlMode.Position, SetAngle);
  }
}
