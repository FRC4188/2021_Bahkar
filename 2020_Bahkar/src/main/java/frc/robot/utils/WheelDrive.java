package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;

public class WheelDrive {

  private TalonFX angleMotor;
  private TalonFX speedMotor;

  private Constants C;

  public WheelDrive(TalonFX angleMotor, TalonFX speedMotor) {
    this.angleMotor = angleMotor;
    this.speedMotor = speedMotor;
  }

  public void drive(double speed, double angle) {
    speed *= C.DRIVE_METERS_PER_ROTATION / 60;
    speedMotor.set(ControlMode.Velocity, speed);

    double position = 360 % (angleMotor.getSelectedSensorPosition() / C.ANGLE_RATIO);
    double rotationsIn = (360 / (angleMotor.getSelectedSensorPosition() / C.ANGLE_RATIO)) - position;

    double SetAngle;
    if ((position - angle) < 180.0) SetAngle = (angle + (rotationsIn * 360));
    else SetAngle = ((angle + (rotationsIn *360)) + 180);

    SetAngle *= C.ANGLE_GEARING * C.FALCON_ENCODER_TICKS;
    angleMotor.set(ControlMode.Position, SetAngle);
  }
}
