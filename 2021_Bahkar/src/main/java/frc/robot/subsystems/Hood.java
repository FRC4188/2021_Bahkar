package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CSPMath;


public class Hood extends SubsystemBase {

  private final Servo rightLinearServo = new Servo(0);
  private final Servo leftLinearServo = new Servo(1);
  
  public Hood() {
    rightLinearServo.set(0.0);
    leftLinearServo.set(0.0);
    rightLinearServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    leftLinearServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Right Hood Position", rightLinearServo.get());
    SmartDashboard.putNumber("Left Hood Position", leftLinearServo.get());
    SmartDashboard.putNumber("Hood Positon", rightLinearServo.get());

  }

  /**
   * Moves hood to linear servo position [0.0, 1.0]
   * @param position 
   */
  public void setHoodPosition(double position) {
    rightLinearServo.set(position);
    leftLinearServo.set(position);
    //try setSpeed(), setPosition() or setAngle() if that doesn't work
  }

  public void setHoodAngle(double angle) {
    double position = /*CSPMath.angleToServoPosition(angle);*/ angle;
    
    rightLinearServo.set(position);
    leftLinearServo.set(position);
  }

  public void toggleHood() {
    double rate = 0.1;

    if (rightLinearServo.get() == 1.0 && leftLinearServo.get() == 1.0) {
      rightLinearServo.set(rightLinearServo.get() - rate);
      leftLinearServo.set(leftLinearServo.get() - rate);
    } else if (rightLinearServo.get() == 0.0 && leftLinearServo.get() == 0.0) {
      rightLinearServo.set(rightLinearServo.get() + rate);
      leftLinearServo.set(leftLinearServo.get() + rate);
    }
  }

  public void raiseHood(double incrementRate) {
  
    rightLinearServo.set(rightLinearServo.get() + incrementRate);
    leftLinearServo.set(leftLinearServo.get() + incrementRate);
  }

  public void lowerHood(double decrementRate) {
    rightLinearServo.set(rightLinearServo.get() - decrementRate);
    leftLinearServo.set(leftLinearServo.get() - decrementRate);
  }

  /**
   * @return the position of the right linear servo
   */
  public double getHoodPosition() {
    double hoodPosition = rightLinearServo.get() == leftLinearServo.get() ? rightLinearServo.get() : -1;
    
    return hoodPosition;
  }

  public void setSpeed(double speed) {
    rightLinearServo.setSpeed(speed);
    leftLinearServo.setSpeed(speed);
  }

  public void holdPosition() {
    rightLinearServo.set(rightLinearServo.get());
    leftLinearServo.set(leftLinearServo.get());
  }
}
