package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CSPMath;


public class Hood extends SubsystemBase {

  private final Servo rightLinearServo = new Servo(69);
  private final Servo leftLinearServo = new Servo(69);
  
  public Hood() {
    rightLinearServo.set(0.0);
    leftLinearServo.set(0.0);
    rightLinearServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    leftLinearServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
  }

  @Override
  public void periodic() {
  //SmartDashboard.putNumber("Hood Position", getHoodPosition());
  }

  /**
   * Moves hood to linear servo position [0.0, 1.0]
   * @param position 
   */
  public void setHoodPosition(/*double position*/) {
    double position = SmartDashboard.getNumber("Set Hood Position", 0.0);
    rightLinearServo.set(position);
    leftLinearServo.set(position);
    //try setSpeed(), setPosition() or setAngle() if that doesn't work
  }

  public void setHoodAngle(double angle) {
    double position = CSPMath.angleToServoPosition(angle);
    
    rightLinearServo.set(position);
    leftLinearServo.set(position);
  }

  public void raiseHood(double incrementRate) {
    rightLinearServo.set(getHoodPosition() + incrementRate);
    leftLinearServo.set(getHoodPosition() + incrementRate);
  }

  public void lowerHood(double decrementRate) {
    rightLinearServo.set(getHoodPosition() - decrementRate);
    rightLinearServo.set(getHoodPosition() - decrementRate);
  }

  /**
   * @return the position of the right linear servo
   */
  public double getHoodPosition() {
    double hoodPosition = rightLinearServo.get() == leftLinearServo.get() ? rightLinearServo.get() : -1;
    
    return hoodPosition;
  }
}
