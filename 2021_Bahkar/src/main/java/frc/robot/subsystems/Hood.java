package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Hood extends SubsystemBase {

  private final Servo linearServo = new Servo(1);
  private boolean atMax;
  
  public Hood() {
    linearServo.set(0.0);
    atMax = false;
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
    linearServo.set(SmartDashboard.getNumber("Set Hood Position", 0.0));
  }

  public void raiseHood(double changeRate) {
    linearServo.set(getHoodPosition() + changeRate);
  }

  public void lowerHood(double changeRate) {
    linearServo.set(getHoodPosition()- changeRate);
  }
  /**
   * @return the position of the linear servo
   */
  public double getHoodPosition() {
    return linearServo.get();
  }
}
