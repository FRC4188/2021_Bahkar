package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CSPMath;


public class Hood extends SubsystemBase {

  private final Servo rLinearServo = new Servo(0);
  private final Servo lLinearServo = new Servo(1);
  
  public Hood() {
    rLinearServo.set(0.0);
    lLinearServo.set(0.0);
    rLinearServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    lLinearServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Right Hood Position", getServoPositions()[1]);
    SmartDashboard.putNumber("Left Hood Position", getServoPositions()[0]);
    SmartDashboard.putNumber("Hood Positon", rLinearServo.get());
  }

  /**
   * Moves hood to linear servo position [0.0, 1.0]
   * 
   * @param position
   */
  public void setHoodPosition(double position) {
    rLinearServo.set(position);
    lLinearServo.set(position);
    // try setSpeed(), setPosition() or setAngle() if that doesn't work
  }

  public void setHoodAngle(double angle) {
    double position = /* CSPMath.angleToServoPosition(angle); */ angle;

    rLinearServo.set(position);
    lLinearServo.set(position);
  }

  public void cycleHood() {
    boolean isSamePosition = getServoPositions()[0] == getServoPositions()[1] ? true : false;
    int endCounter = 0;
    endCounter += isSamePosition && (getServoPositions()[0] == 0.0 || getServoPositions()[0] == 1.0) ? 1 : 0;

    if (endCounter % 2 == 0) {
      //lower hood 
      rLinearServo.set(getServoPositions()[1] - 0.1);
      lLinearServo.set(getServoPositions()[0] - 0.1);
    } else {
      //raise hood
      rLinearServo.set(getServoPositions()[1] + 0.1);
      lLinearServo.set(getServoPositions()[0] + 0.1);
    }
  }

  public void raiseHood(double incrementRate) {
    rLinearServo.set(rLinearServo.get() + incrementRate);
    lLinearServo.set(lLinearServo.get() + incrementRate);
  }

  public void lowerHood(double decrementRate) {
    rLinearServo.set(rLinearServo.get() - decrementRate);
    lLinearServo.set(lLinearServo.get() - decrementRate);
  }

  public void setSpeed(double speed) {
    rLinearServo.setSpeed(speed);
    lLinearServo.setSpeed(speed);
  }

  public double[] getServoPositions() {
    double[] servoPositions = {lLinearServo.get(), rLinearServo.get()};

    return servoPositions;
  }

  public void holdPosition() {
    rLinearServo.set(rLinearServo.get());
    lLinearServo.set(lLinearServo.get());
  }
}
